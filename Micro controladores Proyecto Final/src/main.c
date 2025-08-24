
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "cJSON.h"

static const char *TAG = "TRANSPORTADORA";

// ======= WiFi (edita credenciales) =======
#define WIFI_SSID      "TU_SSID"
#define WIFI_PASS      "TU_PASSWORD"

// ======= GPIO MAP =======
#define PIN_START_BTN      GPIO_NUM_13
#define PIN_STOP_BTN       GPIO_NUM_14
#define PIN_EMERG_BTN      GPIO_NUM_27
#define PIN_LIMIT_FWD      GPIO_NUM_33
#define PIN_LIMIT_REV      GPIO_NUM_32

#define PIN_LED_STATUS     GPIO_NUM_19
#define PIN_LED_BUZZ       GPIO_NUM_21  // LED sustituyendo buzzer

#define PIN_H_IN1          GPIO_NUM_18
#define PIN_H_IN2          GPIO_NUM_17
#define PIN_H_PWM          GPIO_NUM_5   // LEDC PWM

// ======= LEDC PWM =======
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_10_BIT // 0..1023
#define LEDC_FREQ_HZ        20000             // 20 kHz

// ======= TICK (FreeRTOS) =======
#define TICK_MS             100

// ======= FSM =======
typedef enum {
    ST_IDLE = 0,
    ST_RUN_FWD,
    ST_RUN_REV,
    ST_STOPPED,
    ST_EMERGENCY
} machine_state_t;

typedef enum {
    DIR_FWD = 0,
    DIR_REV = 1
} direction_t;

typedef struct {
    machine_state_t state;
    direction_t     dir;
    int             speed_pct;     // 0..100
    bool            btn_start;
    bool            btn_stop;
    bool            btn_emerg;
    bool            lim_fwd;
    bool            lim_rev;
} controller_t;

static controller_t ctrl = {
    .state = ST_IDLE,
    .dir = DIR_FWD,
    .speed_pct = 50
};

// ======= MQTT =======
static esp_mqtt_client_handle_t mqtt_client = NULL;
static char topic_base[64] = {0};
static char topic_cmd_start[96] = {0};
static char topic_cmd_stop[96] = {0};
static char topic_cmd_reset[96] = {0};
static char topic_cmd_set[96]   = {0};
static char topic_state[96]     = {0};
static char topic_event[96]     = {0};

// ======= Debounce =======
typedef struct { int stable; int count; } debounce_t;
static debounce_t db_start = {1, 0};
static debounce_t db_stop  = {1, 0};
static debounce_t db_emerg = {1, 0};
static debounce_t db_lfwd  = {1, 0};
static debounce_t db_lrev  = {1, 0};

static TimerHandle_t tick_timer;

// ======= Helpers =======
static void set_motor_outputs(direction_t dir, int speed_pct)
{
    gpio_set_level(PIN_H_IN1, (dir == DIR_FWD) ? 1 : 0);
    gpio_set_level(PIN_H_IN2, (dir == DIR_FWD) ? 0 : 1);

    if (speed_pct < 0) speed_pct = 0;
    if (speed_pct > 100) speed_pct = 100;
    int duty = (1023 * speed_pct) / 100;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void motor_stop(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    gpio_set_level(PIN_H_IN1, 0);
    gpio_set_level(PIN_H_IN2, 0);
}

static void publish_event(const char *msg)
{
    if (!mqtt_client) return;
    esp_mqtt_client_publish(mqtt_client, topic_event, msg, 0, 1, 0);
}

static void publish_state(void)
{
    if (!mqtt_client) return;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state",
        ctrl.state==ST_IDLE?"IDLE":
        ctrl.state==ST_RUN_FWD?"RUN_FWD":
        ctrl.state==ST_RUN_REV?"RUN_REV":
        ctrl.state==ST_STOPPED?"STOPPED":"EMERGENCY");
    cJSON_AddStringToObject(root, "dir", ctrl.dir==DIR_FWD?"FWD":"REV");
    cJSON_AddNumberToObject(root, "speed_pct", ctrl.speed_pct);

    cJSON *btns = cJSON_CreateObject();
    cJSON_AddBoolToObject(btns, "start", ctrl.btn_start);
    cJSON_AddBoolToObject(btns, "stop", ctrl.btn_stop);
    cJSON_AddBoolToObject(btns, "emerg", ctrl.btn_emerg);
    cJSON_AddItemToObject(root, "buttons", btns);

    cJSON *lims = cJSON_CreateObject();
    cJSON_AddBoolToObject(lims, "fwd", ctrl.lim_fwd);
    cJSON_AddBoolToObject(lims, "rev", ctrl.lim_rev);
    cJSON_AddItemToObject(root, "limits", lims);

    char *json = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(mqtt_client, topic_state, json, 0, 1, 1);
    cJSON_free(json);
    cJSON_Delete(root);
}

static void apply_state_machine(void)
{
    if (!ctrl.btn_emerg) { // activo LOW → emergencia
        ctrl.state = ST_EMERGENCY;
        motor_stop();
        gpio_set_level(PIN_LED_STATUS, 0);
        gpio_set_level(PIN_LED_BUZZ, 1);
        return;
    }
    gpio_set_level(PIN_LED_BUZZ, 0);

    switch (ctrl.state) {
        case ST_IDLE:
        case ST_STOPPED:
            motor_stop();
            gpio_set_level(PIN_LED_STATUS, 0);
            if (!ctrl.btn_start && ctrl.btn_stop) { // START=LOW y STOP=HIGH
                if (ctrl.dir == DIR_FWD && !ctrl.lim_fwd) break;
                if (ctrl.dir == DIR_REV && !ctrl.lim_rev) break;
                ctrl.state = (ctrl.dir == DIR_FWD) ? ST_RUN_FWD : ST_RUN_REV;
                publish_event("RUN");
            }
            break;

        case ST_RUN_FWD:
            if (!ctrl.btn_stop || !ctrl.btn_start) {
                ctrl.state = ST_STOPPED; publish_event("STOP"); break;
            }
            if (!ctrl.lim_fwd) {
                ctrl.state = ST_STOPPED; publish_event("LIMIT_FWD_REACHED"); break;
            }
            set_motor_outputs(DIR_FWD, ctrl.speed_pct);
            gpio_set_level(PIN_LED_STATUS, 1);
            break;

        case ST_RUN_REV:
            if (!ctrl.btn_stop || !ctrl.btn_start) {
                ctrl.state = ST_STOPPED; publish_event("STOP"); break;
            }
            if (!ctrl.lim_rev) {
                ctrl.state = ST_STOPPED; publish_event("LIMIT_REV_REACHED"); break;
            }
            set_motor_outputs(DIR_REV, ctrl.speed_pct);
            gpio_set_level(PIN_LED_STATUS, 1);
            break;

        case ST_EMERGENCY:
        default:
            motor_stop();
            gpio_set_level(PIN_LED_STATUS, 0);
            break;
    }
}

static void debounce_read(debounce_t *db, int level_now)
{
    if (level_now == db->stable) db->count = 0;
    else {
        if (++db->count >= 2) { // ~200 ms
            db->stable = level_now;
            db->count = 0;
        }
    }
}

static void scan_inputs(void)
{
    int s = gpio_get_level(PIN_START_BTN);
    int p = gpio_get_level(PIN_STOP_BTN);
    int e = gpio_get_level(PIN_EMERG_BTN);
    int lf = gpio_get_level(PIN_LIMIT_FWD);
    int lr = gpio_get_level(PIN_LIMIT_REV);

    debounce_read(&db_start, s);
    debounce_read(&db_stop, p);
    debounce_read(&db_emerg, e);
    debounce_read(&db_lfwd, lf);
    debounce_read(&db_lrev, lr);

    ctrl.btn_start = db_start.stable;
    ctrl.btn_stop  = db_stop.stable;
    ctrl.btn_emerg = db_emerg.stable;
    ctrl.lim_fwd   = db_lfwd.stable;
    ctrl.lim_rev   = db_lrev.stable;
}

static void tick_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    scan_inputs();
    apply_state_machine();
    static int cnt = 0;
    if (++cnt >= 10) { publish_state(); cnt = 0; } // cada 1 s
}

// ======= MQTT =======
static void mqtt_subscribe_all(void)
{
    esp_mqtt_client_subscribe(mqtt_client, topic_cmd_start, 1);
    esp_mqtt_client_subscribe(mqtt_client, topic_cmd_stop, 1);
    esp_mqtt_client_subscribe(mqtt_client, topic_cmd_reset, 1);
    esp_mqtt_client_subscribe(mqtt_client, topic_cmd_set, 1);
}

static void mqtt_on_cmd_start(const char *payload, int len)
{
    direction_t dir = ctrl.dir;
    int speed = ctrl.speed_pct;
    if (payload && len > 0) {
        cJSON *root = cJSON_ParseWithLength(payload, len);
        if (root) {
            cJSON *d = cJSON_GetObjectItem(root, "dir");
            if (cJSON_IsString(d)) {
                if (strcmp(d->valuestring, "fwd")==0) dir = DIR_FWD;
                else if (strcmp(d->valuestring, "rev")==0) dir = DIR_REV;
            }
            cJSON *sp = cJSON_GetObjectItem(root, "speed");
            if (cJSON_IsNumber(sp)) speed = (int)sp->valuedouble;
            cJSON_Delete(root);
        }
    }
    ctrl.dir = dir; ctrl.speed_pct = speed;
    if (ctrl.state == ST_EMERGENCY) { publish_event("IGNORED_START_EMERGENCY"); return; }
    ctrl.state = (dir == DIR_FWD) ? ST_RUN_FWD : ST_RUN_REV;
    publish_event("RUN_CMD");
}

static void mqtt_on_cmd_stop(void) { ctrl.state = ST_STOPPED; publish_event("STOP_CMD"); }
static void mqtt_on_cmd_reset(void)
{ if (ctrl.state == ST_EMERGENCY) { ctrl.state = ST_STOPPED; publish_event("EMERGENCY_RESET"); } }

static void mqtt_on_cmd_set(const char *payload, int len)
{
    if (!payload || len <= 0) return;
    cJSON *root = cJSON_ParseWithLength(payload, len); if (!root) return;
    cJSON *sp = cJSON_GetObjectItem(root, "speed");
    if (cJSON_IsNumber(sp)) ctrl.speed_pct = (int)sp->valuedouble;
    cJSON *led = cJSON_GetObjectItem(root, "led");
    if (cJSON_IsBool(led)) gpio_set_level(PIN_LED_STATUS, cJSON_IsTrue(led)?1:0);
    cJSON_Delete(root); publish_event("SET_APPLIED");
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            mqtt_subscribe_all();
            publish_event("MQTT_CONNECTED");
            publish_state();
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, topic_cmd_start, event->topic_len) == 0) mqtt_on_cmd_start(event->data, event->data_len);
            else if (strncmp(event->topic, topic_cmd_stop, event->topic_len) == 0) mqtt_on_cmd_stop();
            else if (strncmp(event->topic, topic_cmd_reset, event->topic_len) == 0) mqtt_on_cmd_reset();
            else if (strncmp(event->topic, topic_cmd_set, event->topic_len) == 0) mqtt_on_cmd_set(event->data, event->data_len);
            break;
        default: break;
    }
}

// ======= WiFi =======
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = { 0 };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ======= IO/PWM init =======
static void io_init(void)
{
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL<<PIN_START_BTN) | (1ULL<<PIN_STOP_BTN) | (1ULL<<PIN_EMERG_BTN) |
                        (1ULL<<PIN_LIMIT_FWD) | (1ULL<<PIN_LIMIT_REV),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_cfg);

    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL<<PIN_LED_STATUS) | (1ULL<<PIN_LED_BUZZ) |
                        (1ULL<<PIN_H_IN1) | (1ULL<<PIN_H_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_cfg);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num = PIN_H_PWM,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch);

    motor_stop();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    io_init();
    wifi_init();

    // construir base de tópicos con chip ID
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t chip = (mac[3]<<16) | (mac[4]<<8) | mac[5];
    snprintf(topic_base, sizeof(topic_base), "plant/transportadora/%06X", chip);
    snprintf(topic_cmd_start, sizeof(topic_cmd_start), "%s/cmd/start", topic_base);
    snprintf(topic_cmd_stop,  sizeof(topic_cmd_stop),  "%s/cmd/stop", topic_base);
    snprintf(topic_cmd_reset, sizeof(topic_cmd_reset), "%s/cmd/emergency_reset", topic_base);
    snprintf(topic_cmd_set,   sizeof(topic_cmd_set),   "%s/cmd/set", topic_base);
    snprintf(topic_state,     sizeof(topic_state),     "%s/state", topic_base);
    snprintf(topic_event,     sizeof(topic_event),     "%s/event", topic_base);

    // Broker MQTT v5 (cambia por el tuyo)
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://test.mosquitto.org",
        .session.protocol_ver = MQTT_PROTOCOL_V_5
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    // Timer 100 ms
    TimerHandle_t t = xTimerCreate("tick100", pdMS_TO_TICKS(TICK_MS), pdTRUE, NULL, tick_cb);
    tick_timer = t; xTimerStart(tick_timer, 0);

    ESP_LOGI(TAG, "Sistema iniciado. Topics base: %s", topic_base);
}
