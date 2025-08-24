# Transportadora (ESP32 + MQTT v5 + FreeRTOS 100ms) — VS Code + PlatformIO

Abrir esta carpeta en **Visual Studio Code** con la extensión **PlatformIO** instalada.

## Pasos
1) Abre VS Code → *Open Folder* → selecciona esta carpeta.
2) Edita `src/main.c` y cambia `WIFI_SSID` / `WIFI_PASS` y tu broker MQTT si aplica.
3) Conecta el ESP32 (board `esp32dev`).
4) En la barra de PlatformIO → **Build** → **Upload** → **Monitor**.

## Tópicos MQTT (base: `plant/transportadora/<chip_id>`)
- Comandos:
  - `…/cmd/start` — JSON opcional: `{"dir":"fwd|rev","speed":0..100}`
  - `…/cmd/stop`
  - `…/cmd/emergency_reset`
  - `…/cmd/set` — JSON: `{"speed":0..100,"led":true|false}`
- Publicaciones:
  - `…/state` — JSON de estado/entradas
  - `…/event` — eventos (RUN, STOP, LIMIT_*, etc.)

## Pines (recomendados, ESP32-WROOM)
- START_BTN=GPIO13, STOP_BTN=GPIO14, EMERG_BTN=GPIO27 (entradas con **pull-up interno**, **activo LOW**)
- LIMIT_FWD=GPIO33, LIMIT_REV=GPIO32
- LED_STATUS=GPIO19, LED_BUZZ(LED en vez de buzzer)=GPIO21
- HBRIDGE_IN1=GPIO18, HBRIDGE_IN2=GPIO17, PWM=GPIO5 (LEDC ch0, 20kHz)

> Botones/limit-switch: un lado a **GND**, el otro al GPIO. GND **común** entre fuente del motor y ESP32.

## Pruebas rápidas con `mosquitto_pub`
```bash
# Arrancar hacia adelante a 60%
mosquitto_pub -h test.mosquitto.org -t plant/transportadora/<ID>/cmd/start -m '{"dir":"fwd","speed":60}'

# Parar
mosquitto_pub -h test.mosquitto.org -t plant/transportadora/<ID>/cmd/stop -m ""

# Cambiar velocidad a 30% y encender LED
mosquitto_pub -h test.mosquitto.org -t plant/transportadora/<ID>/cmd/set -m '{"speed":30,"led":true}'
```