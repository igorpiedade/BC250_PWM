# PWR_BC250 — ESP32 Power Controller

Firmware for the ESP32-based power controller for the ASRock BC250 + FlexATX PSU build.
It drives the motherboard power transistor (GPIO25), reads the board status signal (GPIO34),
controls a single ARGB LED (GPIO23), and exposes a WiFi WebUI plus an HTTP API for the OS.

## HTTP API

The API is available on the ESP32's IP address (port 80) once WiFi is connected.

### Authentication

All API endpoints require a Bearer API key:

```
Authorization: Bearer <API_KEY>
```

The key is 20 random alphanumeric characters, generated from the WebUI:
open the connected WebUI → **API Settings** panel → click **Set new key**.
The key is stored on the device (Preferences) and displayed in the panel.

Requests with a missing or wrong key receive:

```
HTTP 401
{"error":"unauthorized"}
```

### Endpoints

| Method | Route           | Description                                         |
| ------ | --------------- | --------------------------------------------------- |
| GET    | `/status`       | Device and power status                             |
| POST   | `/poweron`      | Turn the GPIO power driver ON                       |
| POST   | `/shutdown`     | Turn the GPIO power driver OFF                      |
| POST   | `/setosaddress` | Store the OS IP address for future use              |
| GET    | `/setLED`       | List all LED states (built-in + custom)             |
| POST   | `/setLED`       | Edit built-in / create-edit custom / activate state |

---

#### `GET /status`

Returns the current device state.

```bash
curl -H "Authorization: Bearer $KEY" http://<esp-ip>/status
```

```json
{
  "mainBoardSignal": true,
  "powerEnabled": true,
  "powerStatus": "ON",
  "wifiStatus": "Connected to MyNetwork (RSSI -52 dBm)",
  "ipAddress": "192.168.1.42"
}
```

#### `POST /poweron`

Enables the power driver (GPIO25). Idempotent.

```bash
curl -X POST -H "Authorization: Bearer $KEY" http://<esp-ip>/poweron
```

```json
{ "success": true, "changed": true, "powerEnabled": true, "powerStatus": "ON" }
```

Power-on requests are rejected while the safety lockout is active
(3 seconds after the driver turns OFF):

```
HTTP 409
{"success":false,"error":"power-on lockout active","lockoutRemainingSec":2}
```

#### `POST /shutdown`

Disables the power driver (GPIO25). Idempotent.

```bash
curl -X POST -H "Authorization: Bearer $KEY" http://<esp-ip>/shutdown
```

```json
{
  "success": true,
  "changed": true,
  "powerEnabled": false,
  "powerStatus": "OFF"
}
```

#### `POST /setosaddress`

Stores the OS IP address on the device (persisted across reboots) for future integrations.

| Parameter | Required | Description                        |
| --------- | -------- | ---------------------------------- |
| `ip`      | yes      | IPv4 address, e.g. `192.168.1.100` |

```bash
curl -X POST -H "Authorization: Bearer $KEY" "http://<esp-ip>/setosaddress?ip=192.168.1.100"
```

```json
{ "success": true, "osAddress": "192.168.1.100" }
```

Invalid or missing IP → `HTTP 400 {"success":false,"error":"invalid ip address"}`

#### `GET /setLED`

Lists all LED states and which custom state is active.

```bash
curl -H "Authorization: Bearer $KEY" http://<esp-ip>/setLED
```

```json
{
  "success": true,
  "activeCustom": "downloading",
  "states": [
    {
      "name": "booting",
      "builtin": true,
      "color": "#FF8C00",
      "intensity": 80,
      "breathing": true,
      "allowedOff": false
    },
    {
      "name": "standby",
      "builtin": true,
      "color": "#1E1E1E",
      "intensity": 30,
      "breathing": false,
      "allowedOff": true
    },
    {
      "name": "downloading",
      "builtin": false,
      "color": "#00FF00",
      "intensity": 60,
      "breathing": true,
      "allowedOff": true
    }
  ]
}
```

#### `POST /setLED` — edit a built-in state

The two automatic states are `booting` (GPIO25 ON) and `standby` (GPIO25 OFF).

| Parameter   | Required | Description                         |
| ----------- | -------- | ----------------------------------- |
| `preset`    | yes      | `booting` or `standby`              |
| `color`     | no       | `#RRGGBB` (URL-encode `#` as `%23`) |
| `intensity` | no       | `5`–`100`                           |
| `breathing` | no       | `true`/`false`                      |
| `allowoff`  | no       | `true`/`false`                      |

```bash
curl -X POST -H "Authorization: Bearer $KEY" \
  "http://<esp-ip>/setLED?preset=standby&color=%231E1E1E&intensity=30&allowoff=true"
```

#### `POST /setLED` — create or edit a custom state

Custom states are created in the WebUI (LED Management → **+ New State**) or here by name.
Up to 8 custom states are stored on the device.

| Parameter   | Required | Description                                  |
| ----------- | -------- | -------------------------------------------- |
| `name`      | yes      | 1–16 chars, unique (not `booting`/`standby`) |
| `color`     | no       | `#RRGGBB`                                    |
| `intensity` | no       | `5`–`100`                                    |
| `breathing` | no       | `true`/`false`                               |
| `allowoff`  | no       | `true` = may render while GPIO25 is OFF      |

```bash
curl -X POST -H "Authorization: Bearer $KEY" \
  "http://<esp-ip>/setLED?name=downloading&color=%2300FF00&intensity=60&breathing=true&allowoff=true"
```

```json
{
  "success": true,
  "state": "downloading",
  "color": "#00FF00",
  "intensity": 60,
  "breathing": true,
  "allowedOff": true
}
```

#### `POST /setLED` — activate / clear a state

Activating a custom state makes the LED show it (if the state is not `allowoff`, it only
renders while power is ON). Activating `booting`/`standby` or sending `clear=1` returns
to the automatic state.

```bash
# activate a custom state
curl -X POST -H "Authorization: Bearer $KEY" "http://<esp-ip>/setLED?activate=downloading"

# back to automatic
curl -X POST -H "Authorization: Bearer $KEY" "http://<esp-ip>/setLED?clear=1"
```

```json
{ "success": true, "activeCustom": "downloading" }
```

### Error responses

| Code | Cause                                        |
| ---- | -------------------------------------------- |
| 400  | Missing/invalid parameter                    |
| 401  | Missing or wrong `Authorization: Bearer` key |
| 409  | `/poweron` rejected: power-on lockout active |

## WebUI

- **Fallback AP mode** (SSID `SteamMachine`): WiFi scan/provisioning portal.
- **Connected mode**: login (`admin` / `admin250` on first boot, forced password change),
  power controls, per-state LED settings, firmware OTA upload, and API key management.
- The WebUI polls `/connected-status` (session-authenticated) — that endpoint is internal
  to the WebUI and not part of the API key flow.

## Build & flash

```bash
pio run                    # build
pio run --target upload    # flash over /dev/cu.SLAB_USBtoUART
```
