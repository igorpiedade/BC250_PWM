#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <DNSServer.h>
#include <mbedtls/sha256.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
// ESP32 Power Controller for ASRock BC250 + FlexATX PSU
// Behavior implemented:
// 1) Press button on GPIO18 -> enable transistor driver on GPIO25.
// 2) While motherboard status signal on GPIO34 is present, keep GPIO25 ON.
// 3) If GPIO34 signal disappears, wait 10 seconds and then turn GPIO25 OFF.
// 4) If button on GPIO18 is held for 5 seconds while ON, turn GPIO25 OFF.
// 5) After GPIO25 turns OFF, ignore power-on requests for 3 seconds.
// 6) ARGB on GPIO23:
//    - BOOTING: amber breathing between 40% and 80% for 15s after power-on.
//    - NORMAL: static white at 80%.
//    - SHUTTING_DOWN: blue breathing between 40% and 80% while GPIO34 signal is missing.
//      If GPIO34 signal is restored, return to NORMAL.
// 7) WiFi + Web UI:
//    - Try saved WiFi credentials in STA mode.
//    - Fallback AP "SteamMachine" is manual: hold GPIO18 for 15s while GPIO25 is OFF.
//      AP stays active for 10 minutes.
//    - Web UI allows scanning and connecting to local WiFi.
//    - Captive portal support in AP mode to prompt browser auto-open on many devices.

// ------------------------------
// Pin mapping
// ------------------------------
static const int PIN_TRANSISTOR_DRIVE = 25; // Output to 2N2222 base (through proper resistor)
static const int PIN_BUTTON_START = 18;     // Start button input
static const int PIN_MB_STATUS = 34;        // Input from BC250: signal present = board ON
static const int PIN_ARGB_DATA = 23;        // ARGB data output
static const int PIN_BOARD_LED = 2;         // ESP32 Dev Module onboard LED
static const uint16_t ARGB_LED_COUNT = 1;

// ------------------------------
// Input behavior configuration
// ------------------------------
// Set to LOW when using INPUT_PULLUP + button to GND.
// Set to HIGH if your touch module outputs HIGH when pressed.
static const int BUTTON_ACTIVE_LEVEL = LOW;

// Set to HIGH if GPIO34 is HIGH when motherboard is ON.
// Set to LOW if GPIO34 is LOW when motherboard is ON.
static const int MB_SIGNAL_PRESENT_LEVEL = HIGH;

static const unsigned long BUTTON_DEBOUNCE_MS = 40;
static const unsigned long BUTTON_ON_ARM_DELAY_AFTER_OFF_MS = 3000;
static const unsigned long BUTTON_HOLD_ARM_DELAY_MS = 3000;
static const unsigned long BUTTON_HOLD_TO_OFF_MS = 5000;
static const unsigned long SIGNAL_LOSS_TIMEOUT_MS = 10000;
static const unsigned long ARGB_BREATH_PERIOD_MS = 2500;
static const uint8_t ARGB_BREATH_MIN_PCT = 5;
static const uint8_t LED_MAX_CUSTOM_STATES = 8;
static const uint8_t LED_CUSTOM_NAME_MAX_LEN = 16;
static const uint8_t LED_ACTIVE_NONE = 0xFF;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;
static const unsigned long WIFI_FALLBACK_AP_HOLD_TO_ENABLE_MS = 15000;
static const unsigned long WIFI_FALLBACK_AP_ACTIVE_MS = 600000;
static const char* WIFI_FALLBACK_AP_SSID = "SteamMachine";
static const char* WIFI_PREF_NAMESPACE = "wifi";
static const char* WIFI_PREF_KEY_SSID = "ssid";
static const char* WIFI_PREF_KEY_PASS = "pass";
static const uint8_t WIFI_MAX_CONNECT_ATTEMPTS = 3;
static const char* WIFI_PREF_KEY_AUTH_HASH = "auth_hash";
static const char* WIFI_PREF_KEY_AUTH_FORCE = "auth_force";
static const char* WIFI_PREF_KEY_LED_BOOT = "led_boot";
static const char* WIFI_PREF_KEY_LED_NORMAL = "led_normal";
static const char* WIFI_PREF_KEY_LED_SHUTDOWN = "led_shutdown";
static const char* WIFI_PREF_KEY_LED_INTENSITY = "led_intensity";
static const char* WIFI_PREF_KEY_LED_BREATH = "led_breath";
static const char* WIFI_PREF_KEY_LED_BOOT_INTENSITY = "led_boot_i";
static const char* WIFI_PREF_KEY_LED_NORMAL_INTENSITY = "led_norm_i";
static const char* WIFI_PREF_KEY_LED_SHUTDOWN_INTENSITY = "led_shut_i";
static const char* WIFI_PREF_KEY_LED_BOOT_BREATH = "led_boot_b";
static const char* WIFI_PREF_KEY_LED_NORMAL_BREATH = "led_norm_b";
static const char* WIFI_PREF_KEY_LED_SHUTDOWN_BREATH = "led_shut_b";
static const char* WIFI_PREF_KEY_API_KEY = "api_key";
static const char* WIFI_PREF_KEY_OS_IP = "os_ip";
static const char* WIFI_PREF_KEY_OS_IP_TS = "os_ip_ts";
static const unsigned long OS_IP_MAX_AGE_MS = 24UL * 60UL * 60UL * 1000UL;
static const uint8_t API_KEY_LENGTH = 20;
static const char* AUTH_DEFAULT_USERNAME = "admin";
static const char* AUTH_DEFAULT_PASSWORD = "admin250";
static const char* AUTH_SESSION_COOKIE = "SMSESSION";

bool powerEnabled = false;
bool signalLossTimerRunning = false;
unsigned long signalLossStartedAt = 0;

// Simple debounced edge detection for GPIO18
int lastRawButton = HIGH;
int stableButton = HIGH;
unsigned long lastButtonChangeAt = 0;
bool buttonPressedEdge = false;
bool buttonReleasedEdge = false;

bool offHoldTimerRunning = false;
unsigned long offHoldStartedAt = 0;
unsigned long offHoldLastProgressSecond = 0;
unsigned long powerEnabledAt = 0;
bool powerOnLockoutActive = false;
unsigned long powerOnLockoutStartedAt = 0;

Preferences preferences;
WebServer webServer(80);
DNSServer dnsServer;

String savedWifiSsid;
String savedWifiPass;
unsigned long wifiConnectStartedAt = 0;
uint8_t wifiConnectAttempts = 0;
bool wifiAttemptLimitLogged = false;
bool wifiFallbackApEnabled = false;
bool wifiWasConnected = false;
bool webServerStarted = false;
bool dnsCaptivePortalEnabled = false;
unsigned long fallbackApExpiresAt = 0;
bool offButtonPressTracking = false;
unsigned long offButtonPressedAt = 0;
bool offButtonLongHoldHandled = false;
String authPasswordHash;
bool authForcePasswordChange = true;
String authSessionId;
String apiKey;
String osIpAddress;
unsigned long osIpAddressUpdatedAt = 0;
bool osEndpointReachable = false;
unsigned long osEndpointLastCheckedAt = 0;
String connectedUiNotice;
bool firmwareUploadAuthorized = false;
bool firmwareUploadStarted = false;
bool firmwareUploadComplete = false;
String firmwareUploadError;

struct RgbColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Automatic states: Booting (GPIO25 ON) and Standby (GPIO25 OFF).
// Custom states are stored by name in Preferences and activated by API.
RgbColor ledColorBooting = {255, 140, 0};
RgbColor ledColorStandby = {30, 30, 30};
uint8_t ledBootingIntensityPct = 80;
uint8_t ledStandbyIntensityPct = 30;
bool ledBootingBreathingEnabled = true;
bool ledStandbyBreathingEnabled = false;

// Custom states: parallel arrays persisted as led_c<i>_name/color/int/breath/allowoff.
String ledCustomNames[LED_MAX_CUSTOM_STATES];
RgbColor ledCustomColors[LED_MAX_CUSTOM_STATES];
uint8_t ledCustomIntensities[LED_MAX_CUSTOM_STATES];
bool ledCustomBreathings[LED_MAX_CUSTOM_STATES];
bool ledCustomAllowedOff[LED_MAX_CUSTOM_STATES];
uint8_t ledCustomCount = 0;
uint8_t ledActiveCustom = LED_ACTIVE_NONE;

void enablePowerDrive();
void disablePowerDrive();
String colorToHex(const RgbColor& color);

int findCustomStateIndex(const String& name) {
  for (uint8_t i = 0; i < ledCustomCount; ++i) {
    if (ledCustomNames[i].equalsIgnoreCase(name)) {
      return i;
    }
  }
  return -1;
}

void clearCustomStatePrefs(uint8_t index) {
  char key[16];
  snprintf(key, sizeof(key), "led_c%u_name", index);
  preferences.remove(key);
  snprintf(key, sizeof(key), "led_c%u_color", index);
  preferences.remove(key);
  snprintf(key, sizeof(key), "led_c%u_int", index);
  preferences.remove(key);
  snprintf(key, sizeof(key), "led_c%u_breath", index);
  preferences.remove(key);
  snprintf(key, sizeof(key), "led_c%u_allowoff", index);
  preferences.remove(key);
}

void saveCustomState(uint8_t index) {
  char key[16];
  snprintf(key, sizeof(key), "led_c%u_name", index);
  preferences.putString(key, ledCustomNames[index]);
  snprintf(key, sizeof(key), "led_c%u_color", index);
  preferences.putString(key, colorToHex(ledCustomColors[index]));
  snprintf(key, sizeof(key), "led_c%u_int", index);
  preferences.putUChar(key, ledCustomIntensities[index]);
  snprintf(key, sizeof(key), "led_c%u_breath", index);
  preferences.putBool(key, ledCustomBreathings[index]);
  snprintf(key, sizeof(key), "led_c%u_allowoff", index);
  preferences.putBool(key, ledCustomAllowedOff[index]);
  preferences.putUChar("led_c_count", ledCustomCount);
}

void removeCustomState(uint8_t index) {
  for (uint8_t i = index; i + 1 < ledCustomCount; ++i) {
    ledCustomNames[i] = ledCustomNames[i + 1];
    ledCustomColors[i] = ledCustomColors[i + 1];
    ledCustomIntensities[i] = ledCustomIntensities[i + 1];
    ledCustomBreathings[i] = ledCustomBreathings[i + 1];
    ledCustomAllowedOff[i] = ledCustomAllowedOff[i + 1];
  }
  --ledCustomCount;
  if (ledActiveCustom == index) {
    ledActiveCustom = LED_ACTIVE_NONE;
  } else if (ledActiveCustom > index && ledActiveCustom != LED_ACTIVE_NONE) {
    --ledActiveCustom;
  }
  for (uint8_t i = 0; i < ledCustomCount; ++i) {
    saveCustomState(i);
  }
  clearCustomStatePrefs(ledCustomCount);
  preferences.putUChar("led_c_count", ledCustomCount);
}

bool isMainboardSignalPresent() {
  return digitalRead(PIN_MB_STATUS) == MB_SIGNAL_PRESENT_LEVEL;
}

bool isOsIpAddressFresh() {
  if (osIpAddress.length() == 0 || osIpAddressUpdatedAt == 0) {
    return false;
  }
  return (millis() - osIpAddressUpdatedAt) <= OS_IP_MAX_AGE_MS;
}

bool refreshOsEndpointReachability() {
  if (!isMainboardSignalPresent()) {
    osEndpointReachable = false;
    return false;
  }

  if (osIpAddress.length() == 0 || !isOsIpAddressFresh()) {
    osEndpointReachable = false;
    return false;
  }

  unsigned long now = millis();
  if (osEndpointLastCheckedAt != 0 && (now - osEndpointLastCheckedAt) < (5UL * 60UL * 1000UL)) {
    return osEndpointReachable;
  }
  osEndpointLastCheckedAt = now;

  WiFiClient client;
  const uint32_t connectTimeoutMs = 1000;
  const uint32_t responseTimeoutMs = 1500;

  unsigned long connectStart = millis();
  while (millis() - connectStart < connectTimeoutMs) {
    if (client.connect(osIpAddress.c_str(), 8765)) {
      break;
    }
    delay(20);
  }

  if (!client.connected()) {
    osEndpointReachable = false;
    client.stop();
    return false;
  }

  client.setTimeout(1000);
  client.print(F("GET /autodiscover HTTP/1.1\r\n"));
  client.print(F("Host: "));
  client.print(osIpAddress);
  client.print(F("\r\n"));
  client.print(F("Connection: close\r\n\r\n"));

  unsigned long start = millis();
  String response;
  response.reserve(256);
  while (millis() - start < responseTimeoutMs) {
    while (client.available()) {
      char c = static_cast<char>(client.read());
      response += c;
      if (response.length() >= 512) {
        break;
      }
    }

    if (response.length() > 0) {
      break;
    }
    delay(10);
  }

  client.stop();

  bool httpOk = response.indexOf("HTTP/1.1 200") >= 0 || response.indexOf("HTTP/1.0 200") >= 0;
  bool statusOk = response.indexOf("\"status\":\"ok\"") >= 0 || response.indexOf("\"status\": \"ok\"") >= 0 ||
                  response.indexOf("\"status\":\"OK\"") >= 0 || response.indexOf("\"status\": \"OK\"") >= 0;

  osEndpointReachable = httpOk && statusOk;
  return osEndpointReachable;
}

void beginWifiConnection(const String& ssid, const String& pass) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  wifiConnectStartedAt = millis();

  Serial.print("[WIFI] Connecting to ");
  Serial.println(ssid);
}

bool trySavedWifiConnection() {
  if (savedWifiSsid.length() == 0) {
    return false;
  }

  if (wifiConnectAttempts >= WIFI_MAX_CONNECT_ATTEMPTS) {
    if (!wifiAttemptLimitLogged) {
      Serial.print("[WIFI] Reached max connect attempts (");
      Serial.print(WIFI_MAX_CONNECT_ATTEMPTS);
      Serial.println("). Stopping retries.");
      wifiAttemptLimitLogged = true;
    }
    return false;
  }

  ++wifiConnectAttempts;
  Serial.print("[WIFI] Connect attempt ");
  Serial.print(wifiConnectAttempts);
  Serial.print("/");
  Serial.println(WIFI_MAX_CONNECT_ATTEMPTS);
  beginWifiConnection(savedWifiSsid, savedWifiPass);
  return true;
}

String htmlEscape(const String& value) {
  String escaped = value;
  escaped.replace("&", "&amp;");
  escaped.replace("<", "&lt;");
  escaped.replace(">", "&gt;");
  escaped.replace("\"", "&quot;");
  escaped.replace("'", "&#39;");
  return escaped;
}

String jsonEscape(const String& value) {
  String escaped = value;
  escaped.replace("\\", "\\\\");
  escaped.replace("\"", "\\\"");
  escaped.replace("\n", "\\n");
  escaped.replace("\r", "\\r");
  escaped.replace("\t", "\\t");
  return escaped;
}

String colorToHex(const RgbColor& color) {
  char hex[8];
  snprintf(hex, sizeof(hex), "#%02X%02X%02X", color.r, color.g, color.b);
  return String(hex);
}

int hexNibble(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return 10 + (c - 'a');
  }
  if (c >= 'A' && c <= 'F') {
    return 10 + (c - 'A');
  }
  return -1;
}

bool parseHexColor(const String& value, RgbColor& outColor) {
  if (value.length() != 7 || value.charAt(0) != '#') {
    return false;
  }

  int v[6];
  for (int i = 0; i < 6; ++i) {
    v[i] = hexNibble(value.charAt(i + 1));
    if (v[i] < 0) {
      return false;
    }
  }

  outColor.r = static_cast<uint8_t>((v[0] << 4) | v[1]);
  outColor.g = static_cast<uint8_t>((v[2] << 4) | v[3]);
  outColor.b = static_cast<uint8_t>((v[4] << 4) | v[5]);
  return true;
}

RgbColor loadLedColorPreference(const char* prefKey, const RgbColor& defaultColor) {
  if (!preferences.isKey(prefKey)) {
    preferences.putString(prefKey, colorToHex(defaultColor));
    return defaultColor;
  }

  String saved = preferences.getString(prefKey, "");
  RgbColor parsed;
  if (parseHexColor(saved, parsed)) {
    return parsed;
  }

  preferences.putString(prefKey, colorToHex(defaultColor));
  return defaultColor;
}

uint8_t clampLedIntensity(int value) {
  if (value < ARGB_BREATH_MIN_PCT) {
    return ARGB_BREATH_MIN_PCT;
  }
  if (value > 100) {
    return 100;
  }
  return static_cast<uint8_t>(value);
}

String sha256Hex(const String& input) {
  unsigned char digest[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, reinterpret_cast<const unsigned char*>(input.c_str()), input.length());
  mbedtls_sha256_finish(&ctx, digest);
  mbedtls_sha256_free(&ctx);

  char hex[65];
  for (size_t i = 0; i < sizeof(digest); ++i) {
    snprintf(&hex[i * 2], 3, "%02x", digest[i]);
  }
  hex[64] = '\0';
  return String(hex);
}

String authSalt() {
  uint64_t chipId = ESP.getEfuseMac();
  char id[17];
  snprintf(id, sizeof(id), "%04X%08X", static_cast<uint16_t>(chipId >> 32), static_cast<uint32_t>(chipId));
  return String("steam-machine:") + id;
}

String hashPassword(const String& password) {
  return sha256Hex(authSalt() + ":" + password);
}

String generateSessionId() {
  uint32_t a = esp_random();
  uint32_t b = esp_random();
  uint32_t c = static_cast<uint32_t>(millis());
  char sessionId[25];
  snprintf(sessionId, sizeof(sessionId), "%08lx%08lx%08lx", static_cast<unsigned long>(a), static_cast<unsigned long>(b), static_cast<unsigned long>(c));
  return String(sessionId);
}

String generateApiKey() {
  static const char* API_KEY_CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  static const uint8_t API_KEY_CHARS_COUNT = 62;
  String key;
  key.reserve(API_KEY_LENGTH);
  for (uint8_t i = 0; i < API_KEY_LENGTH; ++i) {
    key += API_KEY_CHARS[esp_random() % API_KEY_CHARS_COUNT];
  }
  return key;
}

String getCookieValue(const String& key) {
  if (!webServer.hasHeader("Cookie")) {
    return "";
  }

  String cookies = webServer.header("Cookie");
  int start = 0;
  while (start < cookies.length()) {
    int end = cookies.indexOf(';', start);
    if (end < 0) {
      end = cookies.length();
    }

    String part = cookies.substring(start, end);
    part.trim();
    int eq = part.indexOf('=');
    if (eq > 0) {
      String name = part.substring(0, eq);
      String value = part.substring(eq + 1);
      name.trim();
      value.trim();
      if (name == key) {
        return value;
      }
    }

    start = end + 1;
  }

  return "";
}

void sendRedirect(const String& location) {
  webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.sendHeader("Location", location, true);
  webServer.send(302, "text/plain", "");
}

bool hasValidApiKey() {
  if (apiKey.length() == 0) {
    return false;
  }

  if (!webServer.hasHeader("Authorization")) {
    return false;
  }

  String expected = "Bearer " + apiKey;
  return webServer.header("Authorization") == expected;
}

void sendApiJson(int code, const String& json) {
  webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.send(code, "application/json", json);
}

void sendApiUnauthorized() {
  sendApiJson(401, "{\"error\":\"unauthorized\"}");
}

bool isConnectedModeWebUi() {
  return !wifiFallbackApEnabled && WiFi.status() == WL_CONNECTED;
}

bool isAuthenticated() {
  if (authSessionId.length() == 0) {
    return false;
  }

  String cookieValue = getCookieValue(AUTH_SESSION_COOKIE);
  return cookieValue.length() > 0 && cookieValue == authSessionId;
}

void setSessionCookie(const String& sessionId) {
  webServer.sendHeader("Set-Cookie", String(AUTH_SESSION_COOKIE) + "=" + sessionId + "; Path=/; HttpOnly; SameSite=Lax");
}

void clearSessionCookie() {
  authSessionId = "";
  webServer.sendHeader("Set-Cookie", String(AUTH_SESSION_COOKIE) + "=; Path=/; Max-Age=0; HttpOnly; SameSite=Lax");
}

bool requireAuthForConnectedUi() {
  if (!isConnectedModeWebUi()) {
    return false;
  }

  if (!isAuthenticated()) {
    sendRedirect("/login");
    return true;
  }

  if (authForcePasswordChange && webServer.uri() != "/change-password") {
    sendRedirect("/change-password");
    return true;
  }

  return false;
}

void enableFallbackAp() {
  if (wifiFallbackApEnabled) {
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  if (WiFi.softAP(WIFI_FALLBACK_AP_SSID)) {
    wifiFallbackApEnabled = true;
    dnsServer.start(53, "*", WiFi.softAPIP());
    dnsCaptivePortalEnabled = true;
    Serial.print("[WIFI] Fallback AP enabled: ");
    Serial.println(WIFI_FALLBACK_AP_SSID);
    Serial.print("[WIFI] AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("[WIFI] Captive portal DNS enabled");
  } else {
    Serial.println("[WIFI] Failed to start fallback AP");
  }
}

void disableFallbackAp() {
  if (!wifiFallbackApEnabled) {
    return;
  }

  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  wifiFallbackApEnabled = false;
  if (dnsCaptivePortalEnabled) {
    dnsServer.stop();
    dnsCaptivePortalEnabled = false;
  }
  fallbackApExpiresAt = 0;
  Serial.println("[WIFI] Fallback AP disabled");
}

void enableTimedFallbackApWindow() {
  if (!wifiFallbackApEnabled) {
    enableFallbackAp();
  }

  fallbackApExpiresAt = millis() + WIFI_FALLBACK_AP_ACTIVE_MS;
  Serial.println("[WIFI] Fallback AP window active for 10 minutes");
}

String wifiModeLabel() {
  if (WiFi.status() == WL_CONNECTED) {
    return String("Connected to ") + WiFi.SSID() + " (" + WiFi.localIP().toString() + ")";
  }

  if (wifiFallbackApEnabled) {
    return String("Fallback AP active: ") + WIFI_FALLBACK_AP_SSID + " (" + WiFi.softAPIP().toString() + ")";
  }

  return "Not connected";
}

void renderLoginPage(const String& errorMessage = "") {
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>STEAM MACHINE - Login</title>";
  html += "<style>";
  html += "*{box-sizing:border-box;}";
  html += "body{margin:0;min-height:100vh;font-family:'Trebuchet MS',Verdana,sans-serif;padding:48px 18px 20px;";
  html += "background:linear-gradient(180deg,#1f2b5b 0%,#1a234b 58%,#171d38 100%);color:#fff;}";
  html += ".shell{max-width:460px;margin:0 auto;}";
  html += ".title{text-align:center;font-size:40px;font-weight:900;letter-spacing:1.2px;margin:8px 0;}";
  html += ".sub{text-align:center;color:rgba(255,255,255,.84);margin:0 0 24px;font-size:14px;}";
  html += ".panel{background:rgba(10,16,40,.38);border:1px solid rgba(255,255,255,.16);border-radius:14px;padding:16px;}";
  html += "label{display:block;font-size:13px;margin:8px 0 6px;color:#dce8ff;}";
  html += "input{width:100%;padding:12px 10px;border-radius:8px;border:1px solid rgba(255,255,255,.24);background:#101733;color:#fff;}";
  html += ".btn{display:block;width:100%;margin-top:14px;border:none;border-radius:10px;padding:12px 14px;background:#3c8ef4;color:#fff;font-weight:800;font-size:18px;cursor:pointer;}";
  html += ".err{background:rgba(255,68,68,.16);border:1px solid rgba(255,90,90,.38);padding:10px 12px;border-radius:10px;color:#ffd7d7;font-size:13px;margin-bottom:12px;}";
  html += "</style></head><body><main class='shell'>";
  html += "<h1 class='title'>STEAM MACHINE</h1>";
  html += "<p class='sub'>Login to access connected mode settings</p>";
  html += "<section class='panel'>";
  if (errorMessage.length() > 0) {
    html += "<div class='err'>" + htmlEscape(errorMessage) + "</div>";
  }
  html += "<form method='POST' action='/login'>";
  html += "<label for='username'>Username</label><input id='username' name='username' value='admin' required>";
  html += "<label for='password'>Password</label><input id='password' name='password' type='password' required>";
  html += "<button class='btn' type='submit'>LOGIN</button></form></section></main></body></html>";
  webServer.send(200, "text/html", html);
}

void renderChangePasswordPage(const String& errorMessage = "", const String& infoMessage = "") {
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>STEAM MACHINE - Change Password</title>";
  html += "<style>";
  html += "*{box-sizing:border-box;}";
  html += "body{margin:0;min-height:100vh;font-family:'Trebuchet MS',Verdana,sans-serif;padding:48px 18px 20px;";
  html += "background:linear-gradient(180deg,#1f2b5b 0%,#1a234b 58%,#171d38 100%);color:#fff;}";
  html += ".shell{max-width:500px;margin:0 auto;}";
  html += ".title{text-align:center;font-size:38px;font-weight:900;letter-spacing:1.2px;margin:8px 0;}";
  html += ".sub{text-align:center;color:rgba(255,255,255,.84);margin:0 0 24px;font-size:14px;}";
  html += ".panel{background:rgba(10,16,40,.38);border:1px solid rgba(255,255,255,.16);border-radius:14px;padding:16px;}";
  html += "label{display:block;font-size:13px;margin:8px 0 6px;color:#dce8ff;}";
  html += "input{width:100%;padding:12px 10px;border-radius:8px;border:1px solid rgba(255,255,255,.24);background:#101733;color:#fff;}";
  html += ".btn{display:block;width:100%;margin-top:14px;border:none;border-radius:10px;padding:12px 14px;background:#3c8ef4;color:#fff;font-weight:800;font-size:18px;cursor:pointer;}";
  html += ".msg{padding:10px 12px;border-radius:10px;font-size:13px;margin-bottom:12px;}";
  html += ".err{background:rgba(255,68,68,.16);border:1px solid rgba(255,90,90,.38);color:#ffd7d7;}";
  html += ".ok{background:rgba(64,184,114,.15);border:1px solid rgba(111,239,162,.35);color:#d6ffe6;}";
  html += "</style></head><body><main class='shell'>";
  html += "<h1 class='title'>STEAM MACHINE</h1>";
  html += "<p class='sub'>First login detected. Please set a new admin password.</p>";
  html += "<section class='panel'>";
  if (infoMessage.length() > 0) {
    html += "<div class='msg ok'>" + htmlEscape(infoMessage) + "</div>";
  }
  if (errorMessage.length() > 0) {
    html += "<div class='msg err'>" + htmlEscape(errorMessage) + "</div>";
  }
  html += "<form method='POST' action='/change-password'>";
  html += "<label for='current'>Current password</label><input id='current' name='current_password' type='password' required>";
  html += "<label for='next'>New password</label><input id='next' name='new_password' type='password' minlength='8' required>";
  html += "<label for='confirm'>Confirm new password</label><input id='confirm' name='confirm_password' type='password' minlength='8' required>";
  html += "<button class='btn' type='submit'>UPDATE PASSWORD</button></form></section></main></body></html>";
  webServer.send(200, "text/html", html);
}

void handleLoginPage() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  if (isAuthenticated()) {
    if (authForcePasswordChange) {
      sendRedirect("/change-password");
    } else {
      sendRedirect("/");
    }
    return;
  }

  renderLoginPage();
}

void handleLoginSubmit() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  String username = webServer.hasArg("username") ? webServer.arg("username") : String();
  String password = webServer.hasArg("password") ? webServer.arg("password") : String();

  if (username != AUTH_DEFAULT_USERNAME || hashPassword(password) != authPasswordHash) {
    renderLoginPage("Invalid username or password.");
    return;
  }

  authSessionId = generateSessionId();
  setSessionCookie(authSessionId);
  if (authForcePasswordChange) {
    sendRedirect("/change-password");
    return;
  }

  sendRedirect("/");
}

void handleChangePasswordPage() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  if (!isAuthenticated()) {
    sendRedirect("/login");
    return;
  }

  renderChangePasswordPage();
}

void handleChangePasswordSubmit() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  if (!isAuthenticated()) {
    sendRedirect("/login");
    return;
  }

  String currentPassword = webServer.hasArg("current_password") ? webServer.arg("current_password") : String();
  String newPassword = webServer.hasArg("new_password") ? webServer.arg("new_password") : String();
  String confirmPassword = webServer.hasArg("confirm_password") ? webServer.arg("confirm_password") : String();

  if (hashPassword(currentPassword) != authPasswordHash) {
    renderChangePasswordPage("Current password is incorrect.");
    return;
  }

  if (newPassword.length() < 8) {
    renderChangePasswordPage("New password must have at least 8 characters.");
    return;
  }

  if (newPassword != confirmPassword) {
    renderChangePasswordPage("New password and confirmation do not match.");
    return;
  }

  authPasswordHash = hashPassword(newPassword);
  authForcePasswordChange = false;
  preferences.putString(WIFI_PREF_KEY_AUTH_HASH, authPasswordHash);
  preferences.putBool(WIFI_PREF_KEY_AUTH_FORCE, authForcePasswordChange);

  sendRedirect("/");
}

void handleLogout() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  clearSessionCookie();
  sendRedirect("/login");
}

bool requestOsGracefulShutdown() {
  if (!isMainboardSignalPresent() || osIpAddress.length() == 0 || !isOsIpAddressFresh()) {
    return false;
  }

  WiFiClient client;
  if (!client.connect(osIpAddress.c_str(), 8765)) {
    Serial.println("[POWER] Graceful shutdown request failed: OS not reachable on port 8765");
    return false;
  }

  String payload = "{\"action\":\"poweroff\"}";
  String request = "POST /powermgt HTTP/1.1\r\n";
  request += "Host: " + osIpAddress + ":8765\r\n";
  request += "Connection: close\r\n";
  request += "Content-Type: application/json\r\n";
  request += "Content-Length: " + String(payload.length()) + "\r\n\r\n";
  request += payload;

  client.print(request);

  unsigned long start = millis();
  String response;
  response.reserve(256);
  while (millis() - start < 2000) {
    while (client.available()) {
      response += static_cast<char>(client.read());
      if (response.length() >= 512) {
        break;
      }
    }
    if (response.length() > 0) {
      break;
    }
    delay(10);
  }

  client.stop();

  bool httpOk = response.indexOf("HTTP/1.1 200") >= 0 || response.indexOf("HTTP/1.1 202") >= 0 ||
                response.indexOf("HTTP/1.0 200") >= 0 || response.indexOf("HTTP/1.0 202") >= 0;
  bool bodyOk = response.indexOf("\"success\":true") >= 0 || response.indexOf("\"status\":\"ok\"") >= 0 ||
                response.indexOf("\"status\": \"ok\"") >= 0 || response.indexOf("\"action\":\"poweroff\"") >= 0;

  if (httpOk && (bodyOk || response.indexOf("poweroff") >= 0)) {
    Serial.println("[POWER] Graceful OS shutdown request accepted");
    return true;
  }

  Serial.print("[POWER] Graceful shutdown rejected by OS: ");
  Serial.println(response.substring(0, min((unsigned int)160, response.length())));
  return false;
}

void handlePowerToggle() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  if (!isAuthenticated()) {
    sendRedirect("/login");
    return;
  }

  if (authForcePasswordChange) {
    sendRedirect("/change-password");
    return;
  }

  String action = webServer.hasArg("action") ? webServer.arg("action") : String("toggle");
  if (action != "on" && action != "off" && action != "toggle" && action != "force-off") {
    connectedUiNotice = "Invalid power action.";
    sendRedirect("/");
    return;
  }

  if (action == "force-off") {
    if (powerEnabled) {
      disablePowerDrive();
      connectedUiNotice = "Power output forced off.";
    } else {
      connectedUiNotice = "Power output is already disabled.";
    }
    sendRedirect("/");
    return;
  }

  bool shouldEnable = action == "on" || (action == "toggle" && !powerEnabled);

  if (shouldEnable) {
    if (powerOnLockoutActive) {
      unsigned long elapsed = millis() - powerOnLockoutStartedAt;
      if (elapsed >= BUTTON_ON_ARM_DELAY_AFTER_OFF_MS) {
        powerOnLockoutActive = false;
      } else {
        unsigned long remainingMs = BUTTON_ON_ARM_DELAY_AFTER_OFF_MS - elapsed;
        connectedUiNotice = "Power-on lockout active for " + String((remainingMs + 999) / 1000) + "s.";
        sendRedirect("/");
        return;
      }
    }

    if (!powerEnabled) {
      enablePowerDrive();
      connectedUiNotice = "Power output enabled.";
    } else {
      connectedUiNotice = "Power output is already enabled.";
    }
  } else {
    if (!powerEnabled) {
      connectedUiNotice = "Power output is already disabled.";
      sendRedirect("/");
      return;
    }

    if (requestOsGracefulShutdown()) {
      connectedUiNotice = "Shutdown requested to OS.";
    } else {
      connectedUiNotice = "OS shutdown request failed. Hold for 5s to force power off.";
    }
  }

  sendRedirect("/");
}

void handleConnectedStatus() {
  if (!isConnectedModeWebUi() || !isAuthenticated() || authForcePasswordChange) {
    webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    webServer.sendHeader("Pragma", "no-cache");
    webServer.send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return;
  }

  bool mbSignalPresent = isMainboardSignalPresent();
  bool lockoutActiveNow = false;
  unsigned long lockoutRemainingSec = 0;
  if (powerOnLockoutActive) {
    unsigned long elapsed = millis() - powerOnLockoutStartedAt;
    if (elapsed >= BUTTON_ON_ARM_DELAY_AFTER_OFF_MS) {
      powerOnLockoutActive = false;
    } else {
      lockoutActiveNow = true;
      lockoutRemainingSec = (BUTTON_ON_ARM_DELAY_AFTER_OFF_MS - elapsed + 999) / 1000;
    }
  }

  String wifiStatus = wifiModeLabel();
  String bluetoothStatus = "Coming soon";
  String apiStatus = apiKey.length() > 0 ? "Key configured" : "No key set";
  String ledStatus = powerEnabled ? "Booting" : "Standby";
  if (ledActiveCustom != LED_ACTIVE_NONE) {
    ledStatus = "Custom: " + ledCustomNames[ledActiveCustom];
  }
  String ledBootColor = colorToHex(ledColorBooting);
  String ledStandbyColor = colorToHex(ledColorStandby);
  bool isBoardOn = isMainboardSignalPresent();
  bool osConnected = false;
  String osStatusText = "Offline";
  if (!isBoardOn) {
    osStatusText = "Powered off";
  } else if (osIpAddress.length() > 0 && isOsIpAddressFresh()) {
    osConnected = refreshOsEndpointReachability();
    osStatusText = osConnected ? "Connected" : "Unavailable";
  } else {
    osStatusText = "No OS IP";
  }

  String json = "{";
  json += "\"mainBoardSignal\":";
  json += mbSignalPresent ? "true" : "false";
  json += ",\"mainBoardSignalText\":\"" + String(mbSignalPresent ? "Present" : "Missing") + "\"";
  json += ",\"powerEnabled\":";
  json += powerEnabled ? "true" : "false";
  json += ",\"powerText\":\"" + String(powerEnabled ? "ON" : "OFF") + "\"";
  json += ",\"lockoutActive\":";
  json += lockoutActiveNow ? "true" : "false";
  json += ",\"lockoutRemainingSec\":" + String(lockoutRemainingSec);
  json += ",\"wifiStatus\":\"" + jsonEscape(wifiStatus) + "\"";
  json += ",\"bluetoothStatus\":\"" + jsonEscape(bluetoothStatus) + "\"";
  json += ",\"apiStatus\":\"" + jsonEscape(apiStatus) + "\"";
  json += ",\"osConnected\":" + String(osConnected ? "true" : "false");
  json += ",\"osIpAddress\":\"" + jsonEscape(osIpAddress.length() > 0 && isOsIpAddressFresh() ? osIpAddress : "") + "\"";
  json += ",\"osStatusText\":\"" + jsonEscape(osStatusText) + "\"";
  json += ",\"ledStatus\":\"" + jsonEscape(ledStatus) + "\"";
  json += ",\"ledBootColor\":\"" + ledBootColor + "\"";
  json += ",\"ledStandbyColor\":\"" + ledStandbyColor + "\"";
  json += ",\"ledBootIntensityPct\":" + String(ledBootingIntensityPct);
  json += ",\"ledStandbyIntensityPct\":" + String(ledStandbyIntensityPct);
  json += ",\"ledBootBreathingEnabled\":";
  json += ledBootingBreathingEnabled ? "true" : "false";
  json += ",\"ledStandbyBreathingEnabled\":";
  json += ledStandbyBreathingEnabled ? "true" : "false";
  json += "}";

  webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.send(200, "application/json", json);
}

void handleApiStatus() {
  if (!hasValidApiKey()) {
    sendApiUnauthorized();
    return;
  }

  String ipAddress = "--";
  if (WiFi.status() == WL_CONNECTED) {
    ipAddress = WiFi.localIP().toString();
  } else if (wifiFallbackApEnabled) {
    ipAddress = WiFi.softAPIP().toString();
  }

  String json = "{";
  json += "\"mainBoardSignal\":";
  json += isMainboardSignalPresent() ? "true" : "false";
  json += ",\"powerEnabled\":";
  json += powerEnabled ? "true" : "false";
  json += ",\"powerStatus\":\"" + String(powerEnabled ? "ON" : "OFF") + "\"";
  json += ",\"wifiStatus\":\"" + jsonEscape(wifiModeLabel()) + "\"";
  json += ",\"ipAddress\":\"" + ipAddress + "\"";
  json += "}";

  sendApiJson(200, json);
}

void handleApiPowerOn() {
  if (!hasValidApiKey()) {
    sendApiUnauthorized();
    return;
  }

  if (powerOnLockoutActive) {
    unsigned long elapsed = millis() - powerOnLockoutStartedAt;
    if (elapsed >= BUTTON_ON_ARM_DELAY_AFTER_OFF_MS) {
      powerOnLockoutActive = false;
    } else {
      unsigned long remainingSec = (BUTTON_ON_ARM_DELAY_AFTER_OFF_MS - elapsed + 999) / 1000;
      sendApiJson(409, "{\"success\":false,\"error\":\"power-on lockout active\",\"lockoutRemainingSec\":" + String(remainingSec) + "}");
      return;
    }
  }

  bool changed = false;
  if (!powerEnabled) {
    enablePowerDrive();
    changed = true;
  }

  String json = "{\"success\":true,\"changed\":";
  json += changed ? "true" : "false";
  json += ",\"powerEnabled\":true,\"powerStatus\":\"ON\"}";
  sendApiJson(200, json);
}

void handleApiShutdown() {
  if (!hasValidApiKey()) {
    sendApiUnauthorized();
    return;
  }

  bool changed = false;
  if (powerEnabled) {
    disablePowerDrive();
    changed = true;
  }

  String json = "{\"success\":true,\"changed\":";
  json += changed ? "true" : "false";
  json += ",\"powerEnabled\":false,\"powerStatus\":\"OFF\"}";
  sendApiJson(200, json);
}

void handleApiSetOsAddress() {
  if (!hasValidApiKey()) {
    sendApiUnauthorized();
    return;
  }

  if (!webServer.hasArg("ip")) {
    sendApiJson(400, "{\"success\":false,\"error\":\"missing 'ip' parameter\"}");
    return;
  }

  IPAddress parsed;
  if (!parsed.fromString(webServer.arg("ip"))) {
    sendApiJson(400, "{\"success\":false,\"error\":\"invalid ip address\"}");
    return;
  }

  osIpAddress = parsed.toString();
  osIpAddressUpdatedAt = millis();
  preferences.putString(WIFI_PREF_KEY_OS_IP, osIpAddress);
  preferences.putULong(WIFI_PREF_KEY_OS_IP_TS, osIpAddressUpdatedAt);

  sendApiJson(200, "{\"success\":true,\"osAddress\":\"" + osIpAddress + "\"}");
}

// GET  /setLED                      -> list states (built-in + custom)
// POST /setLED?preset=booting|standby [&color&intensity&breathing] -> edit built-in
// POST /setLED?name=<custom> [&color&intensity&breathing&allowoff]  -> create/edit custom
// POST /setLED?activate=<name>      -> activate a state (custom or booting/standby)
// POST /setLED?clear=1              -> deactivate custom, return to automatic
void handleApiSetLed() {
  if (!hasValidApiKey()) {
    sendApiUnauthorized();
    return;
  }

  if (webServer.method() == HTTP_GET) {
    String json = "{\"success\":true,\"activeCustom\":";
    if (ledActiveCustom == LED_ACTIVE_NONE) {
      json += "null";
    } else {
      json += "\"" + jsonEscape(ledCustomNames[ledActiveCustom]) + "\"";
    }
    json += ",\"states\":[";
    json += "{\"name\":\"booting\",\"builtin\":true,\"color\":\"" + colorToHex(ledColorBooting) + "\",\"intensity\":" + String(ledBootingIntensityPct) + ",\"breathing\":" + (ledBootingBreathingEnabled ? "true" : "false") + ",\"allowedOff\":false}";
    json += ",{\"name\":\"standby\",\"builtin\":true,\"color\":\"" + colorToHex(ledColorStandby) + "\",\"intensity\":" + String(ledStandbyIntensityPct) + ",\"breathing\":" + (ledStandbyBreathingEnabled ? "true" : "false") + ",\"allowedOff\":true}";
    for (uint8_t i = 0; i < ledCustomCount; ++i) {
      json += ",{\"name\":\"" + jsonEscape(ledCustomNames[i]) + "\",\"builtin\":false,\"color\":\"" + colorToHex(ledCustomColors[i]) + "\",\"intensity\":" + String(ledCustomIntensities[i]) + ",\"breathing\":" + (ledCustomBreathings[i] ? "true" : "false") + ",\"allowedOff\":" + (ledCustomAllowedOff[i] ? "true" : "false") + "}";
    }
    json += "]}";
    sendApiJson(200, json);
    return;
  }

  if (webServer.hasArg("clear")) {
    ledActiveCustom = LED_ACTIVE_NONE;
    sendApiJson(200, "{\"success\":true,\"activeCustom\":null}");
    return;
  }

  if (webServer.hasArg("activate")) {
    String name = webServer.arg("activate");
    if (name == "booting" || name == "standby") {
      ledActiveCustom = LED_ACTIVE_NONE;
      sendApiJson(200, "{\"success\":true,\"activeCustom\":null}");
      return;
    }
    int idx = findCustomStateIndex(name);
    if (idx < 0) {
      sendApiJson(404, "{\"success\":false,\"error\":\"state not found\"}");
      return;
    }
    ledActiveCustom = static_cast<uint8_t>(idx);
    sendApiJson(200, "{\"success\":true,\"activeCustom\":\"" + jsonEscape(ledCustomNames[idx]) + "\"}");
    return;
  }

  // Edit built-in preset (booting/standby).
  if (webServer.hasArg("preset")) {
    String preset = webServer.arg("preset");
    if (preset != "booting" && preset != "standby") {
      sendApiJson(400, "{\"success\":false,\"error\":\"preset must be booting or standby (use name= for custom states)\"}");
      return;
    }

    RgbColor* color = &ledColorBooting;
    const char* colorKey = WIFI_PREF_KEY_LED_BOOT;
    uint8_t* intensity = &ledBootingIntensityPct;
    const char* intensityKey = WIFI_PREF_KEY_LED_BOOT_INTENSITY;
    bool* breathing = &ledBootingBreathingEnabled;
    const char* breathingKey = WIFI_PREF_KEY_LED_BOOT_BREATH;
    bool allowedOff = false;

    if (preset == "standby") {
      color = &ledColorStandby;
      colorKey = WIFI_PREF_KEY_LED_NORMAL;
      intensity = &ledStandbyIntensityPct;
      intensityKey = WIFI_PREF_KEY_LED_NORMAL_INTENSITY;
      breathing = &ledStandbyBreathingEnabled;
      breathingKey = WIFI_PREF_KEY_LED_NORMAL_BREATH;
      allowedOff = true;
    }

    bool hadAnyUpdate = false;
    if (webServer.hasArg("color")) {
      RgbColor parsedColor;
      if (!parseHexColor(webServer.arg("color"), parsedColor)) {
        sendApiJson(400, "{\"success\":false,\"error\":\"invalid color, use #RRGGBB\"}");
        return;
      }
      *color = parsedColor;
      preferences.putString(colorKey, colorToHex(parsedColor));
      hadAnyUpdate = true;
    }
    if (webServer.hasArg("intensity")) {
      *intensity = clampLedIntensity(webServer.arg("intensity").toInt());
      preferences.putUChar(intensityKey, *intensity);
      hadAnyUpdate = true;
    }
    if (webServer.hasArg("breathing")) {
      String value = webServer.arg("breathing");
      value.toLowerCase();
      if (value == "1" || value == "true" || value == "on") {
        *breathing = true;
      } else if (value == "0" || value == "false" || value == "off") {
        *breathing = false;
      } else {
        sendApiJson(400, "{\"success\":false,\"error\":\"invalid breathing value (true|false)\"}");
        return;
      }
      preferences.putBool(breathingKey, *breathing);
      hadAnyUpdate = true;
    }
    if (!hadAnyUpdate) {
      sendApiJson(400, "{\"success\":false,\"error\":\"nothing to update, pass color, intensity and/or breathing\"}");
      return;
    }

    String json = "{\"success\":true,\"preset\":\"" + preset + "\"";
    json += ",\"color\":\"" + colorToHex(*color) + "\"";
    json += ",\"intensity\":" + String(*intensity);
    json += ",\"breathing\":";
    json += *breathing ? "true" : "false";
    json += ",\"allowedOff\":";
    json += allowedOff ? "true" : "false";
    json += "}";
    sendApiJson(200, json);
    return;
  }

  // Create or update a custom state.
  if (webServer.hasArg("name")) {
    String name = webServer.arg("name");
    name.trim();
    if (name.length() == 0 || name.length() > LED_CUSTOM_NAME_MAX_LEN) {
      sendApiJson(400, "{\"success\":false,\"error\":\"name must be 1-16 chars\"}");
      return;
    }
    if (name == "booting" || name == "standby") {
      sendApiJson(400, "{\"success\":false,\"error\":\"name reserved, use preset= for built-in states\"}");
      return;
    }

    int idx = findCustomStateIndex(name);
    if (idx < 0) {
      if (ledCustomCount >= LED_MAX_CUSTOM_STATES) {
        sendApiJson(409, "{\"success\":false,\"error\":\"max custom states reached (8)\"}");
        return;
      }
      idx = ledCustomCount;
      ledCustomNames[idx] = name;
      ledCustomColors[idx] = {255, 255, 255};
      ledCustomIntensities[idx] = 80;
      ledCustomBreathings[idx] = false;
      ledCustomAllowedOff[idx] = false;
      ++ledCustomCount;
    }

    if (webServer.hasArg("color")) {
      RgbColor parsedColor;
      if (!parseHexColor(webServer.arg("color"), parsedColor)) {
        sendApiJson(400, "{\"success\":false,\"error\":\"invalid color, use #RRGGBB\"}");
        return;
      }
      ledCustomColors[idx] = parsedColor;
    }
    if (webServer.hasArg("intensity")) {
      ledCustomIntensities[idx] = clampLedIntensity(webServer.arg("intensity").toInt());
    }
    if (webServer.hasArg("breathing")) {
      String value = webServer.arg("breathing");
      value.toLowerCase();
      if (value == "1" || value == "true" || value == "on") {
        ledCustomBreathings[idx] = true;
      } else if (value == "0" || value == "false" || value == "off") {
        ledCustomBreathings[idx] = false;
      } else {
        sendApiJson(400, "{\"success\":false,\"error\":\"invalid breathing value (true|false)\"}");
        return;
      }
    }
    if (webServer.hasArg("allowoff")) {
      String value = webServer.arg("allowoff");
      value.toLowerCase();
      ledCustomAllowedOff[idx] = (value == "1" || value == "true" || value == "on");
    }

    saveCustomState(static_cast<uint8_t>(idx));

    String json = "{\"success\":true,\"state\":\"" + jsonEscape(ledCustomNames[idx]) + "\"";
    json += ",\"color\":\"" + colorToHex(ledCustomColors[idx]) + "\"";
    json += ",\"intensity\":" + String(ledCustomIntensities[idx]);
    json += ",\"breathing\":";
    json += ledCustomBreathings[idx] ? "true" : "false";
    json += ",\"allowedOff\":";
    json += ledCustomAllowedOff[idx] ? "true" : "false";
    json += "}";
    sendApiJson(200, json);
    return;
  }

  sendApiJson(400, "{\"success\":false,\"error\":\"missing parameter: preset, name, activate, or clear\"}");
}

void handleLedColorUpdate() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  if (!isAuthenticated()) {
    sendRedirect("/login");
    return;
  }

  if (authForcePasswordChange) {
    sendRedirect("/change-password");
    return;
  }

  String behavior = webServer.hasArg("behavior") ? webServer.arg("behavior") : String();
  int requestedIntensity = webServer.hasArg("intensity") ? webServer.arg("intensity").toInt() : 80;
  uint8_t parsedIntensity = clampLedIntensity(requestedIntensity);
  bool hasBreathingControl = webServer.hasArg("breathing_present");
  bool parsedBreathing = webServer.hasArg("breathing_enabled");
  bool hadAnyUpdate = false;

  if (behavior != "booting" && behavior != "standby") {
    connectedUiNotice = "Unknown LED behavior.";
    sendRedirect("/");
    return;
  }

  RgbColor* color = &ledColorBooting;
  const char* colorKey = WIFI_PREF_KEY_LED_BOOT;
  uint8_t* intensity = &ledBootingIntensityPct;
  const char* intensityKey = WIFI_PREF_KEY_LED_BOOT_INTENSITY;
  bool* breathing = &ledBootingBreathingEnabled;
  const char* breathingKey = WIFI_PREF_KEY_LED_BOOT_BREATH;

  if (behavior == "standby") {
    color = &ledColorStandby;
    colorKey = WIFI_PREF_KEY_LED_NORMAL;
    intensity = &ledStandbyIntensityPct;
    intensityKey = WIFI_PREF_KEY_LED_NORMAL_INTENSITY;
    breathing = &ledStandbyBreathingEnabled;
    breathingKey = WIFI_PREF_KEY_LED_NORMAL_BREATH;
  }

  if (webServer.hasArg("color")) {
    String colorHex = webServer.arg("color");
    RgbColor parsed;
    if (!parseHexColor(colorHex, parsed)) {
      connectedUiNotice = "Invalid color format. Use #RRGGBB.";
      sendRedirect("/");
      return;
    }
    *color = parsed;
    preferences.putString(colorKey, colorToHex(*color));
    hadAnyUpdate = true;
  }

  *intensity = parsedIntensity;
  preferences.putUChar(intensityKey, *intensity);
  if (hasBreathingControl) {
    *breathing = parsedBreathing;
    preferences.putBool(breathingKey, *breathing);
  }

  hadAnyUpdate = true;

  if (hadAnyUpdate) {
    connectedUiNotice = "LED settings updated for " + behavior + ".";
  }
  sendRedirect("/");
}

void handleLedCustom() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }
  if (!isAuthenticated()) {
    sendRedirect("/login");
    return;
  }
  if (authForcePasswordChange) {
    sendRedirect("/change-password");
    return;
  }

  String name = webServer.hasArg("name") ? webServer.arg("name") : String();
  name.trim();

  if (webServer.hasArg("delete")) {
    int idx = findCustomStateIndex(name);
    if (idx >= 0) {
      removeCustomState(static_cast<uint8_t>(idx));
      connectedUiNotice = "State '" + name + "' deleted.";
    }
    sendRedirect("/");
    return;
  }

  if (webServer.hasArg("activate")) {
    int idx = findCustomStateIndex(name);
    if (idx >= 0) {
      ledActiveCustom = static_cast<uint8_t>(idx);
      connectedUiNotice = "State '" + name + "' activated.";
    }
    sendRedirect("/");
    return;
  }

  if (name.length() == 0 || name.length() > LED_CUSTOM_NAME_MAX_LEN) {
    connectedUiNotice = "State name must be 1-16 chars.";
    sendRedirect("/");
    return;
  }
  if (name == "booting" || name == "standby") {
    connectedUiNotice = "State name reserved.";
    sendRedirect("/");
    return;
  }

  int idx = findCustomStateIndex(name);
  bool isNew = idx < 0;
  if (isNew) {
    if (ledCustomCount >= LED_MAX_CUSTOM_STATES) {
      connectedUiNotice = "Max custom states reached.";
      sendRedirect("/");
      return;
    }
    idx = ledCustomCount;
    ledCustomNames[idx] = name;
    ledCustomColors[idx] = {255, 255, 255};
    ledCustomIntensities[idx] = 80;
    ledCustomBreathings[idx] = false;
    ledCustomAllowedOff[idx] = false;
    ++ledCustomCount;
  }

  if (webServer.hasArg("color")) {
    RgbColor parsed;
    if (parseHexColor(webServer.arg("color"), parsed)) {
      ledCustomColors[idx] = parsed;
    }
  }
  if (webServer.hasArg("intensity")) {
    ledCustomIntensities[idx] = clampLedIntensity(webServer.arg("intensity").toInt());
  }
  ledCustomBreathings[idx] = webServer.hasArg("breathing");
  ledCustomAllowedOff[idx] = webServer.hasArg("allowoff");

  saveCustomState(static_cast<uint8_t>(idx));
  connectedUiNotice = isNew ? "State '" + name + "' created." : "State '" + name + "' updated.";
  sendRedirect("/");
}

void handleApiKeyGenerate() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  if (!isAuthenticated()) {
    sendRedirect("/login");
    return;
  }

  if (authForcePasswordChange) {
    sendRedirect("/change-password");
    return;
  }

  apiKey = generateApiKey();
  preferences.putString(WIFI_PREF_KEY_API_KEY, apiKey);
  connectedUiNotice = "New API key generated.";
  sendRedirect("/");
}

void handleFirmwareUpload() {
  HTTPUpload& upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    firmwareUploadAuthorized = isConnectedModeWebUi() && isAuthenticated() && !authForcePasswordChange;
    firmwareUploadStarted = false;
    firmwareUploadComplete = false;
    firmwareUploadError = "";

    if (!firmwareUploadAuthorized) {
      firmwareUploadError = "Authentication required.";
      return;
    }
    if (powerEnabled) {
      firmwareUploadError = "Turn the power output off before updating firmware.";
      return;
    }

    String filename = upload.filename;
    filename.toLowerCase();
    if (!filename.endsWith(".bin")) {
      firmwareUploadError = "Select a compiled .bin firmware file.";
      return;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      firmwareUploadError = Update.errorString();
      return;
    }
    firmwareUploadStarted = true;
    return;
  }

  if (!firmwareUploadStarted) {
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      firmwareUploadError = Update.errorString();
      Update.abort();
      firmwareUploadStarted = false;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    firmwareUploadComplete = Update.end(true);
    if (!firmwareUploadComplete) {
      firmwareUploadError = Update.errorString();
    }
    firmwareUploadStarted = false;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    firmwareUploadStarted = false;
    firmwareUploadError = "Firmware upload was interrupted.";
  }
}

void handleFirmwareUpdate() {
  if (!isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }
  if (!isAuthenticated()) {
    sendRedirect("/login");
    return;
  }
  if (authForcePasswordChange) {
    sendRedirect("/change-password");
    return;
  }

  if (!firmwareUploadAuthorized) {
    connectedUiNotice = firmwareUploadError.length() > 0 ? firmwareUploadError : "No firmware file was received.";
    sendRedirect("/");
    return;
  }
  if (!firmwareUploadComplete) {
    connectedUiNotice = "Firmware update failed: " + firmwareUploadError;
    sendRedirect("/");
    return;
  }

  webServer.send(200, "text/html", "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Firmware update</title></head><body><p>Firmware updated. The board is restarting.</p></body></html>");
  delay(1000);
  ESP.restart();
}

void handleRoot() {
  if (isConnectedModeWebUi()) {
    if (requireAuthForConnectedUi()) {
      return;
    }

    bool mbSignalPresent = isMainboardSignalPresent();
    bool lockoutActiveNow = false;
    unsigned long lockoutRemainingSec = 0;
    if (powerOnLockoutActive) {
      unsigned long elapsed = millis() - powerOnLockoutStartedAt;
      if (elapsed >= BUTTON_ON_ARM_DELAY_AFTER_OFF_MS) {
        powerOnLockoutActive = false;
      } else {
        lockoutActiveNow = true;
        lockoutRemainingSec = (BUTTON_ON_ARM_DELAY_AFTER_OFF_MS - elapsed + 999) / 1000;
      }
    }

    String html;
    html += "<!doctype html><html><head><meta charset='utf-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>STEAM MACHINE - Connected</title>";
    html += "<style>";
    html += "*{box-sizing:border-box;}";
    html += "body{margin:0;min-height:100vh;font-family:'Trebuchet MS',Verdana,sans-serif;padding:56px 18px 20px;";
    html += "background:linear-gradient(180deg,#1f2b5b 0%,#1a234b 58%,#171d38 100%);color:#fff;}";
    html += ".shell{max-width:700px;margin:0 auto;}";
    html += ".title{text-align:center;font-size:40px;font-weight:900;letter-spacing:1.2px;margin:8px 0 12px;}";
    html += ".panel{background:rgba(10,16,40,.38);border:1px solid rgba(255,255,255,.16);border-radius:14px;padding:16px;}";
    html += ".panel-main{margin-top:14px;}";
    html += ".line{color:#dce8ff;font-size:14px;margin:8px 0;}";
    html += ".btn{display:inline-flex;align-items:center;justify-content:center;padding:10px 14px;border-radius:10px;";
    html += "text-decoration:none;color:#fff;background:#3c8ef4;font-weight:800;}";
    html += ".btn{border:none;cursor:pointer;}";
    html += ".btn:disabled{background:#5c6687;color:#cbd2e5;cursor:not-allowed;opacity:.75;}";
    html += ".btn-danger{background:#c84646;}";
    html += ".notice{margin:8px 0 12px;padding:10px 12px;border-radius:10px;font-size:13px;color:#d6ecff;";
    html += "background:rgba(60,142,244,.15);border:1px solid rgba(115,186,255,.35);}";
    html += ".section-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;}";
    html += ".section-title{margin:0 0 8px;font-size:20px;line-height:1.2;font-weight:900;letter-spacing:.5px;}";
    html += ".section-copy{margin:0;color:#dce8ff;font-size:13px;line-height:1.35;}";
    html += ".section-tag{display:inline-block;margin-top:12px;padding:4px 8px;border-radius:999px;font-size:11px;";
    html += "font-weight:800;letter-spacing:.4px;background:rgba(60,142,244,.2);border:1px solid rgba(89,170,255,.5);color:#cfe5ff;}";
    html += ".api-key-row{display:flex;align-items:center;gap:10px;margin-top:12px;padding:10px;border-radius:10px;background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.12);}";
    html += ".api-key-label{font-size:13px;color:#dce8ff;font-weight:700;}";
    html += ".api-key-value{font-family:'Courier New',monospace;font-size:13px;color:#fff;letter-spacing:.6px;word-break:break-all;}";
    html += ".api-key-form{margin:0;}";
    html += ".link-btn{background:none;border:none;padding:0;color:#77afff;font-size:13px;font-weight:700;cursor:pointer;text-decoration:underline;}";
    html += ".indicator-row{display:flex;align-items:center;gap:8px;margin:8px 0;color:#dce8ff;font-size:13px;}";
    html += ".dot{width:12px;height:12px;border-radius:50%;display:inline-block;box-shadow:0 0 0 2px rgba(255,255,255,.14) inset;}";
    html += ".dot-green{background:#34c759;}";
    html += ".dot-red{background:#ff453a;}";
    html += ".power-actions{display:flex;gap:8px;margin-top:12px;}";
    html += ".power-actions form{flex:1;}";
    html += ".power-actions .btn{width:100%;}";
    html += ".led-list{margin-top:10px;display:grid;gap:10px;}";
    html += ".led-row{display:grid;grid-template-columns:minmax(0,1fr);gap:10px;align-items:center;padding:10px;border-radius:10px;background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.12);}";
    html += ".led-name{font-size:13px;color:#dce8ff;font-weight:700;}";
    html += ".led-preview{display:inline-flex;align-items:center;gap:8px;font-size:12px;color:#dce8ff;}";
    html += ".swatch{width:14px;height:14px;border-radius:50%;border:1px solid rgba(255,255,255,.5);display:inline-block;}";
    html += ".led-form{display:grid;grid-template-columns:42px minmax(0,1fr) 84px;gap:8px;align-items:center;width:100%;min-width:0;}";
    html += ".led-form input[type='color']{width:42px;height:32px;padding:0;border:none;background:transparent;cursor:pointer;}";
    html += ".led-form .btn{grid-column:3;grid-row:2;justify-self:end;width:84px;padding:8px;font-size:12px;}";
    html += ".led-form .check-wrap{grid-column:1 / 3;grid-row:2;}";
    html += ".slider-wrap{display:grid;gap:6px;min-width:0;margin:0;}";
    html += ".slider-label{display:flex;justify-content:space-between;align-items:center;font-size:13px;color:#dce8ff;font-weight:700;}";
    html += ".slider-value{font-size:12px;color:#dce8ff;}";
    html += "input[type='range']{width:100%;min-width:0;accent-color:#3c8ef4;}";
    html += ".check-wrap{display:flex;align-items:center;gap:8px;margin:0;color:#dce8ff;font-size:13px;}";
    html += ".check-wrap input{width:16px;height:16px;}";
    html += ".led-full{width:100%;}";
    html += ".led-checks{grid-column:1 / 3;grid-row:2;display:flex;gap:16px;align-items:center;flex-wrap:wrap;}";
    html += ".led-form input[type='text']{grid-column:1 / 3;padding:8px;border-radius:8px;border:1px solid rgba(255,255,255,.25);background:#101733;color:#fff;}";
    html += ".firmware-form{display:grid;gap:10px;margin-top:12px;}";
    html += ".firmware-form input[type='file']{width:100%;padding:10px;border:1px solid rgba(255,255,255,.24);border-radius:8px;background:#101733;color:#dce8ff;}";
    html += "@media (max-width:680px){";
    html += ".section-grid{grid-template-columns:1fr;}";
    html += ".title{font-size:34px;}";
    html += ".led-row{grid-template-columns:1fr;}";
    html += "}";
    html += "</style></head><body><main class='shell'>";
    html += "<h1 class='title'>STEAM MACHINE</h1>";
    if (connectedUiNotice.length() > 0) {
      html += "<p class='notice'>" + htmlEscape(connectedUiNotice) + "</p>";
      connectedUiNotice = "";
    }
    html += "<section class='section-grid'>";
    html += "<article class='panel'><h2 class='section-title'>Power Control</h2><p class='section-copy'>Control startup and shutdown behavior, timing, and safety interlocks.</p>";
    html += "<div class='indicator-row'><span id='mbSignalDot' class='dot ";
    html += mbSignalPresent ? "dot-green" : "dot-red";
    html += "'></span><span>Main Board Signal: ";
    html += "<strong id='mbSignalText'>";
    html += mbSignalPresent ? "Present" : "Missing";
    html += "</strong></span></div>";
    html += "<div class='indicator-row'><span id='powerDot' class='dot ";
    html += powerEnabled ? "dot-green" : "dot-red";
    html += "'></span><span>Power State: ";
    html += "<strong id='powerText'>";
    html += powerEnabled ? "ON" : "OFF";
    html += "</strong></span></div>";
    html += "<div class='indicator-row'><span id='osIpDot' class='dot ";
    html += isMainboardSignalPresent() && osEndpointReachable ? "dot-green" : "dot-red";
    html += "'></span><span>OS IP: ";
    html += "<strong id='osIpText'>";
    if (osIpAddress.length() > 0 && isOsIpAddressFresh()) {
      html += htmlEscape(osIpAddress);
    }
    html += "</strong></span></div>";
    html += "<p id='lockoutText' class='section-copy'>";
    if (lockoutActiveNow) {
      html += "Power-on lockout: " + String(lockoutRemainingSec) + "s remaining.";
    }
    html += "</p>";
    html += "<div class='power-actions'><form method='POST' action='/power-toggle'><input type='hidden' name='action' value='on'><button class='btn' type='submit'";
    if (powerEnabled) {
      html += " disabled";
    }
    if (lockoutActiveNow) {
      html += " disabled";
    }
    html += " id='powerOnBtn'";
    html += ">POWER ON</button></form>";
    html += "<form method='POST' action='/power-toggle'><input type='hidden' name='action' value='off'><button class='btn btn-danger' type='button'";
    if (!powerEnabled) {
      html += " disabled";
    }
    html += " id='powerOffBtn'";
    html += ">POWER OFF</button></form></div>";
    html += "</article>";
    html += "<article class='panel'><h2 class='section-title'>Bluetooth Controllers</h2><p class='section-copy'>Manage paired controllers, discovery mode, and connection health.</p><span id='bluetoothStatusTag' class='section-tag'>Coming soon</span></article>";
    html += "<article class='panel'><h2 class='section-title'>API Settings</h2><p class='section-copy'>Configure API host, port, and authorization settings for integrations.</p>";
    html += "<div class='api-key-row'><span class='api-key-label'>API Key</span>";
    if (apiKey.length() > 0) {
      html += "<span class='api-key-value'>" + htmlEscape(apiKey) + "</span>";
    } else {
      html += "<form class='api-key-form' method='POST' action='/api-key/generate'><button class='link-btn' type='submit'>Set new key</button></form>";
    }
    html += "</div>";
    html += "<span id='apiStatusTag' class='section-tag'>";
    html += apiKey.length() > 0 ? "Key configured" : "No key set";
    html += "</span></article>";
    html += "<article class='panel'><h2 class='section-title'>LED Management</h2><p class='section-copy'>Tune LED effects, brightness levels, and profile behavior by state.</p><div class='led-list'>";
    html += "<div class='led-row'><div><div class='led-name'>Booting State</div><div class='led-preview'><span id='ledBootSwatch' class='swatch' style='background:" + colorToHex(ledColorBooting) + "'></span><span id='ledBootText'>" + colorToHex(ledColorBooting) + "</span></div></div><form class='led-form led-full' method='POST' action='/led-color'><input type='hidden' name='behavior' value='booting'><input type='hidden' name='breathing_present' value='1'><input id='ledBootColorInput' type='color' name='color' value='" + colorToHex(ledColorBooting) + "'><div class='slider-wrap'><div class='slider-label'><span>Intensity</span><span id='ledBootIntensityText' class='slider-value'>" + String(ledBootingIntensityPct) + "%</span></div><input id='ledBootIntensityInput' type='range' min='5' max='100' name='intensity' value='" + String(ledBootingIntensityPct) + "'></div><label class='check-wrap'><input id='ledBootBreathInput' type='checkbox' name='breathing_enabled' value='1'";
    if (ledBootingBreathingEnabled) {
      html += " checked";
    }
    html += "><span>Effect (breathing)</span></label><button class='btn led-full' type='submit'>SAVE</button></form></div>";
    html += "<div class='led-row'><div><div class='led-name'>Standby State (power off)</div><div class='led-preview'><span id='ledStandbySwatch' class='swatch' style='background:" + colorToHex(ledColorStandby) + "'></span><span id='ledStandbyText'>" + colorToHex(ledColorStandby) + "</span></div></div><form class='led-form led-full' method='POST' action='/led-color'><input type='hidden' name='behavior' value='standby'><input type='hidden' name='breathing_present' value='1'><input id='ledStandbyColorInput' type='color' name='color' value='" + colorToHex(ledColorStandby) + "'><div class='slider-wrap'><div class='slider-label'><span>Intensity</span><span id='ledStandbyIntensityText' class='slider-value'>" + String(ledStandbyIntensityPct) + "%</span></div><input id='ledStandbyIntensityInput' type='range' min='5' max='100' name='intensity' value='" + String(ledStandbyIntensityPct) + "'></div><label class='check-wrap'><input id='ledStandbyBreathInput' type='checkbox' name='breathing_enabled' value='1'";
    if (ledStandbyBreathingEnabled) {
      html += " checked";
    }
    html += "><span>Effect (breathing)</span></label><button class='btn led-full' type='submit'>SAVE</button></form></div>";
    html += "</div>";

    // Custom states.
    for (uint8_t i = 0; i < ledCustomCount; ++i) {
      String nm = htmlEscape(ledCustomNames[i]);
      String hx = colorToHex(ledCustomColors[i]);
      html += "<div class='led-row'><div><div class='led-name'>" + nm;
      if (ledActiveCustom == i) {
        html += " <span class='section-tag' style='margin-top:0;padding:2px 6px;font-size:10px;'>ACTIVE</span>";
      }
      html += "</div><div class='led-preview'><span class='swatch' style='background:" + hx + "'></span><span>" + hx + "</span></div></div>";
      html += "<form class='led-form led-full' method='POST' action='/led-custom'><input type='hidden' name='name' value='" + nm + "'><input type='hidden' name='update' value='1'>";
      html += "<input type='color' name='color' value='" + hx + "'>";
      html += "<div class='slider-wrap'><div class='slider-label'><span>Intensity</span><span class='slider-value'>" + String(ledCustomIntensities[i]) + "%</span></div><input type='range' min='5' max='100' name='intensity' value='" + String(ledCustomIntensities[i]) + "'></div>";
      html += "<div class='led-checks'><label class='check-wrap'><input type='checkbox' name='breathing' value='1'" + String(ledCustomBreathings[i] ? " checked" : "") + "><span>Breathing</span></label>";
      html += "<label class='check-wrap'><input type='checkbox' name='allowoff' value='1'" + String(ledCustomAllowedOff[i] ? " checked" : "") + "><span>Run with power off</span></label></div>";
      html += "<button class='btn led-full' type='submit'>SAVE</button></form>";
      html += "<form method='POST' action='/led-custom' style='display:inline'><input type='hidden' name='name' value='" + nm + "'><input type='hidden' name='activate' value='1'><button class='btn' type='submit' style='padding:6px 10px;font-size:11px;'>ACTIVATE</button></form>";
      html += "<form method='POST' action='/led-custom' style='display:inline'><input type='hidden' name='name' value='" + nm + "'><input type='hidden' name='delete' value='1'><button class='btn btn-danger' type='submit' style='padding:6px 10px;font-size:11px;'>DELETE</button></form>";
      html += "</div>";
    }

    if (ledCustomCount < LED_MAX_CUSTOM_STATES) {
      html += "<div class='led-row'><div class='led-name'>+ New State</div>";
      html += "<form class='led-form led-full' method='POST' action='/led-custom'>";
      html += "<input type='text' name='name' placeholder='name (1-16 chars)' maxlength='16' required>";
      html += "<input type='color' name='color' value='#ffffff'>";
      html += "<div class='slider-wrap'><div class='slider-label'><span>Intensity</span><span class='slider-value'>80%</span></div><input type='range' min='5' max='100' name='intensity' value='80'></div>";
      html += "<div class='led-checks'><label class='check-wrap'><input type='checkbox' name='breathing' value='1'><span>Breathing</span></label>";
      html += "<label class='check-wrap'><input type='checkbox' name='allowoff' value='1'><span>Run with power off</span></label></div>";
      html += "<button class='btn led-full' type='submit'>ADD</button></form></div>";
    }
    html += "</article>";
    html += "</section>";
    html += "<section class='panel panel-main'><h2 class='section-title'>Firmware Update</h2>";
    html += "<p class='section-copy'>Upload a compiled ESP32 .bin firmware file. The power output must be off. The board restarts after a successful update.</p>";
    html += "<form class='firmware-form' method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='firmware' accept='.bin,application/octet-stream' required>";
    html += "<button class='btn' type='submit'>UPLOAD FIRMWARE</button></form></section>";
    html += "<section class='panel panel-main'>";
    html += "<p class='line'>Connected mode active.</p>";
    html += "<p id='wifiStatusLine' class='line'>" + htmlEscape(wifiModeLabel()) + "</p>";
    html += "<p style='margin-top:14px'><a class='btn' href='/logout'>LOGOUT</a></p>";
    html += "</section>";
    html += "<script>";
    html += "function escHtml(v){return (v||'').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/\"/g,'&quot;').replace(/'/g,'&#39;');}";
    html += "function setDot(el,isOn){if(!el)return;el.classList.remove('dot-green');el.classList.remove('dot-red');el.classList.add(isOn?'dot-green':'dot-red');}";
    html += "function setSwatch(id,color){var el=document.getElementById(id);if(el&&color){el.style.background=color;}}";
    html += "function setText(id,text){var el=document.getElementById(id);if(el&&typeof text==='string'){el.textContent=text;}}";
    html += "function syncConnectedStatus(d){";
    html += "var mbDot=document.getElementById('mbSignalDot');var mbText=document.getElementById('mbSignalText');";
    html += "var pDot=document.getElementById('powerDot');var pText=document.getElementById('powerText');";
    html += "var osDot=document.getElementById('osIpDot');var osText=document.getElementById('osIpText');";
    html += "var lock=document.getElementById('lockoutText');var onBtn=document.getElementById('powerOnBtn');var offBtn=document.getElementById('powerOffBtn');";
    html += "var wifi=document.getElementById('wifiStatusLine');var bt=document.getElementById('bluetoothStatusTag');var api=document.getElementById('apiStatusTag');";
    html += "var bIn=document.getElementById('ledBootColorInput');var nIn=document.getElementById('ledStandbyColorInput');";
    html += "var bi=document.getElementById('ledBootIntensityInput');var ni=document.getElementById('ledStandbyIntensityInput');";
    html += "var bt=document.getElementById('ledBootIntensityText');var nt=document.getElementById('ledStandbyIntensityText');";
    html += "var bb=document.getElementById('ledBootBreathInput');var nb=document.getElementById('ledStandbyBreathInput');";
    html += "setDot(mbDot,!!d.mainBoardSignal);if(mbText){mbText.textContent=d.mainBoardSignalText||'Unknown';}";
    html += "setDot(pDot,!!d.powerEnabled);if(pText){pText.textContent=d.powerText||'Unknown';}";
    html += "var osConnected=!!d.osConnected;setDot(osDot,osConnected);if(osText){osText.textContent=d.osIpAddress||'';}";
    html += "if(lock){if(d.lockoutActive){lock.textContent='Power-on lockout: '+String(d.lockoutRemainingSec||0)+'s remaining.';}else{lock.textContent='';}}";
    html += "if(onBtn){onBtn.disabled=!!d.powerEnabled||!!d.lockoutActive;}";
    html += "if(offBtn){offBtn.disabled=!d.powerEnabled;}";
    html += "if(wifi){wifi.textContent=d.wifiStatus||'';}";
    html += "if(bt){bt.textContent=d.bluetoothStatus||'Coming soon';}";
    html += "if(api){api.textContent=d.apiStatus||'No key set';}";
    html += "setSwatch('ledBootSwatch',d.ledBootColor);setSwatch('ledStandbySwatch',d.ledStandbyColor);";
    html += "setText('ledBootText',d.ledBootColor||'');setText('ledStandbyText',d.ledStandbyColor||'');";
    html += "if(bIn&&d.ledBootColor&&document.activeElement!==bIn){bIn.value=d.ledBootColor;}";
    html += "if(nIn&&d.ledStandbyColor&&document.activeElement!==nIn){nIn.value=d.ledStandbyColor;}";
    html += "if(bt&&typeof d.ledBootIntensityPct==='number'){bt.textContent=String(d.ledBootIntensityPct)+'%';}";
    html += "if(nt&&typeof d.ledStandbyIntensityPct==='number'){nt.textContent=String(d.ledStandbyIntensityPct)+'%';}";
    html += "if(bi&&typeof d.ledBootIntensityPct==='number'&&document.activeElement!==bi){bi.value=String(d.ledBootIntensityPct);}";
    html += "if(ni&&typeof d.ledStandbyIntensityPct==='number'&&document.activeElement!==ni){ni.value=String(d.ledStandbyIntensityPct);}";
    html += "if(bb&&typeof d.ledBootBreathingEnabled==='boolean'&&document.activeElement!==bb){bb.checked=!!d.ledBootBreathingEnabled;}";
    html += "if(nb&&typeof d.ledStandbyBreathingEnabled==='boolean'&&document.activeElement!==nb){nb.checked=!!d.ledStandbyBreathingEnabled;}";
    html += "}";
    html += "async function submitPowerAction(action){var body='action='+encodeURIComponent(action);try{var res=await fetch('/power-toggle',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded; charset=UTF-8'},body:body,cache:'no-store'});if(!res.ok&&res.status!==302){throw new Error('HTTP '+res.status);}window.location.reload();}catch(e){window.location.reload();}}";
    html += "(function(){var offBtn=document.getElementById('powerOffBtn');if(!offBtn){return;}var holdTimer=null;var clearHold=function(){if(holdTimer){clearTimeout(holdTimer);holdTimer=null;}};offBtn.addEventListener('click',function(e){e.preventDefault();});offBtn.addEventListener('pointerdown',function(e){e.preventDefault();if(offBtn.disabled){return;}clearHold();holdTimer=setTimeout(function(){holdTimer=null;submitPowerAction('force-off');},5000);});offBtn.addEventListener('pointerup',function(e){e.preventDefault();if(holdTimer){clearHold();submitPowerAction('off');}});offBtn.addEventListener('pointerleave',function(){if(holdTimer){clearHold();submitPowerAction('off');}});offBtn.addEventListener('pointercancel',function(){if(holdTimer){clearHold();submitPowerAction('off');}});offBtn.addEventListener('blur',function(){if(holdTimer){clearHold();submitPowerAction('off');}});})();";
    html += "async function refreshConnectedStatus(){";
    html += "try{var res=await fetch('/connected-status',{cache:'no-store'});";
    html += "if(res.status===401){window.location='/login';return;}";
    html += "if(!res.ok){throw new Error('HTTP '+res.status);}var data=await res.json();syncConnectedStatus(data);}catch(e){}";
    html += "}";
    html += "window.addEventListener('load',function(){refreshConnectedStatus();setInterval(refreshConnectedStatus,2000);});";
    html += "(function(){function bindSlider(sliderId,labelId){var s=document.getElementById(sliderId);var l=document.getElementById(labelId);if(s&&l){s.addEventListener('input',function(){l.textContent=String(s.value)+'%';});}}bindSlider('ledBootIntensityInput','ledBootIntensityText');bindSlider('ledStandbyIntensityInput','ledStandbyIntensityText');})();";
    html += "</script>";
    html += "</main></body></html>";
    webServer.send(200, "text/html", html);
    return;
  }

  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>STEAM MACHINE - WiFi Setup</title>";
  html += "<style>";
  html += "*{box-sizing:border-box;}";
  html += "body{margin:0;min-height:100vh;font-family:'Trebuchet MS',Verdana,sans-serif;";
  html += "display:flex;justify-content:center;align-items:flex-start;padding:72px 24px 24px;";
  html += "background:linear-gradient(180deg,#1f2b5b 0%,#1a234b 58%,#171d38 100%);color:#fff;}";
  html += ".shell{width:min(520px,100%);}";
  html += ".brand{display:flex;justify-content:center;align-items:center;margin-bottom:18px;}";
  html += ".brand-text{font-size:46px;line-height:1;font-weight:900;letter-spacing:2px;text-align:center;";
  html += "text-shadow:0 6px 16px rgba(0,0,0,.35);}";
  html += ".subtitle{margin:0 0 54px;text-align:center;color:rgba(255,255,255,.82);font-size:14px;letter-spacing:.4px;}";
  html += ".cta{display:block;width:100%;text-align:center;text-decoration:none;color:#fff;";
  html += "background:#3c8ef4;padding:18px 18px;border-radius:10px;font-size:34px;";
  html += "font-weight:800;letter-spacing:.6px;box-shadow:0 10px 20px rgba(0,0,0,.22);}";
  html += ".status{margin-top:22px;font-size:13px;color:rgba(255,255,255,.78);text-align:center;}";
  html += "@media (max-width:520px){";
  html += "body{padding-top:56px;}";
  html += ".brand-text{font-size:36px;}";
  html += ".subtitle{margin-bottom:46px;}";
  html += ".cta{font-size:30px;padding:17px 16px;}";
  html += "}";
  html += "</style>";
  html += "</head><body>";
  html += "<main class='shell'>";
  html += "<div class='brand'><div class='brand-text'>STEAM MACHINE</div></div>";
  html += "<p class='subtitle'>WiFi provisioning portal</p>";
  html += "<a class='cta' href='/scan'>SCAN WIFI</a>";
  html += "<p class='status'>" + htmlEscape(wifiModeLabel()) + "</p>";
  html += "</main>";
  html += "</body></html>";

  webServer.send(200, "text/html", html);
}

void handleScan() {
  if (isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>STEAM MACHINE - Scan WiFi</title>";
  html += "<style>";
  html += "*{box-sizing:border-box;}";
  html += "body{margin:0;min-height:100vh;font-family:'Trebuchet MS',Verdana,sans-serif;padding:38px 18px 26px;";
  html += "background:linear-gradient(180deg,#1f2b5b 0%,#1a234b 58%,#171d38 100%);color:#fff;}";
  html += ".shell{max-width:860px;margin:0 auto;}";
  html += ".headline{font-size:36px;font-weight:900;letter-spacing:1.2px;text-align:center;margin:6px 0 8px;}";
  html += ".sub{margin:0 0 24px;text-align:center;color:rgba(255,255,255,.82);font-size:14px;}";
  html += ".panel{background:rgba(10,16,40,.36);border:1px solid rgba(255,255,255,.17);border-radius:14px;padding:14px;}";
  html += ".scan-status{display:flex;align-items:center;gap:10px;color:#d9e6ff;font-weight:700;}";
  html += ".spinner{display:inline-block;width:18px;height:18px;border:3px solid rgba(255,255,255,.24);border-top-color:#77afff;border-radius:50%;animation:spin .8s linear infinite;}";
  html += ".grid{display:grid;grid-template-columns:1fr;gap:10px;margin-top:12px;}";
  html += ".wifi-item{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:12px;border-radius:12px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.18);}";
  html += ".meta{min-width:0;}";
  html += ".ssid{font-weight:800;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:360px;}";
  html += ".details{font-size:12px;color:#c9d8ff;margin-top:3px;}";
  html += ".btn{display:inline-flex;align-items:center;justify-content:center;border:none;border-radius:10px;padding:10px 12px;font-weight:800;cursor:pointer;text-decoration:none;}";
  html += ".btn-primary{background:#3c8ef4;color:#fff;}";
  html += ".btn-ghost{background:transparent;color:#fff;border:1px solid rgba(255,255,255,.32);}";
  html += ".top-actions{display:flex;justify-content:space-between;gap:8px;margin-top:12px;}";
  html += ".top-actions a,.top-actions button{flex:1;}";
  html += ".modal{display:none;position:fixed;z-index:20;inset:0;background:rgba(0,0,0,.52);padding:18px;}";
  html += ".modal-card{background:#1a244a;color:#fff;max-width:430px;margin:12% auto 0;padding:16px;border-radius:12px;border:1px solid rgba(255,255,255,.18);}";
  html += ".modal h3{margin:0 0 8px;} .muted{color:#c9d8ff;font-size:14px;margin:0 0 12px;}";
  html += "input{width:100%;padding:11px 10px;border-radius:8px;border:1px solid rgba(255,255,255,.25);background:#101733;color:#fff;}";
  html += ".modal-actions{display:flex;gap:8px;justify-content:flex-end;margin-top:12px;}";
  html += "@keyframes spin{to{transform:rotate(360deg);}}";
  html += "@media (max-width:560px){.headline{font-size:30px;} .ssid{max-width:190px;}}";
  html += "</style>";
  html += "</head><body>";
  html += "<main class='shell'>";
  html += "<h1 class='headline'>STEAM MACHINE</h1>";
  html += "<p class='sub'>Select a network to connect this device</p>";
  html += "<section class='panel'>";
  html += "<div id='scanStatus' class='scan-status'><span class='spinner'></span><span>Scanning WiFi networks...</span></div>";
  html += "<div id='scanResults' class='grid'></div>";
  html += "<div class='top-actions'><a class='btn btn-ghost' href='/'>Back</a><button class='btn btn-primary' type='button' onclick='loadScan()'>Rescan</button></div>";
  html += "</section>";
  html += "</main>";

  html += "<div id='joinModal' class='modal' aria-hidden='true'>";
  html += "<div class='modal-card'>";
  html += "<h3 style='margin-top:0'>Join WiFi</h3>";
  html += "<p class='muted' id='joinSsidLabel'></p>";
  html += "<form id='joinForm' method='POST' action='/connect'>";
  html += "<input type='hidden' id='joinSsid' name='ssid'>";
  html += "<div id='pwdWrap'><label for='joinPwd'>Password</label><input id='joinPwd' name='password' type='password' placeholder='Enter password'></div>";
  html += "<div class='modal-actions'>";
  html += "<button type='button' onclick='closeJoinModal()'>Cancel</button>";
  html += "<button type='submit'>Join</button>";
  html += "</div></form></div></div>";

  html += "<script>";
  html += "function escHtml(v){return (v||'').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/\"/g,'&quot;').replace(/'/g,'&#39;');}";
  html += "function setScanStatus(text,showSpinner){var s=document.getElementById('scanStatus');s.innerHTML=(showSpinner?'<span class=\"spinner\"></span>':'')+'<span>'+escHtml(text)+'</span>'; }";
  html += "function openJoinModal(btn){var ssid=btn.getAttribute('data-ssid')||'';var isOpen=btn.getAttribute('data-open')==='1';";
  html += "var m=document.getElementById('joinModal');var l=document.getElementById('joinSsidLabel');var s=document.getElementById('joinSsid');var p=document.getElementById('joinPwd');var w=document.getElementById('pwdWrap');";
  html += "s.value=ssid;l.innerHTML='SSID: <strong>'+escHtml(ssid)+'</strong>';";
  html += "if(isOpen){w.style.display='none';p.value='';}else{w.style.display='block';p.value='';setTimeout(function(){p.focus();},50);}m.style.display='block';m.setAttribute('aria-hidden','false');";
  html += "}";
  html += "function closeJoinModal(){var m=document.getElementById('joinModal');m.style.display='none';m.setAttribute('aria-hidden','true');}";
  html += "window.onclick=function(e){var m=document.getElementById('joinModal');if(e.target===m){closeJoinModal();}};";
  html += "window.onkeydown=function(e){if(e.key==='Escape'){closeJoinModal();}};";
  html += "function renderRows(rows){";
  html += "if(!rows||rows.length===0){document.getElementById('scanResults').innerHTML='<div class=\"wifi-item\"><div class=\"meta\"><div class=\"ssid\">No networks found</div><div class=\"details\">Try moving closer to your router and press Rescan.</div></div></div>';setScanStatus('Scan complete',false);return;}";
  html += "rows.sort(function(a,b){return b.rssi-a.rssi;});";
  html += "var html='';";
  html += "for(var i=0;i<rows.length;i++){var n=rows[i];var sec=n.open?'Open':'Secured';";
  html += "html+='<div class=\"wifi-item\"><div class=\"meta\"><div class=\"ssid\">'+escHtml(n.ssid)+'</div><div class=\"details\">'+sec+' | RSSI '+n.rssi+'</div></div>'+";
  html += "'<button class=\"btn btn-primary\" type=\"button\" onclick=\"openJoinModal(this)\" data-ssid=\"'+escHtml(n.ssid)+'\" data-open=\"'+(n.open?'1':'0')+'\">JOIN</button></div>';";
  html += "}";
  html += "document.getElementById('scanResults').innerHTML=html;setScanStatus('Scan complete',false);}";
  html += "async function loadScan(){setScanStatus('Scanning WiFi networks...',true);document.getElementById('scanResults').innerHTML='';try{var res=await fetch('/scan-data',{cache:'no-store'});if(!res.ok){throw new Error('HTTP '+res.status);}var data=await res.json();renderRows(data.networks||[]);}catch(e){setScanStatus('Scan failed. Please try again.',false);document.getElementById('scanResults').innerHTML='';}}";
  html += "window.addEventListener('load',loadScan);";
  html += "</script>";
  html += "</body></html>";
  webServer.send(200, "text/html", html);
}

void handleScanData() {
  if (isConnectedModeWebUi()) {
    webServer.send(403, "application/json", "{\"error\":\"not-available\"}");
    return;
  }

  int count = WiFi.scanNetworks();

  String json = "{\"networks\":[";
  bool first = true;
  for (int i = 0; i < count; ++i) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) {
      continue;
    }

    if (!first) {
      json += ",";
    }
    first = false;

    wifi_auth_mode_t authMode = WiFi.encryptionType(i);
    bool isOpen = authMode == WIFI_AUTH_OPEN;

    String escapedSsid = ssid;
    escapedSsid.replace("\\", "\\\\");
    escapedSsid.replace("\"", "\\\"");

    json += "{\"ssid\":\"" + escapedSsid + "\",\"rssi\":" + String(WiFi.RSSI(i));
    json += ",\"open\":";
    json += isOpen ? "true" : "false";
    json += "}";
  }
  json += "]}";

  WiFi.scanDelete();
  webServer.send(200, "application/json", json);
}

void handleConnect() {
  if (isConnectedModeWebUi()) {
    sendRedirect("/");
    return;
  }

  if (!webServer.hasArg("ssid")) {
    webServer.send(400, "text/plain", "Missing ssid");
    return;
  }

  String ssid = webServer.arg("ssid");
  String pass = webServer.hasArg("password") ? webServer.arg("password") : String();

  savedWifiSsid = ssid;
  savedWifiPass = pass;
  preferences.putString(WIFI_PREF_KEY_SSID, savedWifiSsid);
  preferences.putString(WIFI_PREF_KEY_PASS, savedWifiPass);

  wifiConnectAttempts = 0;
  wifiAttemptLimitLogged = false;

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(savedWifiSsid.c_str(), savedWifiPass.c_str());
  wifiConnectAttempts = 1;
  wifiConnectStartedAt = millis();

  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Connecting</title></head><body style='font-family:Arial,sans-serif;max-width:760px;margin:28px auto;padding:0 16px;'>";
  html += "<h1>BC250 Welcome</h1>";
  html += "<p>Trying to connect to SSID: <strong>" + ssid + "</strong></p>";
  html += "<p>If connection succeeds, this fallback AP will be turned off automatically.</p>";
  html += "<p><a href='/'>Back</a></p></body></html>";
  webServer.send(200, "text/html", html);
}

void handleCaptiveProbe() {
  webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.sendHeader("Location", "/", true);
  webServer.send(302, "text/plain", "");
}

void registerCaptivePortalRoutes() {
  // Android
  webServer.on("/generate_204", HTTP_GET, handleCaptiveProbe);
  webServer.on("/gen_204", HTTP_GET, handleCaptiveProbe);
  // Apple
  webServer.on("/hotspot-detect.html", HTTP_GET, handleCaptiveProbe);
  webServer.on("/library/test/success.html", HTTP_GET, handleCaptiveProbe);
  // Microsoft
  webServer.on("/ncsi.txt", HTTP_GET, handleCaptiveProbe);
  webServer.on("/connecttest.txt", HTTP_GET, handleCaptiveProbe);
  webServer.on("/redirect", HTTP_GET, handleCaptiveProbe);
  webServer.on("/fwlink", HTTP_GET, handleCaptiveProbe);
  // Generic
  webServer.on("/canonical.html", HTTP_GET, handleCaptiveProbe);
}

void setupWifiWebUi() {
  preferences.begin(WIFI_PREF_NAMESPACE, false);
  if (preferences.isKey(WIFI_PREF_KEY_SSID)) {
    savedWifiSsid = preferences.getString(WIFI_PREF_KEY_SSID, "");
  } else {
    savedWifiSsid = "";
  }

  if (preferences.isKey(WIFI_PREF_KEY_PASS)) {
    savedWifiPass = preferences.getString(WIFI_PREF_KEY_PASS, "");
  } else {
    savedWifiPass = "";
  }

  if (preferences.isKey(WIFI_PREF_KEY_AUTH_HASH)) {
    authPasswordHash = preferences.getString(WIFI_PREF_KEY_AUTH_HASH, "");
  }
  if (authPasswordHash.length() == 0) {
    authPasswordHash = hashPassword(AUTH_DEFAULT_PASSWORD);
    preferences.putString(WIFI_PREF_KEY_AUTH_HASH, authPasswordHash);
  }

  if (preferences.isKey(WIFI_PREF_KEY_AUTH_FORCE)) {
    authForcePasswordChange = preferences.getBool(WIFI_PREF_KEY_AUTH_FORCE, true);
  } else {
    authForcePasswordChange = true;
    preferences.putBool(WIFI_PREF_KEY_AUTH_FORCE, authForcePasswordChange);
  }

  if (preferences.isKey(WIFI_PREF_KEY_API_KEY)) {
    apiKey = preferences.getString(WIFI_PREF_KEY_API_KEY, "");
  } else {
    apiKey = "";
  }

  if (preferences.isKey(WIFI_PREF_KEY_OS_IP)) {
    osIpAddress = preferences.getString(WIFI_PREF_KEY_OS_IP, "");
  } else {
    osIpAddress = "";
  }

  if (preferences.isKey(WIFI_PREF_KEY_OS_IP_TS)) {
    osIpAddressUpdatedAt = preferences.getULong(WIFI_PREF_KEY_OS_IP_TS, 0);
  } else {
    osIpAddressUpdatedAt = 0;
  }

  if (!isOsIpAddressFresh()) {
    osIpAddress = "";
    osIpAddressUpdatedAt = 0;
    preferences.remove(WIFI_PREF_KEY_OS_IP);
    preferences.remove(WIFI_PREF_KEY_OS_IP_TS);
  }

  ledColorBooting = loadLedColorPreference(WIFI_PREF_KEY_LED_BOOT, ledColorBooting);
  ledColorStandby = loadLedColorPreference(WIFI_PREF_KEY_LED_NORMAL, ledColorStandby);
  uint8_t legacyIntensity = 80;
  bool legacyBreathing = true;
  if (preferences.isKey(WIFI_PREF_KEY_LED_INTENSITY)) {
    legacyIntensity = clampLedIntensity(preferences.getUChar(WIFI_PREF_KEY_LED_INTENSITY, 80));
  }
  if (preferences.isKey(WIFI_PREF_KEY_LED_BREATH)) {
    legacyBreathing = preferences.getBool(WIFI_PREF_KEY_LED_BREATH, true);
  }

  if (preferences.isKey(WIFI_PREF_KEY_LED_BOOT_INTENSITY)) {
    ledBootingIntensityPct = clampLedIntensity(preferences.getUChar(WIFI_PREF_KEY_LED_BOOT_INTENSITY, ledBootingIntensityPct));
  } else {
    ledBootingIntensityPct = legacyIntensity;
    preferences.putUChar(WIFI_PREF_KEY_LED_BOOT_INTENSITY, ledBootingIntensityPct);
  }
  if (preferences.isKey(WIFI_PREF_KEY_LED_NORMAL_INTENSITY)) {
    ledStandbyIntensityPct = clampLedIntensity(preferences.getUChar(WIFI_PREF_KEY_LED_NORMAL_INTENSITY, ledStandbyIntensityPct));
  } else {
    ledStandbyIntensityPct = 30;
    preferences.putUChar(WIFI_PREF_KEY_LED_NORMAL_INTENSITY, ledStandbyIntensityPct);
  }

  if (preferences.isKey(WIFI_PREF_KEY_LED_BOOT_BREATH)) {
    ledBootingBreathingEnabled = preferences.getBool(WIFI_PREF_KEY_LED_BOOT_BREATH, ledBootingBreathingEnabled);
  } else {
    ledBootingBreathingEnabled = legacyBreathing;
    preferences.putBool(WIFI_PREF_KEY_LED_BOOT_BREATH, ledBootingBreathingEnabled);
  }
  if (preferences.isKey(WIFI_PREF_KEY_LED_NORMAL_BREATH)) {
    ledStandbyBreathingEnabled = preferences.getBool(WIFI_PREF_KEY_LED_NORMAL_BREATH, ledStandbyBreathingEnabled);
  } else {
    ledStandbyBreathingEnabled = false;
    preferences.putBool(WIFI_PREF_KEY_LED_NORMAL_BREATH, ledStandbyBreathingEnabled);
  }

  // Drop legacy shutdown-state prefs.
  preferences.remove(WIFI_PREF_KEY_LED_SHUTDOWN);
  preferences.remove(WIFI_PREF_KEY_LED_SHUTDOWN_INTENSITY);
  preferences.remove(WIFI_PREF_KEY_LED_SHUTDOWN_BREATH);

  // Load custom states.
  ledCustomCount = 0;
  uint8_t storedCount = preferences.getUChar("led_c_count", 0);
  if (storedCount > LED_MAX_CUSTOM_STATES) {
    storedCount = LED_MAX_CUSTOM_STATES;
  }
  for (uint8_t i = 0; i < storedCount; ++i) {
    char key[16];
    snprintf(key, sizeof(key), "led_c%u_name", i);
    if (!preferences.isKey(key)) {
      continue;
    }
    String name = preferences.getString(key, "");
    if (name.length() == 0 || name.length() > LED_CUSTOM_NAME_MAX_LEN) {
      continue;
    }
    snprintf(key, sizeof(key), "led_c%u_color", i);
    RgbColor color = {255, 255, 255};
    if (preferences.isKey(key)) {
      RgbColor parsed;
      if (parseHexColor(preferences.getString(key, ""), parsed)) {
        color = parsed;
      }
    }
    snprintf(key, sizeof(key), "led_c%u_int", i);
    uint8_t intensity = preferences.isKey(key) ? clampLedIntensity(preferences.getUChar(key, 80)) : 80;
    snprintf(key, sizeof(key), "led_c%u_breath", i);
    bool breathing = preferences.isKey(key) ? preferences.getBool(key, false) : false;
    snprintf(key, sizeof(key), "led_c%u_allowoff", i);
    bool allowedOff = preferences.isKey(key) ? preferences.getBool(key, false) : false;

    ledCustomNames[ledCustomCount] = name;
    ledCustomColors[ledCustomCount] = color;
    ledCustomIntensities[ledCustomCount] = intensity;
    ledCustomBreathings[ledCustomCount] = breathing;
    ledCustomAllowedOff[ledCustomCount] = allowedOff;
    ++ledCustomCount;
  }
  preferences.putUChar("led_c_count", ledCustomCount);

  const char* headerKeys[] = {"Cookie", "Authorization"};
  webServer.collectHeaders(headerKeys, 2);

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/scan", HTTP_GET, handleScan);
  webServer.on("/scan-data", HTTP_GET, handleScanData);
  webServer.on("/connect", HTTP_POST, handleConnect);
  webServer.on("/power-toggle", HTTP_POST, handlePowerToggle);
  webServer.on("/led-color", HTTP_POST, handleLedColorUpdate);
  webServer.on("/led-custom", HTTP_POST, handleLedCustom);
  webServer.on("/api-key/generate", HTTP_POST, handleApiKeyGenerate);
  webServer.on("/update", HTTP_POST, handleFirmwareUpdate, handleFirmwareUpload);
  webServer.on("/connected-status", HTTP_GET, handleConnectedStatus);
  webServer.on("/status", HTTP_GET, handleApiStatus);
  webServer.on("/poweron", HTTP_POST, handleApiPowerOn);
  webServer.on("/shutdown", HTTP_POST, handleApiShutdown);
  webServer.on("/setosaddress", HTTP_POST, handleApiSetOsAddress);
  webServer.on("/setLED", HTTP_POST, handleApiSetLed);
  webServer.on("/setLED", HTTP_GET, handleApiSetLed);
  webServer.on("/login", HTTP_GET, handleLoginPage);
  webServer.on("/login", HTTP_POST, handleLoginSubmit);
  webServer.on("/change-password", HTTP_GET, handleChangePasswordPage);
  webServer.on("/change-password", HTTP_POST, handleChangePasswordSubmit);
  webServer.on("/logout", HTTP_GET, handleLogout);
  webServer.on("/favicon.ico", HTTP_GET, []() {
    webServer.send(204, "image/x-icon", "");
  });
  webServer.on("/apple-touch-icon.png", HTTP_GET, []() {
    webServer.send(204, "image/png", "");
  });
  webServer.on("/apple-touch-icon-precomposed.png", HTTP_GET, []() {
    webServer.send(204, "image/png", "");
  });
  registerCaptivePortalRoutes();
  webServer.onNotFound([]() {
    String uri = webServer.uri();
    if (uri == "/apple-touch-icon.png" || uri == "/apple-touch-icon-precomposed.png") {
      webServer.send(204, "image/png", "");
      return;
    }

    Serial.print("[WEB] No handler for ");
    Serial.print(webServer.method() == HTTP_GET ? "GET" : (webServer.method() == HTTP_POST ? "POST" : "OTHER"));
    Serial.print(" ");
    Serial.println(uri);

    if (wifiFallbackApEnabled) {
      webServer.sendHeader("Location", "/", true);
      webServer.send(302, "text/plain", "");
      return;
    }

    webServer.send(404, "text/plain", "Not Found");
  });

  if (savedWifiSsid.length() > 0) {
    trySavedWifiConnection();
  } else {
    WiFi.mode(WIFI_STA);
    Serial.println("[WIFI] No saved credentials; fallback AP is OFF until manual 15s hold");
  }

  // Start HTTP server only after WiFi stack has been initialized.
  webServer.begin();
  webServerStarted = true;
  Serial.println("[WIFI] Web UI server started on port 80");
}

void updateWifiState() {
  bool connected = WiFi.status() == WL_CONNECTED;

  if (connected && !wifiWasConnected) {
    Serial.print("[WIFI] Connected, IP: ");
    Serial.println(WiFi.localIP());
    wifiConnectAttempts = 0;
    wifiAttemptLimitLogged = false;
    disableFallbackAp();
  }

  if (!connected && wifiWasConnected) {
    Serial.println("[WIFI] Connection lost");
    trySavedWifiConnection();
  }

  bool timedOut = savedWifiSsid.length() > 0 &&
                  (millis() - wifiConnectStartedAt) >= WIFI_CONNECT_TIMEOUT_MS;
  if (!connected && timedOut) {
    if (wifiConnectAttempts < WIFI_MAX_CONNECT_ATTEMPTS) {
      Serial.println("[WIFI] Connect timeout, retrying saved credentials");
    }
    trySavedWifiConnection();
  }

  if (wifiFallbackApEnabled && fallbackApExpiresAt != 0) {
    if (static_cast<long>(millis() - fallbackApExpiresAt) >= 0) {
      Serial.println("[WIFI] Fallback AP window expired");
      disableFallbackAp();
    }
  }

  if (webServerStarted) {
    webServer.handleClient();
  }
  if (dnsCaptivePortalEnabled) {
    dnsServer.processNextRequest();
  }
  wifiWasConnected = connected;
}

Adafruit_NeoPixel argb(ARGB_LED_COUNT, PIN_ARGB_DATA, NEO_GRB + NEO_KHZ800);

// Automatic states only: Booting (power ON) and Standby (power OFF).
// Custom states are activated by API and render instead when allowed.
enum class ArgbState {
  Standby,
  Booting,
  Custom,
};

ArgbState argbState = ArgbState::Standby;
ArgbState lastReportedArgbState = ArgbState::Standby;

void resolveArgbState() {
  if (ledActiveCustom != LED_ACTIVE_NONE) {
    if (powerEnabled || ledCustomAllowedOff[ledActiveCustom]) {
      argbState = ArgbState::Custom;
      return;
    }
  }

  argbState = powerEnabled ? ArgbState::Booting : ArgbState::Standby;
}

void setAllArgb(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color = argb.Color(r, g, b);
  for (uint16_t i = 0; i < ARGB_LED_COUNT; ++i) {
    argb.setPixelColor(i, color);
  }
  argb.show();
}

uint8_t scaleChannelByPercent(uint8_t channel, uint8_t percent) {
  return static_cast<uint8_t>((static_cast<uint16_t>(channel) * percent) / 100);
}

uint8_t breathingPercent(unsigned long nowMs, uint8_t maxPercent) {
  unsigned long phase = nowMs % ARGB_BREATH_PERIOD_MS;
  unsigned long half = ARGB_BREATH_PERIOD_MS / 2;
  unsigned long ramp = (phase <= half) ? phase : (ARGB_BREATH_PERIOD_MS - phase);
  uint8_t cappedMax = maxPercent < ARGB_BREATH_MIN_PCT ? ARGB_BREATH_MIN_PCT : maxPercent;
  unsigned long span = cappedMax - ARGB_BREATH_MIN_PCT;
  return static_cast<uint8_t>(ARGB_BREATH_MIN_PCT + ((span * ramp) / half));
}

void reportArgbStateIfChanged() {
  if (argbState == lastReportedArgbState) {
    return;
  }

  if (argbState == ArgbState::Standby) {
    Serial.println("[ARGB] STANDBY");
  } else if (argbState == ArgbState::Booting) {
    Serial.println("[ARGB] BOOTING");
  } else if (argbState == ArgbState::Custom) {
    Serial.print("[ARGB] CUSTOM ");
    Serial.println(ledCustomNames[ledActiveCustom]);
  }

  lastReportedArgbState = argbState;
}

void updateArgbStateMachine() {
  resolveArgbState();
}

void renderArgb() {
  if (argbState == ArgbState::Booting) {
    uint8_t pct = ledBootingBreathingEnabled ? breathingPercent(millis(), ledBootingIntensityPct) : ledBootingIntensityPct;
    setAllArgb(
      scaleChannelByPercent(ledColorBooting.r, pct),
      scaleChannelByPercent(ledColorBooting.g, pct),
      scaleChannelByPercent(ledColorBooting.b, pct)
    );
    return;
  }

  if (argbState == ArgbState::Custom && ledActiveCustom != LED_ACTIVE_NONE) {
    uint8_t pct = ledCustomBreathings[ledActiveCustom] ? breathingPercent(millis(), ledCustomIntensities[ledActiveCustom]) : ledCustomIntensities[ledActiveCustom];
    setAllArgb(
      scaleChannelByPercent(ledCustomColors[ledActiveCustom].r, pct),
      scaleChannelByPercent(ledCustomColors[ledActiveCustom].g, pct),
      scaleChannelByPercent(ledCustomColors[ledActiveCustom].b, pct)
    );
    return;
  }

  uint8_t pct = ledStandbyBreathingEnabled ? breathingPercent(millis(), ledStandbyIntensityPct) : ledStandbyIntensityPct;
  setAllArgb(
    scaleChannelByPercent(ledColorStandby.r, pct),
    scaleChannelByPercent(ledColorStandby.g, pct),
    scaleChannelByPercent(ledColorStandby.b, pct)
  );
}

void updateFallbackHoldIndicator() {
  bool holdInProgress = !powerEnabled && offButtonPressTracking && !offButtonLongHoldHandled &&
                        stableButton == BUTTON_ACTIVE_LEVEL && !wifiFallbackApEnabled;
  digitalWrite(PIN_BOARD_LED, holdInProgress && (millis() / 500) % 2 == 0 ? HIGH : LOW);
}

void updateButtonState() {
  int raw = digitalRead(PIN_BUTTON_START);

  if (raw != lastRawButton) {
    lastRawButton = raw;
    lastButtonChangeAt = millis();
  }

  if ((millis() - lastButtonChangeAt) >= BUTTON_DEBOUNCE_MS && stableButton != raw) {
    stableButton = raw;
    if (stableButton == BUTTON_ACTIVE_LEVEL) {
      buttonPressedEdge = true;
    } else {
      buttonReleasedEdge = true;
    }
  }
}

bool consumeButtonPressedEdge() {
  if (buttonPressedEdge) {
    buttonPressedEdge = false;
    return true;
  }

  return false;
}

bool consumeButtonReleasedEdge() {
  if (buttonReleasedEdge) {
    buttonReleasedEdge = false;
    return true;
  }

  return false;
}

bool isButtonHeldPressed() {
  return stableButton == BUTTON_ACTIVE_LEVEL;
}

void enablePowerDrive() {
  powerEnabled = true;
  powerEnabledAt = millis();
  argbState = ArgbState::Booting;
  signalLossTimerRunning = false;
  offHoldTimerRunning = false;
  offHoldLastProgressSecond = 0;
  digitalWrite(PIN_TRANSISTOR_DRIVE, HIGH);
  Serial.println("[POWER] GPIO25 ON");
}

void disablePowerDrive() {
  powerEnabled = false;
  powerEnabledAt = 0;
  powerOnLockoutActive = true;
  powerOnLockoutStartedAt = millis();
  buttonPressedEdge = false; // Discard stale press captured while power was ON.
  buttonReleasedEdge = false;
  signalLossTimerRunning = false;
  offHoldTimerRunning = false;
  offHoldLastProgressSecond = 0;
  argbState = ArgbState::Standby;
  digitalWrite(PIN_TRANSISTOR_DRIVE, LOW);
  Serial.println("[POWER] GPIO25 OFF");
  Serial.println("[POWER] Power-on locked for 3s after OFF");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRANSISTOR_DRIVE, OUTPUT);
  digitalWrite(PIN_TRANSISTOR_DRIVE, LOW);
  pinMode(PIN_BOARD_LED, OUTPUT);
  digitalWrite(PIN_BOARD_LED, LOW);

  pinMode(PIN_BUTTON_START, INPUT_PULLUP);

  // GPIO34 is input-only and has no internal pull-up/pull-down.
  pinMode(PIN_MB_STATUS, INPUT);

  argb.begin();
  argb.clear();
  argb.show();

  // Initialize button state trackers
  lastRawButton = digitalRead(PIN_BUTTON_START);
  stableButton = lastRawButton;
  lastButtonChangeAt = millis();

  setupWifiWebUi();

  Serial.println("ESP32 Power Controller started");
}

void loop() {
  updateButtonState();

  // 1) Start request: press GPIO18 while power is OFF
  if (!powerEnabled) {
    if (consumeButtonPressedEdge()) {
      offButtonPressTracking = true;
      offButtonPressedAt = millis();
      offButtonLongHoldHandled = false;
    }

    if (offButtonPressTracking && !offButtonLongHoldHandled && isButtonHeldPressed() && !powerEnabled) {
      if ((millis() - offButtonPressedAt) >= WIFI_FALLBACK_AP_HOLD_TO_ENABLE_MS) {
        if (digitalRead(PIN_TRANSISTOR_DRIVE) == LOW) {
          enableTimedFallbackApWindow();
          offButtonLongHoldHandled = true;
          Serial.println("[WIFI] 15s hold detected, fallback AP enabled");
        }
      }
    }

    bool startRequestedByShortPress = false;
    if (consumeButtonReleasedEdge() && offButtonPressTracking) {
      unsigned long heldMs = millis() - offButtonPressedAt;
      startRequestedByShortPress = heldMs < WIFI_FALLBACK_AP_HOLD_TO_ENABLE_MS && !offButtonLongHoldHandled;
      offButtonPressTracking = false;
      offButtonLongHoldHandled = false;
    }

    if (powerOnLockoutActive) {
      if ((millis() - powerOnLockoutStartedAt) >= BUTTON_ON_ARM_DELAY_AFTER_OFF_MS) {
        powerOnLockoutActive = false;
        Serial.println("[POWER] Power-on re-enabled");
      } else {
        startRequestedByShortPress = false;
      }
    }

    if (!powerOnLockoutActive && startRequestedByShortPress) {
      enablePowerDrive();
    }
  }

  // 2) If power drive is active, monitor motherboard status signal on GPIO34
  if (powerEnabled) {
    // 2.a) Manual power-off request via long press
    bool holdToOffArmed = (millis() - powerEnabledAt) >= BUTTON_HOLD_ARM_DELAY_MS;
    if (holdToOffArmed) {
      if (isButtonHeldPressed()) {
        if (!offHoldTimerRunning) {
          offHoldTimerRunning = true;
          offHoldStartedAt = millis();
          offHoldLastProgressSecond = 0;
          Serial.println("[BUTTON] Hold detected, waiting 5s for manual OFF");
        } else {
          unsigned long heldMs = millis() - offHoldStartedAt;
          unsigned long elapsedSec = heldMs / 1000;
          if (elapsedSec > 0 && elapsedSec <= 4 && elapsedSec != offHoldLastProgressSecond) {
            offHoldLastProgressSecond = elapsedSec;
            unsigned long remaining = 5 - elapsedSec;
            Serial.print("[BUTTON] Hold progress: ");
            Serial.print(elapsedSec);
            Serial.print("/5s (");
            Serial.print(remaining);
            Serial.println("s remaining)");
          }

          if (heldMs >= BUTTON_HOLD_TO_OFF_MS) {
            Serial.println("[BUTTON] Held for 5s, manual power OFF");
            disablePowerDrive();
          }
        }
      } else if (offHoldTimerRunning) {
        offHoldTimerRunning = false;
        offHoldLastProgressSecond = 0;
        Serial.println("[BUTTON] Hold canceled before 5s");
      }
    } else if (offHoldTimerRunning) {
      offHoldTimerRunning = false;
      offHoldLastProgressSecond = 0;
    }

    if (powerEnabled) {
      if (isMainboardSignalPresent()) {
        // Signal is valid, keep power on and cancel any pending shutdown timer.
        if (signalLossTimerRunning) {
          signalLossTimerRunning = false;
          Serial.println("[SIGNAL] Restored before timeout");
        }
      } else {
        // Signal missing: start (or continue) delayed shutdown timer.
        if (!signalLossTimerRunning) {
          signalLossTimerRunning = true;
          signalLossStartedAt = millis();
          Serial.println("[SIGNAL] Lost, starting 10s shutdown timer");
        } else if (millis() - signalLossStartedAt >= SIGNAL_LOSS_TIMEOUT_MS) {
          Serial.println("[SIGNAL] Missing for 10s, shutting down drive");
          disablePowerDrive();
        }
      }
    }
  }

  updateArgbStateMachine();
  reportArgbStateIfChanged();
  updateFallbackHoldIndicator();
  renderArgb();
  updateWifiState();

  delay(5);
}
