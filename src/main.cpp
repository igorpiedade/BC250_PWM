#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <DNSServer.h>
#include <mbedtls/sha256.h>
#include <Preferences.h>
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
static const unsigned long ARGB_BOOTING_DURATION_MS = 15000;
static const unsigned long ARGB_BREATH_PERIOD_MS = 2500;
static const uint8_t ARGB_BREATH_MIN_PCT = 5;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;
static const unsigned long WIFI_FALLBACK_AP_HOLD_TO_ENABLE_MS = 15000;
static const unsigned long WIFI_FALLBACK_AP_ACTIVE_MS = 600000;
static const char* WIFI_FALLBACK_AP_SSID = "SteamMachine";
static const char* WIFI_PREF_NAMESPACE = "wifi";
static const char* WIFI_PREF_KEY_SSID = "ssid";
static const char* WIFI_PREF_KEY_PASS = "pass";
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
String connectedUiNotice;

struct RgbColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

RgbColor ledColorBooting = {255, 140, 0};
RgbColor ledColorNormal = {255, 255, 255};
RgbColor ledColorShuttingDown = {0, 110, 255};
uint8_t ledBootingIntensityPct = 80;
uint8_t ledNormalIntensityPct = 80;
uint8_t ledShuttingDownIntensityPct = 80;
bool ledBootingBreathingEnabled = true;
bool ledNormalBreathingEnabled = false;
bool ledShuttingDownBreathingEnabled = true;

void enablePowerDrive();
void disablePowerDrive();

bool isMainboardSignalPresent() {
  return digitalRead(PIN_MB_STATUS) == MB_SIGNAL_PRESENT_LEVEL;
}

void beginWifiConnection(const String& ssid, const String& pass) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  wifiConnectStartedAt = millis();

  Serial.print("[WIFI] Connecting to ");
  Serial.println(ssid);
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
  if (action != "on" && action != "off" && action != "toggle") {
    connectedUiNotice = "Invalid power action.";
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
    if (powerEnabled) {
      disablePowerDrive();
      connectedUiNotice = "Power output disabled.";
    } else {
      connectedUiNotice = "Power output is already disabled.";
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
  String apiStatus = "Coming soon";
  String ledStatus = powerEnabled ? "Active" : "Off";
  String ledBootColor = colorToHex(ledColorBooting);
  String ledNormalColor = colorToHex(ledColorNormal);
  String ledShutdownColor = colorToHex(ledColorShuttingDown);

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
  json += ",\"ledStatus\":\"" + jsonEscape(ledStatus) + "\"";
  json += ",\"ledBootColor\":\"" + ledBootColor + "\"";
  json += ",\"ledNormalColor\":\"" + ledNormalColor + "\"";
  json += ",\"ledShutdownColor\":\"" + ledShutdownColor + "\"";
  json += ",\"ledBootIntensityPct\":" + String(ledBootingIntensityPct);
  json += ",\"ledNormalIntensityPct\":" + String(ledNormalIntensityPct);
  json += ",\"ledShutdownIntensityPct\":" + String(ledShuttingDownIntensityPct);
  json += ",\"ledBootBreathingEnabled\":";
  json += ledBootingBreathingEnabled ? "true" : "false";
  json += ",\"ledNormalBreathingEnabled\":";
  json += ledNormalBreathingEnabled ? "true" : "false";
  json += ",\"ledShutdownBreathingEnabled\":";
  json += ledShuttingDownBreathingEnabled ? "true" : "false";
  json += "}";

  webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.send(200, "application/json", json);
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

  if (behavior != "booting" && behavior != "normal" && behavior != "shutdown") {
    connectedUiNotice = "Unknown LED behavior.";
    sendRedirect("/");
    return;
  }

  if (webServer.hasArg("color")) {
    String colorHex = webServer.arg("color");
    RgbColor parsed;
    if (!parseHexColor(colorHex, parsed)) {
      connectedUiNotice = "Invalid color format. Use #RRGGBB.";
      sendRedirect("/");
      return;
    }

    if (behavior == "booting") {
      ledColorBooting = parsed;
      preferences.putString(WIFI_PREF_KEY_LED_BOOT, colorToHex(ledColorBooting));
    } else if (behavior == "normal") {
      ledColorNormal = parsed;
      preferences.putString(WIFI_PREF_KEY_LED_NORMAL, colorToHex(ledColorNormal));
    } else {
      ledColorShuttingDown = parsed;
      preferences.putString(WIFI_PREF_KEY_LED_SHUTDOWN, colorToHex(ledColorShuttingDown));
    }
    hadAnyUpdate = true;
  }

  if (behavior == "booting") {
    ledBootingIntensityPct = parsedIntensity;
    preferences.putUChar(WIFI_PREF_KEY_LED_BOOT_INTENSITY, ledBootingIntensityPct);
    if (hasBreathingControl) {
      ledBootingBreathingEnabled = parsedBreathing;
      preferences.putBool(WIFI_PREF_KEY_LED_BOOT_BREATH, ledBootingBreathingEnabled);
    }
  } else if (behavior == "normal") {
    ledNormalIntensityPct = parsedIntensity;
    preferences.putUChar(WIFI_PREF_KEY_LED_NORMAL_INTENSITY, ledNormalIntensityPct);
    if (hasBreathingControl) {
      ledNormalBreathingEnabled = parsedBreathing;
      preferences.putBool(WIFI_PREF_KEY_LED_NORMAL_BREATH, ledNormalBreathingEnabled);
    }
  } else {
    ledShuttingDownIntensityPct = parsedIntensity;
    preferences.putUChar(WIFI_PREF_KEY_LED_SHUTDOWN_INTENSITY, ledShuttingDownIntensityPct);
    if (hasBreathingControl) {
      ledShuttingDownBreathingEnabled = parsedBreathing;
      preferences.putBool(WIFI_PREF_KEY_LED_SHUTDOWN_BREATH, ledShuttingDownBreathingEnabled);
    }
  }

  hadAnyUpdate = true;

  if (hadAnyUpdate) {
    connectedUiNotice = "LED settings updated for " + behavior + ".";
  }
  sendRedirect("/");
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
    html += ".indicator-row{display:flex;align-items:center;gap:8px;margin:8px 0;color:#dce8ff;font-size:13px;}";
    html += ".dot{width:12px;height:12px;border-radius:50%;display:inline-block;box-shadow:0 0 0 2px rgba(255,255,255,.14) inset;}";
    html += ".dot-green{background:#34c759;}";
    html += ".dot-red{background:#ff453a;}";
    html += ".power-actions{display:flex;gap:8px;margin-top:12px;}";
    html += ".power-actions form{flex:1;}";
    html += ".power-actions .btn{width:100%;}";
    html += ".led-list{margin-top:10px;display:grid;gap:10px;}";
    html += ".led-row{display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center;padding:10px;border-radius:10px;background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.12);}";
    html += ".led-name{font-size:13px;color:#dce8ff;font-weight:700;}";
    html += ".led-preview{display:inline-flex;align-items:center;gap:8px;font-size:12px;color:#dce8ff;}";
    html += ".swatch{width:14px;height:14px;border-radius:50%;border:1px solid rgba(255,255,255,.5);display:inline-block;}";
    html += ".led-form{display:flex;gap:8px;align-items:center;}";
    html += ".led-form input[type='color']{width:42px;height:32px;padding:0;border:none;background:transparent;cursor:pointer;}";
    html += ".led-form .btn{padding:8px 10px;font-size:12px;}";
    html += ".slider-wrap{display:grid;gap:6px;margin-top:8px;margin-bottom:10px;}";
    html += ".slider-label{display:flex;justify-content:space-between;align-items:center;font-size:13px;color:#dce8ff;font-weight:700;}";
    html += ".slider-value{font-size:12px;color:#dce8ff;}";
    html += "input[type='range']{width:100%;accent-color:#3c8ef4;}";
    html += ".check-wrap{display:flex;align-items:center;gap:8px;margin-bottom:10px;color:#dce8ff;font-size:13px;}";
    html += ".check-wrap input{width:16px;height:16px;}";
    html += ".led-full{width:100%;}";
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
    html += "<form method='POST' action='/power-toggle'><input type='hidden' name='action' value='off'><button class='btn btn-danger' type='submit'";
    if (!powerEnabled) {
      html += " disabled";
    }
    html += " id='powerOffBtn'";
    html += ">POWER OFF</button></form></div>";
    html += "</article>";
    html += "<article class='panel'><h2 class='section-title'>Bluetooth Controllers</h2><p class='section-copy'>Manage paired controllers, discovery mode, and connection health.</p><span id='bluetoothStatusTag' class='section-tag'>Coming soon</span></article>";
    html += "<article class='panel'><h2 class='section-title'>API Settings</h2><p class='section-copy'>Configure API host, port, and authorization settings for integrations.</p><span id='apiStatusTag' class='section-tag'>Coming soon</span></article>";
    html += "<article class='panel'><h2 class='section-title'>LED Management</h2><p class='section-copy'>Tune LED effects, brightness levels, and profile behavior by state.</p><div class='led-list'>";
    html += "<div class='led-row'><div><div class='led-name'>Booting State</div><div class='led-preview'><span id='ledBootSwatch' class='swatch' style='background:" + colorToHex(ledColorBooting) + "'></span><span id='ledBootText'>" + colorToHex(ledColorBooting) + "</span></div></div><form class='led-form led-full' method='POST' action='/led-color'><input type='hidden' name='behavior' value='booting'><input type='hidden' name='breathing_present' value='1'><input id='ledBootColorInput' type='color' name='color' value='" + colorToHex(ledColorBooting) + "'><div class='slider-wrap'><div class='slider-label'><span>Intensity</span><span id='ledBootIntensityText' class='slider-value'>" + String(ledBootingIntensityPct) + "%</span></div><input id='ledBootIntensityInput' type='range' min='5' max='100' name='intensity' value='" + String(ledBootingIntensityPct) + "'></div><label class='check-wrap'><input id='ledBootBreathInput' type='checkbox' name='breathing_enabled' value='1'";
    if (ledBootingBreathingEnabled) {
      html += " checked";
    }
    html += "><span>Effect (breathing)</span></label><button class='btn led-full' type='submit'>SAVE</button></form></div>";
    html += "<div class='led-row'><div><div class='led-name'>Normal State</div><div class='led-preview'><span id='ledNormalSwatch' class='swatch' style='background:" + colorToHex(ledColorNormal) + "'></span><span id='ledNormalText'>" + colorToHex(ledColorNormal) + "</span></div></div><form class='led-form led-full' method='POST' action='/led-color'><input type='hidden' name='behavior' value='normal'><input type='hidden' name='breathing_present' value='1'><input id='ledNormalColorInput' type='color' name='color' value='" + colorToHex(ledColorNormal) + "'><div class='slider-wrap'><div class='slider-label'><span>Intensity</span><span id='ledNormalIntensityText' class='slider-value'>" + String(ledNormalIntensityPct) + "%</span></div><input id='ledNormalIntensityInput' type='range' min='5' max='100' name='intensity' value='" + String(ledNormalIntensityPct) + "'></div><label class='check-wrap'><input id='ledNormalBreathInput' type='checkbox' name='breathing_enabled' value='1'";
    if (ledNormalBreathingEnabled) {
      html += " checked";
    }
    html += "><span>Effect (breathing)</span></label><button class='btn led-full' type='submit'>SAVE</button></form></div>";
    html += "<div class='led-row'><div><div class='led-name'>Shutting Down State</div><div class='led-preview'><span id='ledShutdownSwatch' class='swatch' style='background:" + colorToHex(ledColorShuttingDown) + "'></span><span id='ledShutdownText'>" + colorToHex(ledColorShuttingDown) + "</span></div></div><form class='led-form led-full' method='POST' action='/led-color'><input type='hidden' name='behavior' value='shutdown'><input type='hidden' name='breathing_present' value='1'><input id='ledShutdownColorInput' type='color' name='color' value='" + colorToHex(ledColorShuttingDown) + "'><div class='slider-wrap'><div class='slider-label'><span>Intensity</span><span id='ledShutdownIntensityText' class='slider-value'>" + String(ledShuttingDownIntensityPct) + "%</span></div><input id='ledShutdownIntensityInput' type='range' min='5' max='100' name='intensity' value='" + String(ledShuttingDownIntensityPct) + "'></div><label class='check-wrap'><input id='ledShutdownBreathInput' type='checkbox' name='breathing_enabled' value='1'";
    if (ledShuttingDownBreathingEnabled) {
      html += " checked";
    }
    html += "><span>Effect (breathing)</span></label><button class='btn led-full' type='submit'>SAVE</button></form></div>";
    html += "</div></article>";
    html += "</section>";
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
    html += "var lock=document.getElementById('lockoutText');var onBtn=document.getElementById('powerOnBtn');var offBtn=document.getElementById('powerOffBtn');";
    html += "var wifi=document.getElementById('wifiStatusLine');var bt=document.getElementById('bluetoothStatusTag');var api=document.getElementById('apiStatusTag');";
    html += "var bIn=document.getElementById('ledBootColorInput');var nIn=document.getElementById('ledNormalColorInput');var sIn=document.getElementById('ledShutdownColorInput');";
    html += "var bi=document.getElementById('ledBootIntensityInput');var ni=document.getElementById('ledNormalIntensityInput');var si=document.getElementById('ledShutdownIntensityInput');";
    html += "var bt=document.getElementById('ledBootIntensityText');var nt=document.getElementById('ledNormalIntensityText');var st=document.getElementById('ledShutdownIntensityText');";
    html += "var bb=document.getElementById('ledBootBreathInput');var nb=document.getElementById('ledNormalBreathInput');var sb=document.getElementById('ledShutdownBreathInput');";
    html += "setDot(mbDot,!!d.mainBoardSignal);if(mbText){mbText.textContent=d.mainBoardSignalText||'Unknown';}";
    html += "setDot(pDot,!!d.powerEnabled);if(pText){pText.textContent=d.powerText||'Unknown';}";
    html += "if(lock){if(d.lockoutActive){lock.textContent='Power-on lockout: '+String(d.lockoutRemainingSec||0)+'s remaining.';}else{lock.textContent='';}}";
    html += "if(onBtn){onBtn.disabled=!!d.powerEnabled||!!d.lockoutActive;}";
    html += "if(offBtn){offBtn.disabled=!d.powerEnabled;}";
    html += "if(wifi){wifi.textContent=d.wifiStatus||'';}";
    html += "if(bt){bt.textContent=d.bluetoothStatus||'Coming soon';}";
    html += "if(api){api.textContent=d.apiStatus||'Coming soon';}";
    html += "setSwatch('ledBootSwatch',d.ledBootColor);setSwatch('ledNormalSwatch',d.ledNormalColor);setSwatch('ledShutdownSwatch',d.ledShutdownColor);";
    html += "setText('ledBootText',d.ledBootColor||'');setText('ledNormalText',d.ledNormalColor||'');setText('ledShutdownText',d.ledShutdownColor||'');";
    html += "if(bIn&&d.ledBootColor&&document.activeElement!==bIn){bIn.value=d.ledBootColor;}";
    html += "if(nIn&&d.ledNormalColor&&document.activeElement!==nIn){nIn.value=d.ledNormalColor;}";
    html += "if(sIn&&d.ledShutdownColor&&document.activeElement!==sIn){sIn.value=d.ledShutdownColor;}";
    html += "if(bt&&typeof d.ledBootIntensityPct==='number'){bt.textContent=String(d.ledBootIntensityPct)+'%';}";
    html += "if(nt&&typeof d.ledNormalIntensityPct==='number'){nt.textContent=String(d.ledNormalIntensityPct)+'%';}";
    html += "if(st&&typeof d.ledShutdownIntensityPct==='number'){st.textContent=String(d.ledShutdownIntensityPct)+'%';}";
    html += "if(bi&&typeof d.ledBootIntensityPct==='number'&&document.activeElement!==bi){bi.value=String(d.ledBootIntensityPct);}";
    html += "if(ni&&typeof d.ledNormalIntensityPct==='number'&&document.activeElement!==ni){ni.value=String(d.ledNormalIntensityPct);}";
    html += "if(si&&typeof d.ledShutdownIntensityPct==='number'&&document.activeElement!==si){si.value=String(d.ledShutdownIntensityPct);}";
    html += "if(bb&&typeof d.ledBootBreathingEnabled==='boolean'&&document.activeElement!==bb){bb.checked=!!d.ledBootBreathingEnabled;}";
    html += "if(nb&&typeof d.ledNormalBreathingEnabled==='boolean'&&document.activeElement!==nb){nb.checked=!!d.ledNormalBreathingEnabled;}";
    html += "if(sb&&typeof d.ledShutdownBreathingEnabled==='boolean'&&document.activeElement!==sb){sb.checked=!!d.ledShutdownBreathingEnabled;}";
    html += "}";
    html += "async function refreshConnectedStatus(){";
    html += "try{var res=await fetch('/connected-status',{cache:'no-store'});";
    html += "if(res.status===401){window.location='/login';return;}";
    html += "if(!res.ok){throw new Error('HTTP '+res.status);}var data=await res.json();syncConnectedStatus(data);}catch(e){}";
    html += "}";
    html += "window.addEventListener('load',function(){refreshConnectedStatus();setInterval(refreshConnectedStatus,2000);});";
    html += "(function(){function bindSlider(sliderId,labelId){var s=document.getElementById(sliderId);var l=document.getElementById(labelId);if(s&&l){s.addEventListener('input',function(){l.textContent=String(s.value)+'%';});}}bindSlider('ledBootIntensityInput','ledBootIntensityText');bindSlider('ledNormalIntensityInput','ledNormalIntensityText');bindSlider('ledShutdownIntensityInput','ledShutdownIntensityText');})();";
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

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(savedWifiSsid.c_str(), savedWifiPass.c_str());
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

  ledColorBooting = loadLedColorPreference(WIFI_PREF_KEY_LED_BOOT, ledColorBooting);
  ledColorNormal = loadLedColorPreference(WIFI_PREF_KEY_LED_NORMAL, ledColorNormal);
  ledColorShuttingDown = loadLedColorPreference(WIFI_PREF_KEY_LED_SHUTDOWN, ledColorShuttingDown);
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
    ledNormalIntensityPct = clampLedIntensity(preferences.getUChar(WIFI_PREF_KEY_LED_NORMAL_INTENSITY, ledNormalIntensityPct));
  } else {
    ledNormalIntensityPct = legacyIntensity;
    preferences.putUChar(WIFI_PREF_KEY_LED_NORMAL_INTENSITY, ledNormalIntensityPct);
  }
  if (preferences.isKey(WIFI_PREF_KEY_LED_SHUTDOWN_INTENSITY)) {
    ledShuttingDownIntensityPct = clampLedIntensity(preferences.getUChar(WIFI_PREF_KEY_LED_SHUTDOWN_INTENSITY, ledShuttingDownIntensityPct));
  } else {
    ledShuttingDownIntensityPct = legacyIntensity;
    preferences.putUChar(WIFI_PREF_KEY_LED_SHUTDOWN_INTENSITY, ledShuttingDownIntensityPct);
  }

  if (preferences.isKey(WIFI_PREF_KEY_LED_BOOT_BREATH)) {
    ledBootingBreathingEnabled = preferences.getBool(WIFI_PREF_KEY_LED_BOOT_BREATH, ledBootingBreathingEnabled);
  } else {
    ledBootingBreathingEnabled = legacyBreathing;
    preferences.putBool(WIFI_PREF_KEY_LED_BOOT_BREATH, ledBootingBreathingEnabled);
  }
  if (preferences.isKey(WIFI_PREF_KEY_LED_NORMAL_BREATH)) {
    ledNormalBreathingEnabled = preferences.getBool(WIFI_PREF_KEY_LED_NORMAL_BREATH, ledNormalBreathingEnabled);
  } else {
    ledNormalBreathingEnabled = false;
    preferences.putBool(WIFI_PREF_KEY_LED_NORMAL_BREATH, ledNormalBreathingEnabled);
  }
  if (preferences.isKey(WIFI_PREF_KEY_LED_SHUTDOWN_BREATH)) {
    ledShuttingDownBreathingEnabled = preferences.getBool(WIFI_PREF_KEY_LED_SHUTDOWN_BREATH, ledShuttingDownBreathingEnabled);
  } else {
    ledShuttingDownBreathingEnabled = legacyBreathing;
    preferences.putBool(WIFI_PREF_KEY_LED_SHUTDOWN_BREATH, ledShuttingDownBreathingEnabled);
  }

  const char* headerKeys[] = {"Cookie"};
  webServer.collectHeaders(headerKeys, 1);

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/scan", HTTP_GET, handleScan);
  webServer.on("/scan-data", HTTP_GET, handleScanData);
  webServer.on("/connect", HTTP_POST, handleConnect);
  webServer.on("/power-toggle", HTTP_POST, handlePowerToggle);
  webServer.on("/led-color", HTTP_POST, handleLedColorUpdate);
  webServer.on("/connected-status", HTTP_GET, handleConnectedStatus);
  webServer.on("/login", HTTP_GET, handleLoginPage);
  webServer.on("/login", HTTP_POST, handleLoginSubmit);
  webServer.on("/change-password", HTTP_GET, handleChangePasswordPage);
  webServer.on("/change-password", HTTP_POST, handleChangePasswordSubmit);
  webServer.on("/logout", HTTP_GET, handleLogout);
  registerCaptivePortalRoutes();
  webServer.onNotFound([]() {
    if (wifiFallbackApEnabled) {
      webServer.sendHeader("Location", "/", true);
      webServer.send(302, "text/plain", "");
      return;
    }

    webServer.send(404, "text/plain", "Not Found");
  });

  if (savedWifiSsid.length() > 0) {
    beginWifiConnection(savedWifiSsid, savedWifiPass);
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
    disableFallbackAp();
  }

  if (!connected && wifiWasConnected) {
    Serial.println("[WIFI] Connection lost");
    if (savedWifiSsid.length() > 0) {
      beginWifiConnection(savedWifiSsid, savedWifiPass);
    }
  }

  bool timedOut = savedWifiSsid.length() > 0 &&
                  (millis() - wifiConnectStartedAt) >= WIFI_CONNECT_TIMEOUT_MS;
  if (!connected && timedOut) {
    Serial.println("[WIFI] Connect timeout, retrying saved credentials");
    beginWifiConnection(savedWifiSsid, savedWifiPass);
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

enum class ArgbState {
  Off,
  Booting,
  Normal,
  ShuttingDown,
};

ArgbState argbState = ArgbState::Off;
ArgbState lastReportedArgbState = ArgbState::Off;
unsigned long argbBootStartedAt = 0;

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

  if (argbState == ArgbState::Off) {
    Serial.println("[ARGB] OFF");
  } else if (argbState == ArgbState::Booting) {
    Serial.println("[ARGB] BOOTING (amber breathing 40-80%)");
  } else if (argbState == ArgbState::Normal) {
    Serial.println("[ARGB] NORMAL (white 80%)");
  } else if (argbState == ArgbState::ShuttingDown) {
    Serial.println("[ARGB] SHUTTING_DOWN (blue breathing 40-80%)");
  }

  lastReportedArgbState = argbState;
}

void updateArgbStateMachine() {
  if (!powerEnabled) {
    argbState = ArgbState::Off;
    return;
  }

  bool signalPresent = (digitalRead(PIN_MB_STATUS) == MB_SIGNAL_PRESENT_LEVEL);

  if (argbState == ArgbState::Off) {
    argbState = ArgbState::Booting;
    argbBootStartedAt = millis();
  }

  if (argbState == ArgbState::Booting) {
    if (!signalPresent) {
      argbState = ArgbState::ShuttingDown;
    } else if ((millis() - argbBootStartedAt) >= ARGB_BOOTING_DURATION_MS) {
      argbState = ArgbState::Normal;
    }
    return;
  }

  if (argbState == ArgbState::Normal) {
    if (!signalPresent) {
      argbState = ArgbState::ShuttingDown;
    }
    return;
  }

  if (argbState == ArgbState::ShuttingDown && signalPresent) {
    argbState = ArgbState::Normal;
  }
}

void renderArgb() {
  if (argbState == ArgbState::Off) {
    setAllArgb(0, 0, 0);
    return;
  }

  if (argbState == ArgbState::Normal) {
    uint8_t normalPct = ledNormalBreathingEnabled ? breathingPercent(millis(), ledNormalIntensityPct) : ledNormalIntensityPct;
    setAllArgb(
      scaleChannelByPercent(ledColorNormal.r, normalPct),
      scaleChannelByPercent(ledColorNormal.g, normalPct),
      scaleChannelByPercent(ledColorNormal.b, normalPct)
    );
    return;
  }

  if (argbState == ArgbState::Booting) {
    uint8_t bootPct = ledBootingBreathingEnabled ? breathingPercent(millis(), ledBootingIntensityPct) : ledBootingIntensityPct;
    setAllArgb(
      scaleChannelByPercent(ledColorBooting.r, bootPct),
      scaleChannelByPercent(ledColorBooting.g, bootPct),
      scaleChannelByPercent(ledColorBooting.b, bootPct)
    );
    return;
  }

  uint8_t shutdownPct = ledShuttingDownBreathingEnabled ? breathingPercent(millis(), ledShuttingDownIntensityPct) : ledShuttingDownIntensityPct;
  setAllArgb(
    scaleChannelByPercent(ledColorShuttingDown.r, shutdownPct),
    scaleChannelByPercent(ledColorShuttingDown.g, shutdownPct),
    scaleChannelByPercent(ledColorShuttingDown.b, shutdownPct)
  );
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
  argbBootStartedAt = powerEnabledAt;
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
  argbState = ArgbState::Off;
  digitalWrite(PIN_TRANSISTOR_DRIVE, LOW);
  Serial.println("[POWER] GPIO25 OFF");
  Serial.println("[POWER] Power-on locked for 3s after OFF");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRANSISTOR_DRIVE, OUTPUT);
  digitalWrite(PIN_TRANSISTOR_DRIVE, LOW);

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
  renderArgb();
  updateWifiState();

  delay(5);
}
