#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <DNSServer.h>
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
static const uint8_t ARGB_BREATH_MIN_PCT = 40;
static const uint8_t ARGB_BREATH_MAX_PCT = 80;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;
static const unsigned long WIFI_FALLBACK_AP_HOLD_TO_ENABLE_MS = 15000;
static const unsigned long WIFI_FALLBACK_AP_ACTIVE_MS = 600000;
static const char* WIFI_FALLBACK_AP_SSID = "SteamMachine";
static const char* WIFI_PREF_NAMESPACE = "wifi";
static const char* WIFI_PREF_KEY_SSID = "ssid";
static const char* WIFI_PREF_KEY_PASS = "pass";

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

void handleRoot() {
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>BC250 Welcome</title>";
  html += "<style>body{font-family:Arial,sans-serif;max-width:760px;margin:28px auto;padding:0 16px;}";
  html += "h1{margin-bottom:8px;} .card{border:1px solid #ddd;border-radius:10px;padding:14px;margin:14px 0;}";
  html += "input{width:100%;padding:10px;margin:6px 0;} button{padding:10px 14px;}</style>";
  html += "</head><body>";
  html += "<h1>BC250 Welcome</h1>";
  html += "<p>Status: " + wifiModeLabel() + "</p>";
  html += "<div class='card'><h2>Connect to WiFi</h2>";
  html += "<form method='POST' action='/connect'>";
  html += "<label>SSID</label><input name='ssid' required>";
  html += "<label>Password</label><input name='password' type='password'>";
  html += "<button type='submit'>Connect</button></form></div>";
  html += "<div class='card'><h2>Network Scan</h2><p><a href='/scan'>Scan local WiFi</a></p></div>";
  html += "<p style='color:#666;font-size:14px'>Fallback AP is disabled by default. Hold GPIO18 for 15s while power output is OFF to enable SteamMachine for 10 minutes.</p>";
  html += "</body></html>";

  webServer.send(200, "text/html", html);
}

void handleScan() {
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>WiFi Scan</title>";
  html += "<style>body{font-family:Arial,sans-serif;max-width:960px;margin:28px auto;padding:0 16px;}";
  html += "table{width:100%;border-collapse:collapse;margin-top:10px;}th,td{border:1px solid #ddd;padding:10px;text-align:left;}";
  html += "th{background:#f4f4f4;}button{padding:8px 12px;}";
  html += ".modal{display:none;position:fixed;z-index:20;inset:0;background:rgba(0,0,0,.45);} .modal-card{background:#fff;max-width:420px;margin:12% auto;padding:16px;border-radius:12px;}";
  html += ".modal-actions{display:flex;gap:8px;justify-content:flex-end;margin-top:10px;} .muted{color:#666;font-size:14px;} input{width:100%;padding:10px;}";
  html += ".spinner{display:inline-block;width:18px;height:18px;border:3px solid #ddd;border-top-color:#444;border-radius:50%;animation:spin 0.8s linear infinite;vertical-align:middle;margin-right:8px;}";
  html += "@keyframes spin{to{transform:rotate(360deg);}}</style>";
  html += "</head><body>";
  html += "<h1>BC250 Welcome</h1><h2>Nearby WiFi</h2>";

  html += "<div id='scanStatus'><span class='spinner'></span>Scanning WiFi networks...</div>";
  html += "<div id='scanResults' style='margin-top:12px'></div>";

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
  html += "function setScanStatus(text){document.getElementById('scanStatus').innerHTML=text;}";
  html += "function openJoinModal(btn){var ssid=btn.getAttribute('data-ssid')||'';var isOpen=btn.getAttribute('data-open')==='1';";
  html += "var m=document.getElementById('joinModal');var l=document.getElementById('joinSsidLabel');var s=document.getElementById('joinSsid');var p=document.getElementById('joinPwd');var w=document.getElementById('pwdWrap');";
  html += "s.value=ssid;l.innerHTML='SSID: <strong>'+escHtml(ssid)+'</strong>';";
  html += "if(isOpen){w.style.display='none';p.value='';}else{w.style.display='block';p.value='';setTimeout(function(){p.focus();},50);}m.style.display='block';m.setAttribute('aria-hidden','false');";
  html += "}";
  html += "function closeJoinModal(){var m=document.getElementById('joinModal');m.style.display='none';m.setAttribute('aria-hidden','true');}";
  html += "window.onclick=function(e){var m=document.getElementById('joinModal');if(e.target===m){closeJoinModal();}};";
  html += "window.onkeydown=function(e){if(e.key==='Escape'){closeJoinModal();}};";
  html += "function renderRows(rows){";
  html += "if(!rows||rows.length===0){document.getElementById('scanResults').innerHTML='<p>No networks found.</p>';setScanStatus('');return;}";
  html += "var html='<table><thead><tr><th>Type</th><th>WiFi Name</th><th>RSSI</th><th>Action</th></tr></thead><tbody>';";
  html += "for(var i=0;i<rows.length;i++){var n=rows[i];var icon=n.open?'&#128275; Open':'&#128274; Secured';";
  html += "html+='<tr><td style=\"font-size:18px\">'+icon+'</td><td>'+escHtml(n.ssid)+'</td><td>'+n.rssi+'</td>' +";
  html += "'<td><button type=\"button\" onclick=\"openJoinModal(this)\" data-ssid=\"'+escHtml(n.ssid)+'\" data-open=\"'+(n.open?'1':'0')+'\">JOIN</button></td></tr>';";
  html += "}";
  html += "html+='</tbody></table>';document.getElementById('scanResults').innerHTML=html;setScanStatus('Scan complete.');}";
  html += "async function loadScan(){try{var res=await fetch('/scan-data',{cache:'no-store'});if(!res.ok){throw new Error('HTTP '+res.status);}var data=await res.json();renderRows(data.networks||[]);}catch(e){setScanStatus('Scan failed. Please try again.');document.getElementById('scanResults').innerHTML='';}}";
  html += "window.addEventListener('load',loadScan);";
  html += "</script>";

  html += "<p style='margin-top:14px'><a href='/'>Back</a></p></body></html>";
  webServer.send(200, "text/html", html);
}

void handleScanData() {
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

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/scan", HTTP_GET, handleScan);
  webServer.on("/scan-data", HTTP_GET, handleScanData);
  webServer.on("/connect", HTTP_POST, handleConnect);
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

uint8_t breathingPercent(unsigned long nowMs) {
  unsigned long phase = nowMs % ARGB_BREATH_PERIOD_MS;
  unsigned long half = ARGB_BREATH_PERIOD_MS / 2;
  unsigned long ramp = (phase <= half) ? phase : (ARGB_BREATH_PERIOD_MS - phase);
  unsigned long span = ARGB_BREATH_MAX_PCT - ARGB_BREATH_MIN_PCT;
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
    setAllArgb(
      scaleChannelByPercent(255, 80),
      scaleChannelByPercent(255, 80),
      scaleChannelByPercent(255, 80)
    );
    return;
  }

  uint8_t breathPct = breathingPercent(millis());
  if (argbState == ArgbState::Booting) {
    // Amber base color.
    setAllArgb(
      scaleChannelByPercent(255, breathPct),
      scaleChannelByPercent(140, breathPct),
      scaleChannelByPercent(0, breathPct)
    );
    return;
  }

  // Shutting down: blue base color.
  setAllArgb(
    scaleChannelByPercent(0, breathPct),
    scaleChannelByPercent(110, breathPct),
    scaleChannelByPercent(255, breathPct)
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
