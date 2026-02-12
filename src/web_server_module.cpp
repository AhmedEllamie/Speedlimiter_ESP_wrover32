/**
 * Author: Ahmed Ellamiee
 * Email:  ahmed.ellamiee@gmail.com
 */
#include "web_server_module.h"

#if !BUILD_TEST_LOGGER

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "shared_state.h"
#include "sl_config.h"
#include "speed_limit_store.h"

static constexpr const char *AP_PASSWORD = "P@ssw0rd";

static WebServer s_server(80);
static bool s_started = false;
static String s_ssid;
static String s_ip;

static const char *modeName() {
#ifdef MANUAL
  return "MANUAL";
#else
  return "AUTOMATIC";
#endif
}

static String buildApSsid() {
  // ESP.getEfuseMac() -> 48-bit MAC. Use last 2 bytes as 4-hex suffix (XXXX).
  uint64_t mac = ESP.getEfuseMac();
  uint16_t suffix = (uint16_t)(mac & 0xFFFFu);

  char buf[5] = {0};
  snprintf(buf, sizeof(buf), "%04X", (unsigned)suffix);

  String ssid("SpeedLimiter");
  ssid += buf;
  return ssid;
}

static String boolText(bool v, const char *t, const char *f) { return v ? String(t) : String(f); }

static void sendHtml(uint16_t code, const String &html) {
  s_server.sendHeader("Cache-Control", "no-store");
  s_server.send(code, "text/html; charset=utf-8", html);
}

static String htmlPageHeader(const char *title, bool auto_refresh) {
  String h;
  h.reserve(900);
  h += "<!doctype html><html><head><meta charset='utf-8'>"
       "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  if (auto_refresh) {
    h += "<meta http-equiv='refresh' content='1'>";
  }
  h += "<title>";
  h += title;
  h += "</title><style>"
       "body{margin:0;padding:16px;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;"
       "background:#0b0f14;color:#e6edf3;}"
       ".nav{display:flex;gap:12px;align-items:center;margin-bottom:12px;}"
       "a{color:#58a6ff;text-decoration:none;}"
       ".card{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:14px;margin:10px 0;}"
       ".label{color:#8b949e;font-size:13px;}"
       ".value{font-size:30px;font-weight:700;margin-top:6px;}"
       ".row{display:grid;grid-template-columns:1fr;gap:10px;}"
       "@media(min-width:720px){.row{grid-template-columns:1fr 1fr 1fr;}}"
       "input,button{font-size:16px;padding:10px;border-radius:10px;border:1px solid #30363d;"
       "background:#0b0f14;color:#e6edf3;}"
       "button{cursor:pointer;}"
       ".ok{color:#3fb950}.warn{color:#f0883e}.muted{color:#8b949e}"
       "</style></head><body>";
  h += "<div class='nav'><strong>SpeedLimiter</strong><span class='muted'>(";
  h += modeName();
  h += ")</span>"
       "<a href='/'>Status</a><a href='/config'>Config</a></div>";
  return h;
}

static String htmlPageFooter() { return String("</body></html>"); }

static void handleApiStatus() {
  uint32_t now = millis();
  bool speed_valid = SharedState_SpeedValid(now, SPEED_TIMEOUT_MS);
  uint8_t speed_kmh = (uint8_t)g_speed_kmh;
  uint16_t sl = SharedState_GetSpeedLimitKmh();
  bool active = SharedState_IsLimiterActive();

  String json;
  json.reserve(160);
  json += "{";
  json += "\"mode\":\"";
  json += modeName();
  json += "\",";
  json += "\"speed_kmh\":";
  json += speed_valid ? String((unsigned)speed_kmh) : String("null");
  json += ",";
  json += "\"speed_limit_kmh\":";
  json += String((unsigned)sl);
  json += ",";
  json += "\"active\":";
  json += active ? "true" : "false";
#ifdef MANUAL
  json += ",";
  json += "\"manual_override\":";
  json += SharedState_GetManualOverrideEnabled() ? "true" : "false";
  json += ",";
  json += "\"manual_override_relay_on\":";
  json += SharedState_GetManualOverrideRelay() ? "true" : "false";
#endif
  json += "}";

  s_server.sendHeader("Cache-Control", "no-store");
  s_server.send(200, "application/json; charset=utf-8", json);
}

static void handleStatusPage() {
  uint32_t now = millis();
  bool speed_valid = SharedState_SpeedValid(now, SPEED_TIMEOUT_MS);
  uint8_t speed_kmh = (uint8_t)g_speed_kmh;
  uint16_t sl = SharedState_GetSpeedLimitKmh();
  bool active = SharedState_IsLimiterActive();

  String html = htmlPageHeader("SpeedLimiter - Status", true);

  html += "<div class='row'>";

  html += "<div class='card'><div class='label'>Current Speed</div><div class='value'>";
  html += speed_valid ? String((unsigned)speed_kmh) + " km/h" : String("N/A");
  html += "</div></div>";

  html += "<div class='card'><div class='label'>Speed Limit</div><div class='value'>";
  html += (sl == 0) ? String("Disabled") : (String((unsigned)sl) + " km/h");
  html += "</div></div>";

  html += "<div class='card'><div class='label'>Speed Limiter</div><div class='value'>";
  html += active ? "<span class='ok'>ACTIVE</span>" : "<span class='warn'>INACTIVE</span>";
  html += "</div></div>";

#ifdef MANUAL
  // Manual override summary (MANUAL build only).
  bool override_enabled = SharedState_GetManualOverrideEnabled();
  bool override_relay_on = SharedState_GetManualOverrideRelay();

  html += "<div class='card'><div class='label'>Manual Override</div><div class='value'>";
  if (override_enabled) {
    html += override_relay_on ? "<span class='ok'>ENABLED (RELAY ON)</span>"
                              : "<span class='warn'>ENABLED (RELAY OFF)</span>";
  } else {
    html += "<span class='muted'>DISABLED</span>";
  }
  html += "</div></div>";
#endif

  html += "</div>";

  html += "<div class='card'><div class='label'>Wi‑Fi AP</div>";
  html += "<div class='muted'>SSID: <strong>";
  html += s_ssid;
  html += "</strong> &nbsp; Password: <strong>";
  html += AP_PASSWORD;
  html += "</strong> &nbsp; IP: <strong>";
  html += s_ip;
  html += "</strong></div>";
  html += "<div class='muted'>JSON: <a href='/api/status'>/api/status</a></div>";
  html += "</div>";

  html += htmlPageFooter();
  sendHtml(200, html);
}

static void handleConfigPage() {
  uint16_t sl = SharedState_GetSpeedLimitKmh();

  String html = htmlPageHeader("SpeedLimiter - Config", false);

  html += "<div class='card'>"
          "<div class='label'>Speed Limit (km/h)</div>"
          "<div class='muted'>Range: 0..250. Use <strong>0</strong> to disable the limiter.</div>"
          "<form action='/set' method='post' style='margin-top:12px;display:flex;gap:10px;flex-wrap:wrap;align-items:center;'>"
          "<input type='number' name='sl' min='0' max='250' value='";
  html += String((unsigned)sl);
  html += "' required>"
          "<button type='submit'>Save</button>"
          "</form>"
          "</div>";

  html += "<div class='card'>"
          "<div class='label'>Reset</div>"
          "<div class='muted'>Resets to default: <strong>";
  html += String((unsigned)SPEED_LIMIT_DEFAULT_KMH);
  html += " km/h</strong></div>"
          "<form action='/reset' method='post' style='margin-top:12px;'>"
          "<button type='submit' onclick=\"return confirm('Reset speed limit to default?');\">Reset to Default</button>"
          "</form>"
          "</div>";

#ifdef MANUAL
  // Manual override controls (only in MANUAL build).
  bool override_enabled = SharedState_GetManualOverrideEnabled();
  bool override_relay_on = SharedState_GetManualOverrideRelay();

  html += "<div class='card'>"
          "<div class='label'>Manual Relay Override</div>"
          "<div class='muted'>When enabled, the relay is controlled directly from this page and ignores speed logic.</div>"
          "<div class='muted'>Status: <strong>";
  html += override_enabled ? "ENABLED" : "DISABLED";
  html += "</strong> &nbsp; Relay: <strong>";
  html += override_relay_on ? "ON" : "OFF";
  html += "</strong></div>"
          "<form action='/override' method='post' style='margin-top:12px;display:flex;gap:10px;flex-wrap:wrap;align-items:center;'>"
          "<input type='hidden' name='enabled' value='";
  html += override_enabled ? "0" : "1";
  html += "'>"
          "<button type='submit'>";
  html += override_enabled ? "Disable Override" : "Enable Override";
  html += "</button>"
          "</form>";

  if (override_enabled) {
    html += "<form action='/override_relay' method='post' style='margin-top:8px;display:flex;gap:10px;flex-wrap:wrap;align-items:center;'>"
            "<input type='hidden' name='state' value='on'>"
            "<button type='submit'>Relay ON</button>"
            "</form>";
    html += "<form action='/override_relay' method='post' style='margin-top:8px;display:flex;gap:10px;flex-wrap:wrap;align-items:center;'>"
            "<input type='hidden' name='state' value='off'>"
            "<button type='submit'>Relay OFF</button>"
            "</form>";
  }

  html += "</div>";
#endif

  html += htmlPageFooter();
  sendHtml(200, html);
}

static void redirectToConfig() {
  s_server.sendHeader("Location", "/config", true);
  s_server.send(303, "text/plain", "See /config");
}

static void handleSetSpeedLimit() {
  if (!s_server.hasArg("sl")) {
    s_server.send(400, "text/plain", "Missing field: sl");
    return;
  }

  int v = s_server.arg("sl").toInt();
  if (v < 0 || v > 250) {
    s_server.send(400, "text/plain", "Invalid speed limit (0..250)");
    return;
  }

  SpeedLimitStore_SetKmh((uint16_t)v);
  redirectToConfig();
}

static void handleResetSpeedLimit() {
  SpeedLimitStore_ResetToDefault();
  redirectToConfig();
}

#ifdef MANUAL
static void handleManualOverride() {
  if (!s_server.hasArg("enabled")) {
    s_server.send(400, "text/plain", "Missing field: enabled");
    return;
  }

  String v = s_server.arg("enabled");
  bool enabled = (v == "1" || v == "on" || v == "ON" || v == "true" || v == "TRUE");

  SharedState_SetManualOverrideEnabled(enabled);
  if (!enabled) {
    // When disabling override, default the requested manual relay state to OFF.
    SharedState_SetManualOverrideRelay(false);
  }

  redirectToConfig();
}

static void handleManualOverrideRelay() {
  if (!s_server.hasArg("state")) {
    s_server.send(400, "text/plain", "Missing field: state");
    return;
  }

  String v = s_server.arg("state");
  bool on = (v == "1" || v == "on" || v == "ON" || v == "true" || v == "TRUE");

  SharedState_SetManualOverrideRelay(on);
  redirectToConfig();
}
#endif

static void handleNotFound() { s_server.send(404, "text/plain", "Not found"); }

void WebServerModule_Begin() {
  if (s_started) return;

  // Ensure speed limit is loaded for UI + runtime use.
  SpeedLimitStore_Begin();

  s_ssid = buildApSsid();

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(s_ssid.c_str(), AP_PASSWORD);
  IPAddress ip = WiFi.softAPIP();
  s_ip = ip.toString();

  Serial.printf("WiFi AP %s (%s) IP=%s\r\n", s_ssid.c_str(), ok ? "OK" : "FAIL", s_ip.c_str());

  s_server.on("/", HTTP_GET, handleStatusPage);
  s_server.on("/config", HTTP_GET, handleConfigPage);
  s_server.on("/api/status", HTTP_GET, handleApiStatus);
  s_server.on("/set", HTTP_POST, handleSetSpeedLimit);
  s_server.on("/reset", HTTP_POST, handleResetSpeedLimit);
#ifdef MANUAL
  s_server.on("/override", HTTP_POST, handleManualOverride);
  s_server.on("/override_relay", HTTP_POST, handleManualOverrideRelay);
#endif
  s_server.onNotFound(handleNotFound);

  s_server.begin();
  s_started = true;
}

void WebServerModule_HandleClient() {
  if (!s_started) return;
  s_server.handleClient();
}

#endif

