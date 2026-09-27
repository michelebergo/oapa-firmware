#include "web_server.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>

#include <cmath>

#include "loop/loop_status_json.h"

#include <cstring>

#include "asiair/asiair_status.h"
#include "asiair_client.h"
#include "device_config.h"
#include "device_state.h"
#include "loop/calibration_status.h"
#include "page_json.h"

#include "net.h"
#include "ota_rules.h"
#include "shared_state.h"
#include "status_json.h"
#include "web_assets.h"
#include "web_bridge.h"

namespace webserver {
namespace {

AsyncWebServer server(80);
const char *firmwareVersion = "";

void sendPage(AsyncWebServerRequest *req, const uint8_t *gz, size_t len) {
  AsyncWebServerResponse *res = req->beginResponse(200, "text/html", gz, len);
  res->addHeader("Content-Encoding", "gzip");
  res->addHeader("Cache-Control", "no-store");
  req->send(res);
}

String formField(AsyncWebServerRequest *req, const char *name) {
  return req->hasParam(name, true) ? req->getParam(name, true)->value() : String();
}

double numberField(AsyncWebServerRequest *req, const char *name, double fallback) {
  String value = formField(req, name);
  return value.length() ? value.toDouble() : fallback;
}

uint32_t sinceParam(AsyncWebServerRequest *req) {
  return req->hasParam("since") ? static_cast<uint32_t>(req->getParam("since")->value().toInt()) : 0;
}

bool loopRunning() {
  LoopStatusSnapshot s = shared::readLoop();
  return std::strcmp(s.phase, "idle") != 0 && std::strcmp(s.phase, "ended") != 0;
}

bool calibrating() {
  CalibrationSnapshot c = shared::readCalibration();
  for (const char *state : {"preloading", "baseline", "probing", "measuring", "restoring"}) {
    if (std::strcmp(c.state, state) == 0) return true;
  }
  return false;
}

// Firmware update state. Every handler runs on the single AsyncTCP task, so plain statics are safe.
ota::Lockout otaLockout;
ota::UploadGate otaGate;
uint32_t otaNextId = 1;
enum class OtaResult : uint8_t { Rejected, Writing, Done };

// One per upload request, kept in req->_tempObject (the server frees it with the request),
// so a second upload's outcome never overwrites the first one's.
struct OtaUpload {
  uint32_t id;
  OtaResult result;
  int code;
  char message[80];
  size_t bytes;
};

void otaReject(OtaUpload *u, int code, const char *message) {
  u->result = OtaResult::Rejected;
  u->code = code;
  std::snprintf(u->message, sizeof u->message, "%s", message);
}

const char *otaBusyReason() {
  StatusSnapshot s = shared::read();
  if (s.ninaActive) return "A PC is controlling OAPA";
  if (loopRunning()) return "automatic alignment is running";
  if (calibrating()) return "calibration is running";
  if (s.running) return "motors are moving";
  return nullptr;
}

// Checked on the first chunk, before a single byte is written.
bool otaAuthorized(AsyncWebServerRequest *req, OtaUpload *u) {
  uint32_t now = millis();
  if (!device::otaPasswordSet()) {
    otaReject(u, 403, "set an update password in System first");
    return false;
  }
  if (otaLockout.locked(now)) {
    otaReject(u, 429, "too many wrong passwords, wait a minute");
    return false;
  }
  String password = req->hasHeader("X-OTA-Password") ? req->header("X-OTA-Password") : String();
  if (!device::checkOtaPassword(password.c_str())) {
    otaLockout.fail(now);
    otaReject(u, 401, "wrong password");
    return false;
  }
  otaLockout.succeed();
  return true;
}

void sendSettings(AsyncWebServerRequest *req, const devset::LoopConfig &c) {
  char host[16];
  device::asiairHost(host, sizeof host);
  char body[320];
  formatSettingsJson(c, host, body, sizeof body);
  req->send(200, "application/json", body);
}

}  // namespace

void begin(const char *fwVersion) {
  firmwareVersion = fwVersion;

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (net::inSetupMode()) sendPage(req, SETUP_HTML_GZ, SETUP_HTML_GZ_LEN);
    else sendPage(req, INDEX_HTML_GZ, INDEX_HTML_GZ_LEN);
  });

  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *req) {
    sendPage(req, SETUP_HTML_GZ, SETUP_HTML_GZ_LEN);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    StatusSnapshot s = shared::read();
    s.stationConnected = net::stationConnected();
    s.rssi = net::rssi();
    s.uptimeS = millis() / 1000;
    s.bootCount = net::bootCount();
    s.resetReason = net::resetReasonName();
    s.fwVersion = firmwareVersion;
    char body[256];
    if (formatStatusJson(s, body, sizeof body) < 0) {
      req->send(500, "text/plain", "status too long");
      return;
    }
    req->send(200, "application/json", body);
  });

  server.on("/api/jog", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (shared::read().ninaActive) {
      req->send(409, "text/plain", "A PC is controlling OAPA");
      return;
    }
    String axis = formField(req, "axis");
    String feed = formField(req, "feed");
    char cmd[web::kCommandLen];
    bool valid = axis.length() == 1 &&
                 web::buildJogCommand(axis.charAt(0), formField(req, "steps").toInt(),
                                      feed.length() ? feed.toInt() : web::kDefaultFeed, cmd, sizeof cmd);
    if (!valid) {
      req->send(400, "text/plain", "invalid jog");
      return;
    }
    if (!shared::enqueueCommand(cmd)) {
      req->send(503, "text/plain", "busy, try again");
      return;
    }
    req->send(202, "text/plain", cmd);
  });

  // STOP is accepted even while N.I.N.A. is active: stopping is always safe.
  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest *req) {
    shared::requestStop();
    shared::requestLoopStop();
    req->send(202, "text/plain", "stopping");
  });

  server.on("/api/wifi", HTTP_POST, [](AsyncWebServerRequest *req) {
    String ssid = formField(req, "ssid");
    String password = formField(req, "password");
    if (!net::saveCredentials(ssid.c_str(), password.c_str())) {
      req->send(400, "text/plain", "SSID 1-32 characters; password empty or 8-63 characters");
      return;
    }
    req->send(200, "text/plain", "saved");
    net::requestRestart();
  });

  server.on("/api/loop", HTTP_GET, [](AsyncWebServerRequest *req) {
    char body[512];
    if (formatLoopJson(shared::readLoop(), body, sizeof body) < 0) {
      req->send(500, "text/plain", "loop status too long");
      return;
    }
    req->send(200, "application/json", body);
  });

  server.on("/api/loop/start", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (shared::read().ninaActive) {
      req->send(409, "text/plain", "A PC is controlling OAPA");
      return;
    }
    if (calibrating()) {
      req->send(409, "text/plain", "calibration is running");
      return;
    }
    if (!shared::readLoop().sourceReady) {
      req->send(409, "text/plain", "no error source: start the PA on ASIAIR or enable the simulator");
      return;
    }
    shared::requestLoopStart();
    req->send(202, "text/plain", "starting");
  });

  server.on("/api/loop/stop", HTTP_POST, [](AsyncWebServerRequest *req) {
    shared::requestLoopStop();
    req->send(202, "text/plain", "stopping");
  });

  server.on("/api/sim", HTTP_POST, [](AsyncWebServerRequest *req) {
    auto number = [req](const char *name, double fallback) {
      String value = formField(req, name);
      return value.length() ? value.toDouble() : fallback;
    };
    SimRequest request;
    request.enable = formField(req, "enable") != "0";
    double factor = number("factor", 60);
    double az = number("az", 30);
    double alt = number("alt", -20);
    double noise = number("noise", 0.05);
    double refreshMs = number("refresh_ms", 4000);
    double backlash = number("backlash", 0);
    bool valid = factor >= 1 && factor <= 10000 && std::fabs(az) <= 600 && std::fabs(alt) <= 600 && noise >= 0 &&
                 noise <= 5 && refreshMs >= 500 && refreshMs <= 600000 && backlash >= 0 && backlash <= 30;
    if (!valid) {
      req->send(400, "text/plain", "invalid simulation parameters");
      return;
    }
    request.factorX = request.factorY = factor;
    request.params.x.trueStepsPerArcmin = request.params.y.trueStepsPerArcmin = factor;
    request.params.x.sign = number("sign_x", 1) < 0 ? -1 : 1;
    request.params.initialAzArcmin = az;
    request.params.initialAltArcmin = alt;
    request.params.noiseSigmaArcmin = noise;
    request.params.refreshMs = static_cast<uint32_t>(refreshMs);
    request.params.x.backlashArcmin = request.params.y.backlashArcmin = backlash;
    request.params.seed = static_cast<uint32_t>(millis()) | 1u;
    shared::requestSim(request);
    req->send(202, "text/plain", request.enable ? "simulation enabled" : "simulation disabled");
  });

  server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *req) { sendSettings(req, device::loopConfig()); });

  server.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest *req) {
    devset::LoopConfig c = device::loopConfig();
    c.factorX = numberField(req, "factorX", c.factorX);
    c.factorY = numberField(req, "factorY", c.factorY);
    c.toleranceArcmin = numberField(req, "tolerance", c.toleranceArcmin);
    c.userCapArcmin = numberField(req, "cap", c.userCapArcmin);
    double settle = numberField(req, "settleMs", c.settleMs);
    c.settleMs = settle < 0 ? 999999u : static_cast<uint32_t>(settle);
    c.feed = static_cast<int>(numberField(req, "feed", c.feed));
    c.calThresholdArcmin = numberField(req, "calThresholdArcmin", c.calThresholdArcmin);
    if (const char *error = devset::validateLoop(c)) {
      req->send(400, "text/plain", error);
      return;
    }
    bool hostGiven = req->hasParam("asiairHost", true);
    String host = formField(req, "asiairHost");
    if (hostGiven) {
      if (const char *error = devset::validateHost(host.c_str())) {
        req->send(400, "text/plain", error);
        return;
      }
    }
    device::saveLoopConfig(c);
    if (hostGiven) {
      device::saveAsiairHost(host.c_str());
      device::logEvent("settings", "ASIAIR address %s", host.length() ? host.c_str() : "automatic");
      asiairclient::reconnect();
    }
    device::logEvent("settings", "factors %.2f/%.2f, tolerance %.2f', cap %.0f'", c.factorX, c.factorY,
                     c.toleranceArcmin, c.userCapArcmin);
    sendSettings(req, c);
  });

  server.on("/api/direction", HTTP_GET, [](AsyncWebServerRequest *req) {
    char body[48];
    std::snprintf(body, sizeof body, "{\"invertX\":%s,\"invertY\":%s}", device::motorInverted('X') ? "true" : "false",
                  device::motorInverted('Y') ? "true" : "false");
    req->send(200, "application/json", body);
  });

  // Applied by the motor side as soon as that axis is standing still.
  server.on("/api/direction", HTTP_POST, [](AsyncWebServerRequest *req) {
    String axis = formField(req, "axis");
    String invert = formField(req, "invert");
    if ((axis != "X" && axis != "Y") || (invert != "0" && invert != "1")) {
      req->send(400, "text/plain", "axis must be X or Y, invert 0 or 1");
      return;
    }
    device::saveMotorInverted(axis.charAt(0), invert == "1");
    device::logEvent("direction", "%s motor %s", axis == "X" ? "AZ (X)" : "ALT (Y)", invert == "1" ? "inverted" : "normal");
    req->send(200, "text/plain", "ok");
  });

  server.on("/api/drivers", HTTP_GET, [](AsyncWebServerRequest *req) {
    devset::DriverConfig x, y;
    device::readActiveDrivers(x, y);
    char body[160];
    formatDriversJson(x, y, body, sizeof body);
    req->send(200, "application/json", body);
  });

  server.on("/api/drivers", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (shared::read().ninaActive) {
      req->send(409, "text/plain", "A PC is controlling OAPA");
      return;
    }
    String axisField = formField(req, "axis");
    char axis = axisField.length() == 1 ? axisField.charAt(0) : '?';
    devset::DriverConfig activeX, activeY;
    device::readActiveDrivers(activeX, activeY);
    devset::DriverConfig current = axis == 'Y' ? activeY : activeX;
    devset::DriverConfig c;
    c.runMa = static_cast<int>(numberField(req, "run", current.runMa));
    c.holdPct = static_cast<int>(numberField(req, "hold", current.holdPct));
    c.microsteps = static_cast<int>(numberField(req, "micro", current.microsteps));
    char commands[3][12];
    if (!devset::driverCommands(axis, c, commands)) {
      const char *error = devset::validateDriver(c);
      req->send(400, "text/plain", error ? error : "axis must be X or Y");
      return;
    }
    device::saveDriverConfig(axis, c);
    for (const char *command : {commands[0], commands[1], commands[2]}) {
      if (!shared::enqueueCommand(command)) {
        req->send(503, "text/plain", "busy, try again");
        return;
      }
    }
    device::logEvent("drivers", "%c: %d mA, hold %d %%, 1/%d", axis, c.runMa, c.holdPct, c.microsteps);
    req->send(202, "text/plain", "applying");
  });

  server.on("/api/move", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (shared::read().ninaActive) {
      req->send(409, "text/plain", "A PC is controlling OAPA");
      return;
    }
    if (loopRunning()) {
      req->send(409, "text/plain", "automatic alignment is running");
      return;
    }
    if (calibrating()) {
      req->send(409, "text/plain", "calibration is running");
      return;
    }
    devset::LoopConfig config = device::loopConfig();
    String axisField = formField(req, "axis");
    char axis = axisField.length() == 1 ? axisField.charAt(0) : '?';
    double factor = axis == 'Y' ? config.factorY : config.factorX;
    char cmd[web::kCommandLen];
    if (!devset::moveCommand(axis, numberField(req, "arcmin", 0), factor,
                             static_cast<int>(numberField(req, "feed", config.feed)), cmd, sizeof cmd)) {
      req->send(400, "text/plain", "invalid move");
      return;
    }
    if (!shared::enqueueCommand(cmd)) {
      req->send(503, "text/plain", "busy, try again");
      return;
    }
    req->send(202, "text/plain", cmd);
  });

  server.on("/api/events", HTTP_GET, [](AsyncWebServerRequest *req) {
    // Static buffers are safe: every handler runs on the single AsyncTCP task.
    static EventLog::Item items[25];
    static char body[4096];
    uint32_t last = 0;
    size_t n = device::readEvents(sinceParam(req), items, 25, last);
    if (formatEventsJson(items, n, last, body, sizeof body) < 0) {
      req->send(500, "text/plain", "events too long");
      return;
    }
    req->send(200, "application/json", body);
  });

  server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *req) {
    static HistoryLog::Item items[100];
    static char body[4096];
    uint32_t last = 0;
    size_t n = device::readHistory(sinceParam(req), items, 100, last);
    if (formatHistoryJson(items, n, last, body, sizeof body) < 0) {
      req->send(500, "text/plain", "history too long");
      return;
    }
    req->send(200, "application/json", body);
  });

  server.on("/api/wifi/reset", HTTP_POST, [](AsyncWebServerRequest *req) {
    net::clearCredentials();
    device::clearOtaPassword();  // the recovery path for a forgotten update password
    device::logEvent("wifi", "network and update password forgotten, restarting into OAPA-setup");
    req->send(200, "text/plain", "restarting into OAPA-setup");
    net::requestRestart();
  });

  server.on("/api/asiair", HTTP_GET, [](AsyncWebServerRequest *req) {
    AsiairSnapshot s = shared::readAsiair();
    asiairclient::Status link = asiairclient::status();
    s.connected = link.connected;
    std::snprintf(s.host, sizeof s.host, "%s", link.host);
    s.lines = link.lines;
    s.badLines = link.badLines;
    char body[512];
    if (formatAsiairJson(s, body, sizeof body) < 0) {
      req->send(500, "text/plain", "asiair status too long");
      return;
    }
    req->send(200, "application/json", body);
  });

  server.on("/api/calibrate", HTTP_GET, [](AsyncWebServerRequest *req) {
    char body[512];
    if (formatCalibrationJson(shared::readCalibration(), body, sizeof body) < 0) {
      req->send(500, "text/plain", "calibration status too long");
      return;
    }
    req->send(200, "application/json", body);
  });

  server.on("/api/calibrate/start", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (shared::read().ninaActive) {
      req->send(409, "text/plain", "A PC is controlling OAPA");
      return;
    }
    if (loopRunning()) {
      req->send(409, "text/plain", "automatic alignment is running");
      return;
    }
    if (!shared::readLoop().sourceReady) {
      req->send(409, "text/plain", "start the PA on ASIAIR and wait for the adjustment step");
      return;
    }
    shared::requestCalibrationStart();
    req->send(202, "text/plain", "calibrating");
  });

  server.on("/api/calibrate/stop", HTTP_POST, [](AsyncWebServerRequest *req) {
    shared::requestLoopStop();
    req->send(202, "text/plain", "stopping");
  });

  server.on("/api/calibrate/save", HTTP_POST, [](AsyncWebServerRequest *req) {
    CalibrationSnapshot cal = shared::readCalibration();
    if (std::strcmp(cal.state, "done") != 0) {
      req->send(409, "text/plain", "no calibration result to save");
      return;
    }
    devset::LoopConfig c = device::loopConfig();
    c.factorX = cal.xFactor;
    c.factorY = cal.yFactor;
    if (const char *error = devset::validateLoop(c)) {
      req->send(400, "text/plain", error);
      return;
    }
    device::saveLoopConfig(c);
    device::logEvent("settings", "calibrated factors %.2f/%.2f saved", c.factorX, c.factorY);
    sendSettings(req, c);
  });

  server.on("/api/ota", HTTP_GET, [](AsyncWebServerRequest *req) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    char body[128];
    std::snprintf(body, sizeof body, "{\"passwordSet\":%s,\"lockedMs\":%lu,\"partition\":\"%s\"}",
                  device::otaPasswordSet() ? "true" : "false",
                  static_cast<unsigned long>(otaLockout.remainingMs(millis())), running ? running->label : "");
    req->send(200, "application/json", body);
  });

  server.on("/api/ota/password", HTTP_POST, [](AsyncWebServerRequest *req) {
    uint32_t now = millis();
    if (otaLockout.locked(now)) {
      req->send(429, "text/plain", "too many wrong passwords, wait a minute");
      return;
    }
    if (device::otaPasswordSet() && !device::checkOtaPassword(formField(req, "current").c_str())) {
      otaLockout.fail(now);
      req->send(401, "text/plain", "current password is wrong");
      return;
    }
    String password = formField(req, "password");
    if (const char *error = ota::validatePassword(password.c_str())) {
      req->send(400, "text/plain", error);
      return;
    }
    otaLockout.succeed();
    device::setOtaPassword(password.c_str());
    device::logEvent("ota", "update password set");
    req->send(200, "text/plain", "saved");
  });

  server.on(
      "/api/ota", HTTP_POST,
      [](AsyncWebServerRequest *req) {
        auto *u = static_cast<OtaUpload *>(req->_tempObject);
        if (!u) {
          req->send(400, "text/plain", "no firmware file received");
          return;
        }
        if (u->result == OtaResult::Done) {
          device::logEvent("ota", "firmware written (%lu bytes), restarting", static_cast<unsigned long>(u->bytes));
          req->send(200, "text/plain", "updated, restarting");
          net::requestRestart();  // the gate stays claimed until the restart
        } else if (u->result == OtaResult::Writing) {
          if (otaGate.owns(u->id, millis())) {  // the body ended without its last chunk
            Update.abort();
            otaGate.release(u->id);
          }
          req->send(400, "text/plain", "incomplete firmware file");
        } else {
          req->send(u->code, "text/plain", u->message);
        }
      },
      [](AsyncWebServerRequest *req, const String &, size_t index, uint8_t *data, size_t len, bool final) {
        uint32_t now = millis();
        OtaUpload *u = static_cast<OtaUpload *>(req->_tempObject);
        if (index == 0) {
          u = static_cast<OtaUpload *>(std::calloc(1, sizeof(OtaUpload)));
          if (!u) return;
          req->_tempObject = u;
          u->id = otaNextId++;
          if (const char *busy = otaBusyReason()) {
            otaReject(u, 409, busy);
            return;
          }
          if (!otaGate.claim(u->id, now)) {
            otaReject(u, 409, "update already in progress");
            return;
          }
          if (!otaAuthorized(req, u)) {
            otaGate.release(u->id);
            return;
          }
          if (!ota::looksLikeEsp32Image(data, len)) {
            otaGate.release(u->id);
            otaReject(u, 400, "not an ESP32 firmware image");
            return;
          }
          if (Update.isRunning()) Update.abort();  // an abandoned upload the gate has let go
          if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
            otaGate.release(u->id);
            otaReject(u, 500, Update.errorString());
            return;
          }
          u->result = OtaResult::Writing;
          device::logEvent("ota", "update started");
        }
        if (!u || u->result != OtaResult::Writing) return;
        if (!otaGate.owns(u->id, now)) {  // abandoned for 15 s and taken by another upload
          otaReject(u, 409, "update taken over by another upload");
          return;
        }
        if (Update.write(data, len) != len) {
          otaReject(u, 500, Update.errorString());
          Update.abort();
          otaGate.release(u->id);
          return;
        }
        u->bytes += len;
        if (final) {
          if (Update.end(true)) {
            u->result = OtaResult::Done;
          } else {
            otaReject(u, 400, Update.errorString());  // checksum or size check failed: the old firmware stays
            otaGate.release(u->id);
          }
        }
      });

  // Captive portal: phones probe random URLs when joining OAPA-setup.
  server.onNotFound([](AsyncWebServerRequest *req) {
    if (net::inSetupMode()) req->redirect("http://192.168.4.1/");
    else req->send(404, "text/plain", "not found");
  });

  server.begin();
}

}  // namespace webserver
