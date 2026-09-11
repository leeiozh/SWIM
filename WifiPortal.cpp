#include "WifiPortal.h"
#include <DNSServer.h>
#include <SD.h>
#include <WebServer.h>
#include <WiFi.h>
#include "DataLogger.h"
#include "SWIMConfig.h"

namespace {
const char PAGE_HEAD[] PROGMEM = R"HTML(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>SWIM files</title><style>body{font-family:sans-serif;max-width:36rem;margin:2rem auto;padding:0 1rem;background:#07131c;color:#eaf8ff}button,.download{display:inline-block;background:#08bddd;color:#00151c;border:0;font-weight:bold;padding:.7rem;margin:.35rem;text-decoration:none;font-size:1rem}.danger{background:#ef554f;color:white}.log{padding:.7rem 0;border-bottom:1px solid #36515b}small{color:#b9d6df}</style></head><body><h1>SWIM files</h1><form method="get" action="/export.csv"><p><button type="button" onclick="document.querySelectorAll('input[name=file]').forEach(x=>x.checked=true)">Select all</button><button type="submit">Download selected as one CSV</button><button type="submit" class="danger" formmethod="post" formaction="/delete" onclick="return confirm('Delete selected logs permanently?')">Delete selected</button></p>)HTML";
const char PAGE_TAIL[] PROGMEM = R"HTML(</form><p><small>The active log cannot be deleted. The browser chooses the download folder.</small></p></body></html>)HTML";
enum RecordType : uint8_t { RECORD_GNSS = 2, RECORD_WAVES = 3, RECORD_EVENT = 5, RECORD_SUMMARY = 6 };
struct __attribute__((packed)) RecordHeader { uint8_t type, version; uint16_t payloadSize; uint64_t monotonicUs, utcMs; };
struct __attribute__((packed)) GnssPayload { uint8_t flags, fixQuality, satellitesUsed, signalsVisible, bestCn0DbHz, reserved[3]; float hdop; double latitudeDeg, longitudeDeg; float altitudeM, sogMps, cogDeg; };
struct __attribute__((packed)) WavePayload { uint8_t flags, reserved[3]; float hsM, peakFrequencyHz, peakPeriodS, tm02S, directionFromDeg, gnssAltitudeHsM, gnssAltitudeTzS; uint32_t imuSamples; uint16_t gnssAltitudeSamples, reserved2; };
struct __attribute__((packed)) SummaryPayload {
  uint8_t flags, reserved[3]; double latitudeDeg, longitudeDeg;
  float altitudeM, sogMps, cogDeg, hsM, peakFrequencyHz, peakPeriodS, tm02S, directionFromDeg;
  uint8_t spectrum[32]; int8_t a1[32], b1[32], a2[32], b2[32];
};
/// Normalizes and validates a requested binary-log path.
bool validLogName(String &name) { if (!name.startsWith("/")) name = "/" + name; return name.indexOf("..") < 0 && name.endsWith(".bin"); }
}

void WifiPortal::begin(DataLogger &logger) { logger_ = &logger; WiFi.mode(WIFI_OFF); }
bool WifiPortal::connected() const { return active_ && WiFi.softAPgetStationNum() > 0; }
String WifiPortal::ipText() const { return active_ ? WiFi.softAPIP().toString() : "--"; }
void WifiPortal::configureServer() {
  if (serverConfigured_) return; dns_ = new DNSServer(); server_ = new WebServer(80);
  server_->on("/", HTTP_GET, [this]() { server_->send(200, "text/html", mainPage()); });
  server_->on("/download", HTTP_GET, [this]() { sendLogFile(); });
  server_->on("/export.csv", HTTP_GET, [this]() { sendCombinedCsv(); });
  server_->on("/delete", HTTP_POST, [this]() { deleteSelected(); });
  server_->onNotFound([this]() { server_->sendHeader("Location", "http://192.168.4.1/", true); server_->send(302, "text/plain", ""); });
  serverConfigured_ = true;
}
void WifiPortal::start() {
  if (active_) return; configureServer(); WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(setupSsid(), setupPassword())) { Serial.println("WIFI: failed to start AP"); WiFi.mode(WIFI_OFF); return; }
  dns_->start(53, "*", WiFi.softAPIP()); server_->begin(); active_ = true;
  Serial.printf("WIFI: ON, SSID %s, http://%s/\n", setupSsid(), ipText().c_str());
}
void WifiPortal::stop() { if (!active_) return; server_->stop(); dns_->stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); active_ = false; Serial.println("WIFI: OFF"); }
bool WifiPortal::toggle() { if (active_) stop(); else start(); return active_; }
void WifiPortal::update() { if (active_) { dns_->processNextRequest(); server_->handleClient(); } }

String WifiPortal::mainPage() const {
  String html(FPSTR(PAGE_HEAD)); File root = SD.open("/"); File file = root ? root.openNextFile() : File(); bool found = false;
  while (file) {
    String name = file.name(); if (!name.startsWith("/")) name = "/" + name;
    if (!file.isDirectory() && name.endsWith(".bin")) {
      found = true; const bool activeLog = logger_ && name == logger_->fileName();
      html += "<div class='log'><label><input type='checkbox' name='file' value='" + name + "'";
      html += "> " + name + " &mdash; " + String((unsigned long)file.size() / 1024UL) + " KB";
      if (activeLog) html += " (recording)";
      html += "</label> <a class='download' href='/download?name=" + name + "'>BIN</a></div>";
    }
    file = root.openNextFile();
  }
  if (!found) html += "<p>No log files yet.</p>"; html += FPSTR(PAGE_TAIL); return html;
}
void WifiPortal::sendLogFile() {
  String name = server_->arg("name"); if (!validLogName(name)) { server_->send(400, "text/plain", "Invalid log name"); return; }
  if (logger_ && name == logger_->fileName()) logger_->flush();
  File file = SD.open(name, FILE_READ); if (!file || file.isDirectory()) { server_->send(404, "text/plain", "Log not found"); return; }
  server_->sendHeader("Content-Disposition", "attachment; filename=\"" + name.substring(1) + "\""); server_->streamFile(file, "application/octet-stream"); file.close();
}
bool WifiPortal::selectedFiles(String *names, int &count) const {
  count = 0; for (int i = 0; i < server_->args() && count < 32; ++i) if (server_->argName(i) == "file") { String name = server_->arg(i); if (validLogName(name)) names[count++] = name; } return count > 0;
}
void WifiPortal::sendCombinedCsv() {
  String names[32]; int count = 0; if (!selectedFiles(names, count)) { server_->send(400, "text/plain", "No files selected"); return; }
  if (logger_) logger_->flush();
  server_->sendHeader("Content-Disposition", "attachment; filename=\"swim_export.csv\""); server_->setContentLength(CONTENT_LENGTH_UNKNOWN); server_->send(200, "text/csv; charset=utf-8", "");
  String csvHeader = "source,type,monotonic_us,utc_ms,latitude_deg,longitude_deg,altitude_m,sog_kn,cog_deg,hs_m,fp_hz,tp_s,tm02_s,direction_from_deg";
  const float spectrumDf = SwimConfig::WAVE_RATE_HZ / SwimConfig::FFT_SIZE;
  const int lastSpectrumBin = (int)floorf(SwimConfig::FMAX_HZ / spectrumDf);
  auto storedFrequency = [&](int i) {
    return constrain((int)lroundf((SwimConfig::FMAX_HZ * i / 31.0f) / spectrumDf),
                     0, lastSpectrumBin) * spectrumDf;
  };
  for (int i = 0; i < 32; ++i)
    csvHeader += ",s_norm_" + String(storedFrequency(i), 3) + "Hz";
  for (const char *coefficient : {"a1", "b1", "a2", "b2"})
    for (int i = 0; i < 32; ++i)
      csvHeader += "," + String(coefficient) + "_" +
                   String(storedFrequency(i), 3) + "Hz";
  csvHeader += ",event\r\n";
  server_->sendContent(csvHeader);
  uint8_t payload[256]; char line[384];
  for (int n = 0; n < count; ++n) {
    File file = SD.open(names[n], FILE_READ); if (!file || file.size() < 44) continue; file.seek(44);
    while (file.available() >= (int)sizeof(RecordHeader)) {
      RecordHeader r; if (file.read((uint8_t *)&r, sizeof(r)) != sizeof(r) || r.payloadSize > sizeof(payload) || file.read(payload, r.payloadSize) != r.payloadSize) break;
      int length = 0;
      if (r.type == RECORD_SUMMARY && r.payloadSize == sizeof(SummaryPayload)) {
        const SummaryPayload &p = *(const SummaryPayload *)payload;
        String row; row.reserve(1800);
        snprintf(line, sizeof(line), "%s,SUMMARY,%llu,%llu,%.8f,%.8f,%.3f,%.3f,%.2f,%.4f,%.5f,%.3f,%.3f,%.2f",
                 names[n].c_str()+1, r.monotonicUs, r.utcMs, p.latitudeDeg,
                 p.longitudeDeg, p.altitudeM, p.sogMps * 1.943844f, p.cogDeg,
                 p.hsM, p.peakFrequencyHz, p.peakPeriodS, p.tm02S, p.directionFromDeg);
        row = line;
        for (uint8_t value : p.spectrum) row += "," + String(value);
        for (int8_t value : p.a1) row += "," + String((int)value);
        for (int8_t value : p.b1) row += "," + String((int)value);
        for (int8_t value : p.a2) row += "," + String((int)value);
        for (int8_t value : p.b2) row += "," + String((int)value);
        row += ",\r\n"; server_->sendContent(row);
      } else if (r.type == RECORD_EVENT && r.payloadSize == sizeof(uint32_t)) {
        uint32_t code; memcpy(&code,payload,sizeof(code)); const char *event=code==1?"MARK":(code==2?"SET_ZERO":"UNKNOWN");
        snprintf(line, sizeof(line), "%s,EVENT,%llu,%llu", names[n].c_str()+1, r.monotonicUs, r.utcMs);
        String row(line);
        for (int i = 0; i < 171; ++i) row += ',';
        row += event; row += "\r\n"; server_->sendContent(row);
      } else if (r.type == RECORD_GNSS && r.payloadSize == sizeof(GnssPayload)) {
        const GnssPayload &p = *(const GnssPayload *)payload; length = snprintf(line,sizeof(line),"%s,GNSS,%llu,%llu,%.8f,%.8f,%.6g,%.4g,%.6g\r\n",names[n].c_str()+1,r.monotonicUs,r.utcMs,p.latitudeDeg,p.longitudeDeg,p.altitudeM,p.sogMps*1.943844f,p.cogDeg);
        server_->sendContent(String(line,length));
      }
      delay(0);
    }
    file.close();
  }
  server_->sendContent("");
}
void WifiPortal::deleteSelected() {
  String names[32]; int count=0; if (!selectedFiles(names,count)) { server_->send(400,"text/plain","No files selected"); return; }
  int deleted=0,refused=0; for(int i=0;i<count;++i){ if(logger_&&names[i]==logger_->fileName()){refused++;continue;} if(SD.remove(names[i]))deleted++; }
  String html="<html><meta name='viewport' content='width=device-width'><body><h2>Deleted "; html+=deleted; html+=" file(s)</h2>"; if(refused)html+="<p>The active log was protected.</p>"; html+="<p><a href='/'>Back</a></p></body></html>"; server_->send(200,"text/html",html);
}
