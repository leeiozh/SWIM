#pragma once

#include <Arduino.h>

class DNSServer;
class WebServer;
class DataLogger;

class WifiPortal {
 public:
  /// Attaches the logger and leaves Wi-Fi disabled.
  void begin(DataLogger &logger);
  /// Services DNS and HTTP clients while the portal is active.
  void update();
  /// Toggles the access point and returns its resulting state.
  bool toggle();
  /// Starts the local access point and web server.
  void start();
  /// Stops the server and powers down Wi-Fi.
  void stop();
  /// Reports whether the portal is active.
  bool active() const { return active_; }
  /// Reports whether a station is connected.
  bool connected() const;
  /// Returns the short portal state label.
  const char *statusText() const { return active_ ? "ON" : "OFF"; }
  /// Returns the advertised network name or a placeholder.
  String networkName() const { return active_ ? setupSsid() : "--"; }
  /// Returns the access-point address or a placeholder.
  String ipText() const;
  /// Returns the access-point SSID.
  static const char *setupSsid() { return "SWIM"; }
  /// Returns the access-point password.
  static const char *setupPassword() { return "swimswim"; }

 private:
  /// Registers HTTP routes once.
  bool configureServer();
  /// Builds the log-management page.
  String mainPage() const;
  /// Streams one validated binary log.
  void sendLogFile();
  /// Streams selected logs as one combined CSV.
  void sendCombinedCsv();
  /// Deletes selected inactive logs.
  void deleteSelected();
  /// Collects and validates selected log names.
  bool selectedFiles(String *names, int &count) const;

  DNSServer *dns_ = nullptr;
  WebServer *server_ = nullptr;
  DataLogger *logger_ = nullptr;
  bool active_ = false;
  bool serverConfigured_ = false;
};
