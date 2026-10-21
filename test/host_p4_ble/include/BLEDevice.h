#pragma once
#include <Arduino.h>
#include <cassert>
#include <cstdint>
#include <vector>

struct BLEServer;
struct BLECharacteristic;
struct ble_gap_conn_desc {};
struct BLEServerCallbacks {
  virtual ~BLEServerCallbacks() = default;
  virtual void onConnect(BLEServer*) {}
  virtual void onDisconnect(BLEServer*) {}
};
struct BLECharacteristicCallbacks {
  virtual ~BLECharacteristicCallbacks() = default;
  virtual void onWrite(BLECharacteristic*) {}
  virtual void onSubscribe(BLECharacteristic*, ble_gap_conn_desc*, uint16_t) {}
};
struct BLECharacteristic {
  static constexpr unsigned PROPERTY_NOTIFY = 1;
  static constexpr unsigned PROPERTY_WRITE = 2;
  static constexpr unsigned PROPERTY_WRITE_NR = 4;
  BLECharacteristicCallbacks* callbacks = nullptr;
  String value;
  std::vector<String> notifications;
  String getValue() { return value; }
  void setValue(const uint8_t* data, size_t length) {
    value.assign(reinterpret_cast<const char*>(data), length);
  }
  void setCallbacks(BLECharacteristicCallbacks* cb) { callbacks = cb; }
  void notify() { notifications.push_back(value); }
};
struct BLEService {
  BLECharacteristic rx, tx;
  BLECharacteristic* createCharacteristic(const char*, unsigned properties) {
    return properties & BLECharacteristic::PROPERTY_NOTIFY ? &tx : &rx;
  }
  bool start() { return true; }
};
struct BLEServer {
  BLEServerCallbacks* callbacks = nullptr;
  BLEService service;
  void setCallbacks(BLEServerCallbacks* cb) { callbacks = cb; }
  void advertiseOnDisconnect(bool enabled) { assert(enabled); }
  BLEService* createService(const char*) { return &service; }
};
struct BLEAdvertising {
  bool succeeds = true;
  void addServiceUUID(const char*) {}
  void setScanResponse(bool enabled) { assert(enabled); }
  bool start() { return succeeds; }
};
struct BLEDevice {
  static inline BLEServer server;
  static inline BLEAdvertising advertising;
  static inline unsigned initCalls = 0, deinitCalls = 0;
  static bool setPins(int clk, int cmd, int d0, int d1, int d2, int d3, int reset) {
    assert(clk == 18 && cmd == 19 && d0 == 14 && d1 == 15 && d2 == 16 && d3 == 17 && reset == 54);
    return true;
  }
  static bool init(const char*) { ++initCalls; return true; }
  static void deinit() { ++deinitCalls; }
  static BLEServer* createServer() { return &server; }
  static BLEAdvertising* getAdvertising() { return &advertising; }
};
