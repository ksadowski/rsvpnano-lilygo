#include <Arduino.h>
#include <esp_log.h>

#include "app/App.h"
#include "board/BoardConfig.h"

App app;

void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_INFO);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000) {
    delay(10);
  }
  delay(50);
  Serial.println("[main] Before BoardConfig::begin()");
  BoardConfig::begin();
  Serial.println("[main] After BoardConfig::begin()");
  Serial.println("[main] app setup");
  app.begin();
}

void loop() {
  const uint32_t now = millis();
  app.update(now);
}
