#include "board/BoardConfig.h"

#include <Wire.h>
#include <algorithm>
#include <driver/gpio.h>
#include <esp_sleep.h>

// T-Display-S3-Pro: SY6970 (BQ25895-compatible) PMU on I2C bus (SDA=5, SCL=6).

namespace BoardConfig {

namespace {

constexpr uint8_t kSy6970Address = 0x6A;
constexpr uint8_t kSy6970Reg02   = 0x02;  // ADC control: bit7=CONV_RATE, bit6=CONV_START
constexpr uint8_t kSy6970Reg05   = 0x05;  // bits[5:4]=WATCHDOG
constexpr uint8_t kSy6970Reg09   = 0x09;  // bit5=BATFET_DIS
constexpr uint8_t kSy6970Reg0E   = 0x0E;  // bits[6:0]=BATV (2304mV + 20mV/LSB)

bool sy6970Read(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(kSy6970Address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(kSy6970Address, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

bool sy6970Write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kSy6970Address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

uint8_t batteryPercentForVoltage(float voltage) {
  struct Point {
    float voltage;
    uint8_t percent;
  };
  constexpr Point kCurve[] = {
      {3.30f, 0},  {3.50f, 5},  {3.60f, 10}, {3.70f, 20},
      {3.75f, 30}, {3.80f, 40}, {3.85f, 50}, {3.90f, 60},
      {3.95f, 70}, {4.00f, 80}, {4.10f, 90}, {4.20f, 100},
  };
  constexpr size_t curveSize = sizeof(kCurve) / sizeof(kCurve[0]);
  if (voltage <= kCurve[0].voltage) {
    return kCurve[0].percent;
  }
  if (voltage >= kCurve[curveSize - 1].voltage) {
    return kCurve[curveSize - 1].percent;
  }
  for (size_t i = 1; i < curveSize; ++i) {
    if (voltage > kCurve[i].voltage) {
      continue;
    }
    const float span = kCurve[i].voltage - kCurve[i - 1].voltage;
    const float ratio = span <= 0.0f ? 0.0f : (voltage - kCurve[i - 1].voltage) / span;
    const int pct = static_cast<int>(
        kCurve[i - 1].percent + (kCurve[i].percent - kCurve[i - 1].percent) * ratio + 0.5f);
    return static_cast<uint8_t>(std::max(0, std::min(100, pct)));
  }
  return 0;
}

}  // namespace

void begin() {
  gpio_hold_dis(static_cast<gpio_num_t>(PIN_SD_CS));
  // After deep-sleep wake the digital GPIO peripheral has been reset, so the
  // pad reverts to INPUT after hold release.  Re-drive it HIGH immediately so
  // the SD card never sees CS asserted while the SPI bus is uninitialised.
  gpio_set_direction(static_cast<gpio_num_t>(PIN_SD_CS), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(PIN_SD_CS), 1);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_PWR_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON3, INPUT_PULLUP);
  pinMode(PIN_LCD_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_LCD_BACKLIGHT, LOW);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);

  // SY6970 init: clear BATFET_DIS (ensure battery FET is on after any previous power-off),
  // enable continuous ADC so battery voltage is always fresh, disable watchdog.
  uint8_t reg = 0;
  if (sy6970Read(kSy6970Reg09, reg)) {
    sy6970Write(kSy6970Reg09, static_cast<uint8_t>(reg & ~0x20));
  }
  if (sy6970Read(kSy6970Reg02, reg)) {
    sy6970Write(kSy6970Reg02, static_cast<uint8_t>(reg | 0x80));
  }
  if (sy6970Read(kSy6970Reg05, reg)) {
    sy6970Write(kSy6970Reg05, static_cast<uint8_t>(reg & ~0x30));
  }
}

void lightSleepUntilBootButton() {
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  gpio_wakeup_enable(static_cast<gpio_num_t>(PIN_BOOT_BUTTON), GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  if (Serial) Serial.flush();
  esp_light_sleep_start();
  gpio_wakeup_disable(static_cast<gpio_num_t>(PIN_BOOT_BUTTON));
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
}

bool readBatteryStatus(BatteryStatus &status) {
  status = BatteryStatus{};

  uint8_t reg02 = 0;
  if (!sy6970Read(kSy6970Reg02, reg02)) {
    return false;
  }
  // If continuous ADC stopped (e.g. after a spurious reset), restart it.
  if (!(reg02 & 0x80)) {
    sy6970Write(kSy6970Reg02, static_cast<uint8_t>(reg02 | 0x80));
    delay(300);
  }

  uint8_t reg0e = 0;
  if (!sy6970Read(kSy6970Reg0E, reg0e)) {
    return false;
  }
  const uint16_t batvRaw = reg0e & 0x7F;
  status.voltage = (2304.0f + batvRaw * 20.0f) / 1000.0f;
  status.present = (status.voltage >= 2.5f && status.voltage <= 4.6f);
  if (!status.present) {
    status.percent = 0;
    return false;
  }
  status.percent = batteryPercentForVoltage(status.voltage);
  return true;
}

bool releaseBatteryPowerHold() {
  // T-Display-S3-Pro: do NOT set BATFET_DIS. Setting it prevents the device from
  // restarting from battery (the FET stays off until USB is connected). Power-off
  // is achieved entirely via esp_deep_sleep_start() in App::enterPowerOff().
  return true;
}

void prepareForDeepSleep() {
  // GPIO14 is an RTC GPIO on ESP32-S3, so gpio_hold_en() alone persists through
  // deep sleep without needing gpio_deep_sleep_hold_en() (which would force-hold
  // ALL digital GPIOs including the shared SPI CLK/MOSI and break display init).
  gpio_set_direction(static_cast<gpio_num_t>(PIN_SD_CS), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(PIN_SD_CS), 1);
  gpio_hold_en(static_cast<gpio_num_t>(PIN_SD_CS));
}

}  // namespace BoardConfig
