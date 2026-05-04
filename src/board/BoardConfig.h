#pragma once

#include <Arduino.h>
#include <XPowersLib.h>

namespace BoardConfig {

// LILYGO T-Display-S3-Pro (ST7796 SPI display, CST226SE touch, SY6970 PMU)
constexpr int PIN_BOOT_BUTTON = 0;
constexpr int PIN_PWR_BUTTON = 12;   // btn2
constexpr int PIN_BUTTON3 = 16;      // btn3
constexpr int PIN_LED = 38;          // onboard green LED (shared with camera LED)
constexpr int PIN_BATTERY_ADC = -1;  // No direct ADC; SY6970 PMU is I2C-only

constexpr int PIN_LCD_CS = 39;
constexpr int PIN_LCD_SCLK = 18;
constexpr int PIN_LCD_MOSI = 17;
constexpr int PIN_LCD_MISO = 8;
constexpr int PIN_LCD_DC = 9;
constexpr int PIN_LCD_RST = 47;
constexpr int PIN_LCD_BACKLIGHT = 48;

constexpr int PANEL_NATIVE_WIDTH = 222;
constexpr int PANEL_NATIVE_HEIGHT = 480;
constexpr int DISPLAY_WIDTH = 480;
constexpr int DISPLAY_HEIGHT = 222;
constexpr bool UI_ROTATED_180 = false;

constexpr int PIN_SD_CS = 14;        // SD shares SPI bus with TFT
constexpr int PIN_I2C_SDA = 5;
constexpr int PIN_I2C_SCL = 6;
constexpr int PIN_TOUCH_RST = 13;
constexpr int PIN_TOUCH_IRQ = 21;

struct BatteryStatus {
  bool present = false;
  float voltage = 0.0f;
  uint8_t percent = 0;
  bool isUsbConnected = false;
  bool isCharging = false;
  float vbusVoltage = 0.0f;
  uint16_t chargeCurrentLimit = 0;  // Set charge current limit (REG10), not actual current
  bool ntcFault = false;
};

void begin();
void lightSleepUntilBootButton();
void prepareForDeepSleep();
bool readBatteryStatus(BatteryStatus &status);
bool releaseBatteryPowerHold();

}  // namespace BoardConfig
