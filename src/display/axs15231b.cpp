#include "display/axs15231b.h"

#include <esp_log.h>

#include "board/BoardConfig.h"

#include <SPI.h>

namespace {

constexpr uint32_t kSpiFrequency = 80000000;
static const char *kSt7796Tag = "st7796";

// The 222-px wide ST7796 GRAM is 320 columns wide; visible area starts at col 49.
constexpr uint16_t kCgramColOffset = 49;

bool gBusReady = false;
bool gBacklightOn = false;
uint8_t gBrightnessPercent = 100;

// V1.1 backlight: 16-level step-down controller driven by GPIO pulses.
// Each LOW→HIGH toggle decrements the brightness level by one step (ring buffer).
static uint8_t gBlLevel = 0;

void setBacklightLevel(uint8_t value) {
  if (value == 0) {
    digitalWrite(BoardConfig::PIN_LCD_BACKLIGHT, LOW);
    delay(3);
    gBlLevel = 0;
    return;
  }
  if (gBlLevel == 0) {
    digitalWrite(BoardConfig::PIN_LCD_BACKLIGHT, HIGH);
    gBlLevel = 16;
    delayMicroseconds(30);
  }
  constexpr uint8_t kSteps = 16;
  const int from = kSteps - gBlLevel;
  const int to = kSteps - value;
  const int num = (kSteps + to - from) % kSteps;
  for (int i = 0; i < num; i++) {
    digitalWrite(BoardConfig::PIN_LCD_BACKLIGHT, LOW);
    digitalWrite(BoardConfig::PIN_LCD_BACKLIGHT, HIGH);
  }
  gBlLevel = value;
}

void writeBacklightPwm() {
  if (!gBacklightOn) {
    setBacklightLevel(0);
    return;
  }
  const uint8_t pct = gBrightnessPercent < 1 ? 1 : gBrightnessPercent;
  const uint8_t level = static_cast<uint8_t>((static_cast<uint16_t>(pct) * 16U + 50U) / 100U);
  setBacklightLevel(level < 1 ? 1 : level);
}

void sendCommand(uint8_t cmd) {
  SPI.beginTransaction(SPISettings(kSpiFrequency, MSBFIRST, SPI_MODE0));
  digitalWrite(BoardConfig::PIN_LCD_DC, LOW);
  digitalWrite(BoardConfig::PIN_LCD_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(BoardConfig::PIN_LCD_CS, HIGH);
  digitalWrite(BoardConfig::PIN_LCD_DC, HIGH);
  SPI.endTransaction();
}

void sendCommandWithData(uint8_t cmd, const uint8_t *data, size_t len) {
  SPI.beginTransaction(SPISettings(kSpiFrequency, MSBFIRST, SPI_MODE0));
  digitalWrite(BoardConfig::PIN_LCD_DC, LOW);
  digitalWrite(BoardConfig::PIN_LCD_CS, LOW);
  SPI.transfer(cmd);
  if (len > 0 && data != nullptr) {
    digitalWrite(BoardConfig::PIN_LCD_DC, HIGH);
    SPI.writeBytes(data, len);
  }
  digitalWrite(BoardConfig::PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

}  // namespace

void axs15231bInit() {
  gBacklightOn = false;
  gBlLevel = 0;
  pinMode(BoardConfig::PIN_LCD_BACKLIGHT, OUTPUT);
  digitalWrite(BoardConfig::PIN_LCD_BACKLIGHT, LOW);

  if (!gBusReady) {
    SPI.begin(BoardConfig::PIN_LCD_SCLK, BoardConfig::PIN_LCD_MISO,
              BoardConfig::PIN_LCD_MOSI, -1);
    gBusReady = true;
  }

  pinMode(BoardConfig::PIN_LCD_CS, OUTPUT);
  digitalWrite(BoardConfig::PIN_LCD_CS, HIGH);
  pinMode(BoardConfig::PIN_LCD_DC, OUTPUT);
  digitalWrite(BoardConfig::PIN_LCD_DC, HIGH);

  pinMode(BoardConfig::PIN_LCD_RST, OUTPUT);
  digitalWrite(BoardConfig::PIN_LCD_RST, HIGH);
  delay(10);
  digitalWrite(BoardConfig::PIN_LCD_RST, LOW);
  delay(50);
  digitalWrite(BoardConfig::PIN_LCD_RST, HIGH);
  delay(120);

  // Full ST7796 init sequence (matches TFT_eSPI ST7796_Init.h).
  sendCommand(0x01);  // Software Reset
  delay(120);
  sendCommand(0x11);  // Sleep Out
  delay(120);

  {
    const uint8_t d[] = {0xC3};
    sendCommandWithData(0xF0, d, 1);  // Enable extension command 2 part I
  }
  {
    const uint8_t d[] = {0x96};
    sendCommandWithData(0xF0, d, 1);  // Enable extension command 2 part II
  }
  {
    // MADCTL: MY=1 (reverse row order) + BGR=1. MY flips the portrait row
    // direction so the 90°-CCW panel mounting aligns with DisplayManager's
    // UI_ROTATED_180=false coordinate transform.
    const uint8_t d[] = {0x88};
    sendCommandWithData(0x36, d, 1);
  }
  {
    const uint8_t d[] = {0x55};
    sendCommandWithData(0x3A, d, 1);  // COLMOD: 16-bit RGB565
  }
  {
    const uint8_t d[] = {0x01};
    sendCommandWithData(0xB4, d, 1);  // Column inversion: 1-dot
  }
  {
    const uint8_t d[] = {0x80, 0x02, 0x3B};
    sendCommandWithData(0xB6, d, 3);  // Display Function Control
  }
  {
    const uint8_t d[] = {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33};
    sendCommandWithData(0xE8, d, 8);  // Display Output Ctrl Adjust
  }
  {
    const uint8_t d[] = {0x06};
    sendCommandWithData(0xC1, d, 1);  // Power Control 2
  }
  {
    const uint8_t d[] = {0xA7};
    sendCommandWithData(0xC2, d, 1);  // Power Control 3
  }
  {
    const uint8_t d[] = {0x18};
    sendCommandWithData(0xC5, d, 1);  // VCOM Control
  }
  delay(120);

  {
    const uint8_t d[] = {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F,
                         0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B};
    sendCommandWithData(0xE0, d, 14);  // Gamma+
  }
  {
    const uint8_t d[] = {0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B,
                         0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B};
    sendCommandWithData(0xE1, d, 14);  // Gamma-
  }
  delay(120);

  {
    const uint8_t d[] = {0x3C};
    sendCommandWithData(0xF0, d, 1);  // Disable extension command 2 part I
  }
  {
    const uint8_t d[] = {0x69};
    sendCommandWithData(0xF0, d, 1);  // Disable extension command 2 part II
  }
  delay(120);

  // Set full-panel address window with CGRAM column offset.
  // Visible columns 0..221 live at physical GRAM cols 49..270.
  {
    const uint8_t d[] = {0x00, kCgramColOffset,
                         0x00, static_cast<uint8_t>(kCgramColOffset + 221)};
    sendCommandWithData(0x2A, d, 4);  // CASET: 49..270
  }
  {
    const uint8_t d[] = {0x00, 0x00, 0x01, 0xDF};
    sendCommandWithData(0x2B, d, 4);  // RASET: 0..479
  }

  sendCommand(0x29);  // Display On
  delay(20);

  ESP_LOGI(kSt7796Tag, "ST7796 SPI init complete");
}

void axs15231bSetBacklight(bool on) {
  gBacklightOn = on;
  writeBacklightPwm();
}

void axs15231bSetBrightnessPercent(uint8_t percent) {
  if (percent == 0) {
    percent = 1;
  } else if (percent > 100) {
    percent = 100;
  }
  gBrightnessPercent = percent;
  writeBacklightPwm();
}

void axs15231bSleep() {
  sendCommand(0x28);  // Display Off
  delay(20);
  sendCommand(0x10);  // Sleep In
  delay(5);
  gBacklightOn = false;
  setBacklightLevel(0);
}

void axs15231bWake() {
  sendCommand(0x11);  // Sleep Out
  delay(120);
  sendCommand(0x29);  // Display On
  delay(20);
  gBacklightOn = true;
  writeBacklightPwm();
}

void axs15231bPushColors(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                         const uint16_t *data) {
  if (data == nullptr || width == 0 || height == 0) {
    return;
  }

  // Apply CGRAM column offset so data lands in the visible area.
  const uint16_t xOff = static_cast<uint16_t>(x + kCgramColOffset);
  const uint8_t caset[] = {
      static_cast<uint8_t>(xOff >> 8),
      static_cast<uint8_t>(xOff),
      static_cast<uint8_t>((xOff + width - 1) >> 8),
      static_cast<uint8_t>(xOff + width - 1),
  };
  sendCommandWithData(0x2A, caset, 4);

  const uint8_t raset[] = {
      static_cast<uint8_t>(y >> 8),
      static_cast<uint8_t>(y),
      static_cast<uint8_t>((y + height - 1) >> 8),
      static_cast<uint8_t>(y + height - 1),
  };
  sendCommandWithData(0x2B, raset, 4);

  const size_t bytes = static_cast<size_t>(width) * height * sizeof(uint16_t);
  SPI.beginTransaction(SPISettings(kSpiFrequency, MSBFIRST, SPI_MODE0));
  digitalWrite(BoardConfig::PIN_LCD_DC, LOW);
  digitalWrite(BoardConfig::PIN_LCD_CS, LOW);
  SPI.transfer(0x2C);  // RAMWR
  digitalWrite(BoardConfig::PIN_LCD_DC, HIGH);
  SPI.writeBytes(reinterpret_cast<const uint8_t *>(data), bytes);
  digitalWrite(BoardConfig::PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}

