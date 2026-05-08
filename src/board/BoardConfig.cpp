#include "board/BoardConfig.h"

#include <algorithm>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <Wire.h>

// T-Display-S3-Pro: SY6970 (BQ25895-compatible) PMU on I2C bus (SDA=5, SCL=6).

namespace BoardConfig {

namespace {

PowersSY6970 PMU;

uint8_t batteryPercentForVoltage(float voltage) {
  struct Point {
    float voltage;
    uint8_t percent;
  };

  // Ostateczna krzywa LiHV (3.40V - 4.20V)
// Zoptymalizowana pod kątem płynności wskazań i ochrony ogniwa
constexpr Point kCurve[] = {
    {3.40f, 0},   // 0% - Bezpieczny próg odcięcia przy włączonym ekranie
    {3.45f, 2},   
    {3.50f, 5},   
    {3.55f, 9},   
    {3.60f, 14},  
    {3.65f, 20},  
    {3.70f, 28},  // Okolice 30% - moment na ostrzeżenie o niskim stanie
    {3.75f, 37},
    {3.80f, 48},  // Napięcie nominalne 3.8V wypada teraz blisko połowy (48%)
    {3.85f, 58},
    {3.90f, 68},
    {4.00f, 82},  
    {4.05f, 88},
    {4.10f, 93},  
    {4.15f, 97},  // Dla Twojego 4.14V zobaczysz teraz ok. 96% - bardzo naturalnie
    {4.20f, 100}, // Twoje obecne maksimum ładowania
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
  ESP_LOGI("BoardConfig", "begin() called");
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

  // Initialize PMU as in example
  ESP_LOGI("PMU", "Initializing...");
  if (!PMU.init(Wire, PIN_I2C_SDA, PIN_I2C_SCL, SY6970_SLAVE_ADDRESS)) {
    ESP_LOGI("PMU", "Init failed (may already be initialized by board)");
  } else {
    ESP_LOGI("PMU", "Init success");
  }
  // Configure as in example
  PMU.setInputCurrentLimit(1000);
  PMU.setChargeTargetVoltage(4352);
  PMU.setPrechargeCurr(64);
  PMU.setChargerConstantCurr(192);
  PMU.enableStatLed();
  PMU.enableMeasure();
  ESP_LOGI("PMU", "Configuration done");
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

  // PMU already initialized by XPowersCommon (board support)
  // Just use getter methods
  status.isUsbConnected = PMU.isVbusIn();
  status.vbusVoltage = PMU.getVbusVoltage() / 1000.0f;
  status.voltage = PMU.getBattVoltage() / 1000.0f;
  status.chargeCurrentLimit = PMU.getChargerConstantCurr();
  status.ntcFault = (PMU.getNTCStatus() != 0);

  uint8_t chrgStatus = PMU.chargeStatus();
  status.chargeStatus = chrgStatus;
  status.isCharging = (chrgStatus == 0x01 || chrgStatus == 0x02);

  ESP_LOGI("PMU", "raw: usb=%d vbus=%.2fV bat=%.3fV chrg=0x%02X cur=%umA ntc=%d",
           static_cast<int>(status.isUsbConnected),
           status.vbusVoltage,
           status.voltage,
           chrgStatus,
           static_cast<unsigned int>(status.chargeCurrentLimit),
           static_cast<int>(status.ntcFault));

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
