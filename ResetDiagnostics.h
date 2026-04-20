#pragma once
#include <Arduino.h>
#include "esp_system.h"     // esp_reset_reason(), esp_reset_reason_t
#include "esp_sleep.h"      // esp_sleep_get_wakeup_cause()

struct ResetInfo {
  esp_reset_reason_t reason;          // raw reason from ROM
  esp_sleep_wakeup_cause_t wakeup;    // deep-sleep wake cause
  bool from_deep_sleep;               // true if woke from deep sleep
  bool unexpected;                    // your policy of "unexpected"
};

// Convert reset reason to readable text
static const char* resetReasonToStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN:     return "UNKNOWN";
    case ESP_RST_POWERON:     return "POWERON";
    case ESP_RST_EXT:         return "EXT_RESET (EN pin)";
    case ESP_RST_SW:          return "SW_RESET (ESP.restart())";
    case ESP_RST_PANIC:       return "PANIC/ABORT";
    case ESP_RST_INT_WDT:     return "INT_WDT";
    case ESP_RST_TASK_WDT:    return "TASK_WDT";
    case ESP_RST_WDT:         return "WDT (other)";
    case ESP_RST_DEEPSLEEP:   return "DEEPSLEEP wake";
    case ESP_RST_BROWNOUT:    return "BROWNOUT (low voltage)";
    case ESP_RST_SDIO:        return "SDIO reset";
    case ESP_RST_USB:         return "USB reset";
    case ESP_RST_JTAG:        return "JTAG reset";
    case ESP_RST_EFUSE:       return "EFUSE error";
    case ESP_RST_PWR_GLITCH:  return "PWR_GLITCH (supply glitch)";
    case ESP_RST_CPU_LOCKUP:  return "CPU_LOCKUP (double exception)";
    default:                  return "OTHER";
  }
}

// Convert wake cause to text (only meaningful after DEEPSLEEP)
static const char* wakeupCauseToStr(esp_sleep_wakeup_cause_t c) {
  switch (c) {
    case ESP_SLEEP_WAKEUP_UNDEFINED:  return "UNDEFINED";
    case ESP_SLEEP_WAKEUP_ALL:        return "ALL";
    case ESP_SLEEP_WAKEUP_EXT0:       return "EXT0";
    case ESP_SLEEP_WAKEUP_EXT1:       return "EXT1";
    case ESP_SLEEP_WAKEUP_TIMER:      return "TIMER";
    case ESP_SLEEP_WAKEUP_TOUCHPAD:   return "TOUCHPAD";
    case ESP_SLEEP_WAKEUP_ULP:        return "ULP";
    case ESP_SLEEP_WAKEUP_GPIO:       return "GPIO";
    case ESP_SLEEP_WAKEUP_UART:       return "UART";
    case ESP_SLEEP_WAKEUP_WIFI:       return "WIFI";
    case ESP_SLEEP_WAKEUP_COCPU:      return "COCPU";
    case ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG: return "COCPU_TRAP";
    case ESP_SLEEP_WAKEUP_VBAT_UNDER_VOLT: return "UNDER_VOLT";
#if ESP_IDF_VERSION_MAJOR >= 4
    case ESP_SLEEP_WAKEUP_BT:         return "BT";
#endif
    default:                          return "OTHER";
  }
}

// Policy: what counts as "unexpected"?
// Adjust to your needs (e.g., treat SW reset as expected if you call ESP.restart()).
static bool isUnexpected(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:
    case ESP_RST_DEEPSLEEP:
      return false; // normal boots
    case ESP_RST_SW:
      // If you only reboot via SW intentionally, mark this false.
      // If random SW resets occur, set to true.
      return false;
    case ESP_RST_BROWNOUT:  // low voltage
    case ESP_RST_PANIC:     // crash
    case ESP_RST_INT_WDT:   // interrupt watchdog
    case ESP_RST_TASK_WDT:  // task watchdog
    case ESP_RST_WDT:       // generic WDT
      return true;
    default:
      return true;
  }
}

// Capture reset info at boot
static ResetInfo readResetInfo() {
  ResetInfo info;
  info.reason = esp_reset_reason();
  info.from_deep_sleep = (info.reason == ESP_RST_DEEPSLEEP);
  info.wakeup = esp_sleep_get_wakeup_cause();
  info.unexpected = isUnexpected(info.reason);
  return info;
}


