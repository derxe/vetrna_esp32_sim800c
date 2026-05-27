#define UART_DEBUG_SIM_TX_PIN  39
#define UART_DEBUG_SIM_RX_PIN  37 

#include <SerialMod0.h>
SerialMod0 SerialUart0(UART_NUM_0, UART_DEBUG_SIM_TX_PIN, UART_DEBUG_SIM_RX_PIN); 
#define SerialDBG SerialUart0

#define Serial_print(x)    do { SerialDBG.print(x); /* Serial1.print(x);*/ } while (0)
#define Serial_println(x)  do { SerialDBG.println(x); /* Serial1.println(x);*/ } while (0)
#define Serial_write(x)  do { SerialDBG.write(x); /*Serial1.write(x);*/ } while (0)

// pin definitions:
#define SIM_RX_PIN 34
#define SIM_TX_PIN 35

// watchdog pins 
#define WAKE_PIN       1
#define DONE_PIN       2
#define DONE_PULSE_MS  5    // duration of the pulse to reset the watchdog timer

#define BUTTON_INFO_PIN   0
#define BUTTON_SEND_PIN   40
#define GPRS_ON_PIN       21   // MOS FET turn on pin
#define GPRS_POWER_PIN    33   // PWX pin on the SIM800C board
#define V_BATT_PIN        18
#define V_SOALR_PIN       17

#define BLINK_LED_PIN      15 
#define SPIN_LED_PIN       38
#define ERROR_LED_PIN      36


#include <Preferences.h>
#include "unix_compile_time.h"
#include "esp_log.h" 
#include <SoftwareSerial.h>
#include "SimpleButton.h"
#include "esp_timer.h"
#include "3d_mag_dir_sensor.h"
#include <Wire.h>
#include "myTime.h"
#include <sys/time.h>
#include "ErrorLogger.h"
#include "ResetDiagnostics.h"
#include "MyPrefs.h"
#include "FloatRunningAverage.h"
#include <string.h>  // for memcpy
#include "SH40_SoftI2C.h"
#include "speed_hal_sensor.h"
#include "sim800c_comunicator.h"
#include <driver/rtc_io.h>

typedef struct {
    int16_t avg;    // negative errors means errors
    //uint16_t max; // we dont need to log max speed
    int16_t dir;      // from 0 - 360, negative values for errors 
    //uint16_t ts;   // we only log last and first log instead of logging every timestamp
} WindSample;


//#define PRINT_SIM_COMM 
//#define PRINT_MAGNET_READ_DEBUG

//#define SerialDBG Serial


#define POWER_GPRS_BOARD_ON()  do { digitalWrite(GPRS_ON_PIN, HIGH); } while (0)
#define POWER_GPRS_BOARD_OFF() do { digitalWrite(GPRS_ON_PIN, LOW);  } while (0)


#define HAL_SENSOR_PIN     7
#define HAL_POWER_PIN      6
SpeedHalSensor speedHalSensor(HAL_SENSOR_PIN, HAL_POWER_PIN);

// this code block is execuded only once used for single time prints 
#define RUN_ONCE(code)            \
do {                              \
    static bool _once = false;    \
    if (!_once) {                 \
        _once = true;             \
        code;                     \
    }                             \
} while(0)


// Define a custom TwoWire instance
#define DIR_SENSOR_SDA_PIN 4
#define DIR_SENSOR_SCL_PIN 3
#define DIR_SENSOR_POWER_PIN 5
MagDirSensor3D windDirSensor(DIR_SENSOR_SDA_PIN, DIR_SENSOR_SCL_PIN, DIR_SENSOR_POWER_PIN);

#define SH40_1_PWR_PIN   10
#define SH40_1_SDA_PIN   8
#define SH40_1_SCL_PIN   9
SH40SoftI2C sht1(SH40_1_SDA_PIN, SH40_1_SCL_PIN, SH40_1_PWR_PIN, 1);

#define SH40_2_PWR_PIN   12
#define SH40_2_SDA_PIN   11
#define SH40_2_SCL_PIN   13
SH40SoftI2C sht2(SH40_2_SDA_PIN, SH40_2_SCL_PIN, SH40_2_PWR_PIN, 2);

SimpleButton buttonInfo;
SimpleButton buttonSend;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t lastNow = 0;

void IRAM_ATTR onReadDirection(void* arg);
void IRAM_ATTR onReadHal(void* arg);
void IRAM_ATTR onReadSpeed(void* arg);
void IRAM_ATTR onBlinkLed(void* arg);
void IRAM_ATTR onSpinLed(void* arg);
void IRAM_ATTR onErrorLed(void* arg);
void IRAM_ATTR onStoreWindData(void* arg);
void IRAM_ATTR onResetWatchdogTimer(void* arg);

void clickInfo();
void clickSend();
bool updateSerial();

#define DEEP_SLEEP_DURATION  (3600ULL * 1000*1000) // value in microseconds so: one hour
//#define DEEP_SLEEP_DURATION  5*60 * 1000 * 1000  // 20 seconds
RTC_DATA_ATTR time_t sleepUntil = 0;      // stores how long do we want to sleep for so we know if we have to continue sleeping when waking up  

void printVersionAndCompileDate() {
  Serial_print("Compiled on ");
  Serial_print(__DATE__);
  Serial_print(" at ");
  Serial_print(__TIME__);
  Serial_print(" unix:");
  Serial_println(BUILD_UNIX_TIME);

  Serial_print("Version:"); Serial_println(prefs.version);
}


void welcomTurnOnBlink() {
  digitalWrite(SPIN_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  digitalWrite(BLINK_LED_PIN, LOW);
  delay(500);
  digitalWrite(SPIN_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, HIGH);
  digitalWrite(BLINK_LED_PIN, HIGH);
  delay(500);
}


// Pretty print (for Serial debug)
static void printResetInfo(const ResetInfo& i) {
  Serial_print("[Reset] reason=");
  Serial_print(resetReasonToStr(i.reason));
  Serial_print(" (");
  Serial_print((int)i.reason);
  Serial_println(")");

  if (i.from_deep_sleep) {
    Serial_print("[Reset] wakeup=");
    Serial_print(wakeupCauseToStr(i.wakeup));
    Serial_print(" (");
    Serial_print((int)i.wakeup);
    Serial_println(")");
  }

  Serial_print("[Reset] unexpected=");
  Serial_println(i.unexpected ? "YES" : "no");
}

ErrorLogger elog;

// pass in BUILD_UNIX_TIME so that we update the preferences on each new compile 
PrefBlob<AppPrefs> store(BUILD_UNIX_TIME);

void savePrefs(const AppPrefs& in) {
  int save = store.save(prefs);
  Serial_print("Save status: "); Serial_println(save);
  if (save < 0) {
    Serial_println("ERROR: Save failed");
  }
}

void loadPreferences() {
  store.begin();

  AppPrefs prefsLoaded;
  int loaded = store.load(prefsLoaded);
  Serial_print("Prefs loaded status: "); Serial_println(loaded);

  if (loaded == -7) {
    Serial_println("First boot using/saving default preferences");
    savePrefs(prefs);
  } else if (loaded < 0) {
    Serial_println("ERROR: Error loading ... saving default preferences");
    savePrefs(prefs);
  } else if(prefsLoaded.load_def_prefs == 1) {
    Serial_println("load_def_prefs = 1, using default preferences.");
    savePrefs(prefs); // save the newly set flag
  } else {
    Serial_println("Prefs loaded ok! Using that"); 
    prefs = prefsLoaded;
  }
  
  //printPreferences();

  store.end();
}

void savePreferences() {
  store.begin();
  Serial_println("Saving preferences");

  int save = store.save(prefs);
  Serial_print("Save status: "); Serial_println(save);
  if (save < 0) {
    Serial_println("ERROR: Save failed");
  }

  printPreferences();
  store.end();
}

void errorLedBlink(uint8_t nblinks, uint16_t blinkDelay=200) {
  while(nblinks-- > 0) {
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(blinkDelay);
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(blinkDelay);
  }
}

void checkSensorsConnected() {
  if(!windDirSensor.isConnected()) {
    elog.log(ErrorLogger::ERR_DIR_NOT_CONNECTED);
    Serial_println("ERROR: Wind direction sensor is not connected!");
    errorLedBlink(2, 300);
    delay(500);
  }

  if(!speedHalSensor.isConnected()) {
    elog.log(ErrorLogger::ERR_WIND_NOT_CONNECTED);
    Serial_println("ERROR: Speed HAL sensor is not connected!");
    errorLedBlink(3, 150);
  }
}

bool accurateTimeSet = false;
bool hasSendAfterTurnOn = false; 
volatile bool updateSerialEnabled = false;

// gets called each 5 seconds and resets the external watchdog timer
void IRAM_ATTR onResetWatchdogTimer(void* arg) {
    reset_watchdog_timer();
}

static void reset_watchdog_timer() {
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, HIGH);
  delay(DONE_PULSE_MS);
  digitalWrite(DONE_PIN, LOW);
  pinMode(DONE_PIN, INPUT_PULLDOWN);
}


void calibrateWindDirectionNorth() {
  Serial_println("Button send pressed down. Calibrating vane north value!");
  digitalWrite(SPIN_LED_PIN, HIGH);
  int angle = 0;
  windDirSensor.setNorthOffset(0);
  if(!performFullWindDirRead(angle)) return;
  digitalWrite(SPIN_LED_PIN, LOW);
  delay(100);
  digitalWrite(SPIN_LED_PIN, HIGH);
  delay(300);
  digitalWrite(SPIN_LED_PIN, LOW);
  delay(100);
  digitalWrite(SPIN_LED_PIN, HIGH);
  delay(300);
  digitalWrite(SPIN_LED_PIN, LOW);

  Serial_print("Calibrated north: "); Serial_println(angle);

  windDirSensor.setNorthOffset(angle);
  windDirSensor.saveNorthOffset();
}



void setup() {
  pinMode(BLINK_LED_PIN, OUTPUT);  digitalWrite(BLINK_LED_PIN, HIGH); // on
  delay(500);

  reset_watchdog_timer();

  hasSendAfterTurnOn = false;

  elog.init();
  ResetInfo ri = readResetInfo();

  switch (ri.reason) {
    case ESP_RST_BROWNOUT:       
      elog.log(ErrorLogger::ERR_RESET_BROWNOUT); break;

    case ESP_RST_PANIC:          
      elog.log(ErrorLogger::ERR_RESET_PANIC); break;

    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:            
      elog.log(ErrorLogger::ERR_RESET_WDT); break;

    default:                     
      elog.log(ErrorLogger::ERR_RESET_UNEXPECTED); break;

    /* expected bootups  */
    case ESP_RST_POWERON:        
       elog.log(ErrorLogger::LOG_RESET_POWERON); break;

    case ESP_RST_SW:        
       elog.log(ErrorLogger::LOG_RESET_SW); break;

    case ESP_RST_DEEPSLEEP:                
      break;
  }

  //Serial.begin(115200);

  SerialDBG.begin(115200);
  Serial_println();

  Serial_println("### PROGRAM START! ###");
  printResetInfo(ri);
  printVersionAndCompileDate();
  restoreTimeIfScheduledReset();

  loadPreferences();

  esp_log_level_set("i2c.master", ESP_LOG_NONE); 


  // TODO if the reason is unexpected_reset get more information about it 
  // We should also check ESP_RST_EXT and treat it as error reason 
  // the RST_EXT is when EN pin is pulled down 

  // TODO also log which part of the program was in when the reset happened to debug where it crashed


  if(sleepUntil == 0) {
    Serial_println("Fresh start. Not from sleep.");
  } else {
    // timeWas set before sleep so we can check if is time to wake up already!
    int64_t now_s = now_rtc_s();
    set_system_time_unix((time_t)now_s);
    accurateTimeSet = true;
    Serial_print("Woke up from deep sleep. Current time:"); Serial_println(getFormattedTimeLibString());
    Serial_print("We need to sleep until: "); Serial_println(getFormattedUnixTime(sleepUntil));
    
    int64_t secSleepRemaining = sleepUntil - now_s;
    if(secSleepRemaining > 0) {
      Serial_print("we need to go back to sleep for: " + String(secSleepRemaining/60) + " min");
      goToDeepSleep(secSleepRemaining / 60);
    }
    
    evaluateIfDeepSleep();
  }

  SerialAT.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

  pinMode(GPRS_ON_PIN, OUTPUT); POWER_GPRS_BOARD_OFF();
  pinMode(GPRS_POWER_PIN, OUTPUT); digitalWrite(GPRS_POWER_PIN, HIGH); // off
  //pinMode(BLINK_LED_PIN, OUTPUT);  digitalWrite(BLINK_LED_PIN, HIGH); // on
  pinMode(SPIN_LED_PIN, OUTPUT);   digitalWrite(SPIN_LED_PIN, LOW);     // off
  pinMode(ERROR_LED_PIN, OUTPUT);  digitalWrite(ERROR_LED_PIN, LOW);    // off
  pinMode(SH40_1_PWR_PIN, OUTPUT);  digitalWrite(SH40_1_PWR_PIN, LOW);
  // pinMode(SH40_2_PWR_PIN, OUTPUT);  digitalWrite(SH40_2_PWR_PIN, LOW);
  pinMode(WAKE_PIN, INPUT);
  pinMode(DONE_PIN, INPUT_PULLDOWN);
  
  // Power ON SH40
  //pinMode(TIMER_PIN, OUTPUT); digitalWrite(TIMER_PIN, LOW);
  //pinMode(VANE_POWER_PIN, OUTPUT); digitalWrite(VANE_POWER_PIN, HIGH);  
  //gpio_set_drive_capability((gpio_num_t) VANE_POWER_PIN, GPIO_DRIVE_CAP_3);


  speedHalSensor.begin();
  buttonInfo.begin(BUTTON_INFO_PIN);
  buttonInfo.setClickHandler(clickInfo);

  buttonSend.begin(BUTTON_SEND_PIN);
  buttonSend.setClickHandler(clickSend);

  windDirSensor.begin();

  checkSensorsConnected();

  if(buttonSend.isPressedDown()) {
    // set the north direction if the button send was pressed down on boot
    calibrateWindDirectionNorth();

    buttonSend.ignoreNextPress(); // we dont want to execute send when we let go of the button
  } else {

    windDirSensor.loadNorthOffset();
    Serial_print("Loaded north offset: "); Serial_println(windDirSensor.getNorthOffset());
  }

  sht1.init();
  //sht2.init();

  setCpuFrequencyMhz(80);

  welcomTurnOnBlink();

  const esp_timer_create_args_t hal_timer_args = {
    .callback = &onReadHal,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "hal_timer"
  };
  esp_timer_handle_t hal_timer;
  if(prefs.enable_wind_speed_read > 0 && prefs.store_wind_data_interval_s > 0) {
    esp_timer_create(&hal_timer_args, &hal_timer);
    esp_timer_start_periodic(hal_timer, 4*1000);  // every 4 ms
  }

  esp_timer_handle_t blinkLed_timer;
  const esp_timer_create_args_t blinkLed_args = {
    .callback = &onBlinkLed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "blinkLed_timer"
  };
  if(prefs.blink_led_on_time_ms > 0) {
    esp_timer_create(&blinkLed_args, &blinkLed_timer);
    esp_timer_start_periodic(blinkLed_timer, prefs.blink_led_on_time_ms * 1000); 
  }

  esp_timer_handle_t spinLed_timer;
  const esp_timer_create_args_t spinLed_args = {
    .callback = &onSpinLed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "spinLed_timer"
  };
  if(prefs.spin_led_on_time_ms > 0) {
    esp_timer_create(&spinLed_args, &spinLed_timer);
    esp_timer_start_periodic(spinLed_timer, prefs.spin_led_on_time_ms * 1000); 
  }

  esp_timer_handle_t errorLed_timer;
  const esp_timer_create_args_t errorLed_args = {
    .callback = &onErrorLed,               
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "errorLed_timer"
  };
  if(prefs.error_led_on_time_ms > 0) {
    esp_timer_create(&errorLed_args, &errorLed_timer);
    esp_timer_start_periodic(errorLed_timer, prefs.error_led_on_time_ms * 1000); 
  }
  

  esp_timer_handle_t readSpeed_timer;
  const esp_timer_create_args_t readSpeed_args = {
    .callback = &onReadSpeed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readSpeed_timer"
  };
  if(prefs.enable_wind_speed_read > 0 && prefs.store_wind_data_interval_s > 0) {
    esp_timer_create(&readSpeed_args, &readSpeed_timer);
    esp_timer_start_periodic(readSpeed_timer, 1000 * 1000); // 1s
  }

  esp_timer_handle_t readDirection_timer;
  const esp_timer_create_args_t readDirection_args = {
    .callback = &onReadDirection,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readDirection_timer"
  };
  if(prefs.store_wind_data_interval_s > 0 && prefs.wind_dir_read_interval_s > 0) {
    esp_timer_create(&readDirection_args, &readDirection_timer);
    esp_timer_start_periodic(readDirection_timer, prefs.wind_dir_read_interval_s*1000*1000ULL);
  }
  

  esp_timer_handle_t storeWindData_timer;
  const esp_timer_create_args_t storeWindData_args = {
      .callback = &onStoreWindData,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "storeWindData_timer"
  };
  if(prefs.store_wind_data_interval_s > 0) {
    esp_timer_create(&storeWindData_args, &storeWindData_timer);
    esp_timer_start_periodic(storeWindData_timer, prefs.store_wind_data_interval_s*1000*1000ULL); // X seconds (in microseconds)
  }

  esp_timer_handle_t watchdogReset_timer;
  const esp_timer_create_args_t watchdogReset_args = {
      .callback = &onResetWatchdogTimer,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "watchdogReset_timer"
  };
  esp_timer_create(&watchdogReset_args, &watchdogReset_timer);
  esp_timer_start_periodic(watchdogReset_timer, 30 * 1000 * 1000ULL);

  Serial_println("Done init!");
  digitalWrite(BLINK_LED_PIN, LOW);

  reset_watchdog_timer();
}

bool readTempHum(SH40SoftI2C &tempHumSensor, float &t, float &h) {
  tempHumSensor.setPower(true);

  if (!tempHumSensor.checkIsConnected()) {
    Serial_print("ERROR: Temp sensor"); Serial_print(tempHumSensor.id); Serial_println(": is not connected!");
    return false;
  }

  delay(10);
  int readStatus = 0;
  int retries = 10;
  for(int i=0; i<retries; i++) {
    readStatus = tempHumSensor.read(t, h);
    if(readStatus == 1) break; 
    
    Serial_print("Temp read sensor "); Serial_print(tempHumSensor.id); 
    Serial_print(", status: "); Serial_println(tempHumSensor.getLastReadStatusDescription());
    delay(30);
  }

  tempHumSensor.setPower(false);
  
  return readStatus == 1;
}

uint32_t get_log_timestamp(int hour, int minute, int second) {
  //return hour*60*30 + minute*30 + second/2;  // round on every 2 seconds so that we can store it inside 16 bit int. Max:43200 < 2**16
  return hour*60*60 + minute*60 + second; // timestamp in seconds
}

uint32_t get_log_timestamp() {
  return get_log_timestamp(current_hour(), current_minute(), current_second());
}

//#define PRINT_MAGNET_READ_DEBUG

#ifdef PRINT_MAGNET_READ_DEBUG
  #define DBG_MNG(...) do { __VA_ARGS__; } while (0)
#else
  #define DBG_MNG(...) do {} while (0)
#endif


void IRAM_ATTR onBlinkLed(void* arg) {
  static int nOnBlinkLedcalls = 0;
  nOnBlinkLedcalls ++;

  if(updateSerialEnabled) {
    digitalWrite(BLINK_LED_PIN, HIGH);
    return;
  }

  // prevents devision by 0 when changing the preferences 
  if(prefs.blink_led_on_time_ms == 0) digitalWrite(BLINK_LED_PIN, LOW);

  // if the device is not light sleeping blink faster
  #define NO_SLEEP_BLINK_INTERVAL 500  // ms 
  uint16_t blinkInterval = !isLightSleep() ? NO_SLEEP_BLINK_INTERVAL : prefs.blink_led_interval_ds*100;
  uint16_t toggleCount = blinkInterval / prefs.blink_led_on_time_ms;

  if(nOnBlinkLedcalls % toggleCount == 0) {
    digitalWrite(BLINK_LED_PIN, HIGH);    
  } else {
    digitalWrite(BLINK_LED_PIN, LOW); 
  }
}

volatile int rotation_detected_blink = 0;
void IRAM_ATTR onSpinLed(void* arg) {
  if(rotation_detected_blink == 1) {
    rotation_detected_blink = 0;
    digitalWrite(SPIN_LED_PIN, HIGH);    
  } else {
    digitalWrite(SPIN_LED_PIN, LOW); 
  }
}


volatile int error_notify_led = 0;
volatile bool enableErrorLedBlinking = true;
void IRAM_ATTR onErrorLed(void* arg) {
  if(!enableErrorLedBlinking) return;

  if(error_notify_led == 1) {
    error_notify_led = 0;
    digitalWrite(ERROR_LED_PIN, HIGH);    
  } else {
    digitalWrite(ERROR_LED_PIN, LOW); 
  }
}


bool performFullWindDirRead(int& angle) {
  MagDirSensor3D::ReadStatus readStatus;

  readStatus = windDirSensor.read();
  if(readStatus != MagDirSensor3D::ReadStatus::OK) {
    error_notify_led = 1;
    angle = readStatus; // save the readStatus inside the angle if the reading failed 
    Serial_print("ERROR: Sensor read failed! readStatus"); Serial_println(readStatus);
    return false;
  }

  angle = windDirSensor.getDirection();
  return true;
}


#define DIRECTIONS_LOG_LEN 10  // buffer to store diractions before the are averaged and saved in an array to be send 
int directions_log[DIRECTIONS_LOG_LEN];
volatile int directions_log_i = 0;
int last_direction_read = -1; // used to display on auto refresh stream

void IRAM_ATTR onReadDirection(void* arg) {
  int angle = -1;

  if(!performFullWindDirRead(angle)) {
    //Serial_println("ERROR: Error reading wind direction");
    elog.logTmp(ErrorLogger::ERR_DIR_READ);
  }

  if(millis() < 1000*60) { // only log for first 60 s
    //Serial_print("Read wind direction: "); Serial_println(angle);
  }

  portENTER_CRITICAL(&timerMux);
  if(directions_log_i < DIRECTIONS_LOG_LEN) {
    // save the measurement inside the log
    directions_log[directions_log_i++] = angle;
    last_direction_read = angle;
  } else {
    elog.logTmp(ErrorLogger::ERR_DIR_SHORT_BUF_FULL);
  }
  portEXIT_CRITICAL(&timerMux);

  
}


//#define PRINT_SPEED
//#define PRINT_ON_STORE_WIND

volatile float rps = 0;
volatile uint32_t rotationCount = 0;
volatile int lastHalSensorRead = -1;
volatile uint32_t lastDetection = 0; // we need to store the time of last detection to calculate rotations per second
void IRAM_ATTR onReadHal(void* arg) {
  int halSensorRead = speedHalSensor.read();
  uint32_t now = micros() / 1000;

  if (halSensorRead != lastHalSensorRead && halSensorRead == LOW) {
    portENTER_CRITICAL(&timerMux);
    rotationCount ++;                    // number of rotations counted, used to average rps every second in a diferent interupt
    rotation_detected_blink = 1;

    if(lastDetection != 0 && now > lastDetection) {
      rps += 1000.0f / (now - lastDetection);
    }
    portEXIT_CRITICAL(&timerMux);
    #ifdef PRINT_SPEED
      Serial_print("|");
    #endif
    lastDetection = now;
  }
  lastHalSensorRead = halSensorRead;

}


#define SPEEDS_LOG_LEN 60
uint16_t speeds_log[SPEEDS_LOG_LEN]; // speed logged each second for a short interval
volatile int speeds_log_i = 0;
int16_t lastSpeedRead = -1;

// read speed every second
void IRAM_ATTR onReadSpeed(void* arg) {
  portENTER_CRITICAL(&timerMux);

  float speed = rotationCount == 0 ? 0 : rps / rotationCount;
  rotationCount = 0;
  rps = 0;

  if (speeds_log_i < SPEEDS_LOG_LEN) {
    speeds_log[speeds_log_i++] = int(speed * 10);
    lastSpeedRead = int(speed * 10);
    portEXIT_CRITICAL(&timerMux);
  } else {
    portEXIT_CRITICAL(&timerMux);
    elog.logTmp(ErrorLogger::ERR_WIND_SHORT_BUF_FULL);
  }

  

  #ifdef PRINT_SPEED
    Serial_print("Speed: "); Serial_print(speed * 10);
    Serial_print(", cnt:"); Serial_println(rotationCount);
  #endif
}



//#define STORE_WIND_N_HOURS 2
// calculate how much store lenght do we need to save N hours of data  if we store every interval second
//#define WIND_LOG_STORE_LEN ((STORE_WIND_N_HOURS* 60*60) / STORE_WIND_DATA_INTERVAL) 
#define WIND_LOG_STORE_LEN 1000 // no reason to store more then 1000 data as it cant properly send all of it 

// ---- Data record ------------------------------------------------------------
uint32_t wind_data_start_time = 0;

// ---- Storage (ring buffer) --------------------------------------------------
static WindSample wind_log[WIND_LOG_STORE_LEN];

// Head points to next write position; count is number of valid items (<= LEN)
static volatile uint16_t w_head = 0;
static volatile uint16_t w_count = 0;
static volatile uint32_t first_timestamp = 0;
static volatile uint32_t last_timestamp = 0;

// ---- Helpers ----------------------------------------------------------------
uint16_t windlog_len(void) {
    return (uint16_t) w_count;
}

// Oldest item index in the circular buffer (physical index within wind_log[])
uint16_t windlog_oldest_index(void) {
    if (w_count == 0) return 0;
    // (head - count) modulo LEN
    uint16_t oldest = (uint16_t)((w_head + prefs.wind_log_store_len - w_count) % prefs.wind_log_store_len);
    return oldest;
}

// ---- Public API ------------------------------------------------------------

// Store one record (overwrites the oldest when full)
void windlog_push(uint16_t avg, int16_t dir, uint32_t ts) {
    portENTER_CRITICAL(&timerMux);
    wind_log[w_head].avg  = avg;
    //wind_log[w_head].max  = max;
    wind_log[w_head].dir  = dir;
    //wind_log[w_head].ts = ts;

    if(w_count == 0) first_timestamp = ts;
    last_timestamp = ts;
    
    w_head = (uint16_t)((w_head + 1) % prefs.wind_log_store_len);
    if (w_count < prefs.wind_log_store_len) {
        w_count++;
        portEXIT_CRITICAL(&timerMux);
    } else{
      first_timestamp += prefs.store_wind_data_interval_s; // just increase the fist timestamp by the expected interval that timestamps increase (by interval )
      portEXIT_CRITICAL(&timerMux);
      // buffer full -> oldest is implicitly dropped
      elog.logTmp(ErrorLogger::ERR_WIND_BUF_OVERWRITE);
    }
}

int windlog_copy(WindSample* out, uint16_t max_out) {
  if (!out || max_out == 0) return 0;

  uint16_t copied = 0;
  uint16_t size   = w_count;
  uint16_t to_copy = (size < max_out) ? size : max_out;

  if (to_copy > 0) {
      uint16_t base = windlog_oldest_index();

      // The logical span may wrap; split into two memcpy if needed
      uint16_t first_run = (uint16_t)((base + to_copy <= prefs.wind_log_store_len) ? to_copy : (prefs.wind_log_store_len - base));
      memcpy(out, &wind_log[base], first_run * sizeof(WindSample));

      uint16_t remaining = (uint16_t)(to_copy - first_run);
      if (remaining) {
          memcpy(out + first_run, &wind_log[0], remaining * sizeof(WindSample));
      }
      copied = to_copy;
  }

  return copied;
}

// Optional: clear buffer
inline void windlog_clear(void) {
    w_head = 0;
    w_count = 0;
}

int16_t average_direction(const int* directions_log, size_t len) {
  float sum_sin = 0.0;
  float sum_cos = 0.0;

  int nValidValues = 0;
  for (size_t i = 0; i < len; i++) {
    if(directions_log[i] < 0) continue; // ignore negaitve values which are not real angles but are just error codes 
    
    nValidValues++;
    float radians = directions_log[i] * DEG_TO_RAD;
    sum_cos += cos(radians);
    sum_sin += sin(radians);
  }

  if(nValidValues == 0) {
    // no valid values so we just log the first error code as the angle 
    return directions_log[0];
  }

  float avg_angle = atan2(sum_sin, sum_cos) * RAD_TO_DEG;
  if (avg_angle < 0) avg_angle += 360.0;  // Normalize to [0, 360)
  return (int16_t) avg_angle;
}


uint16_t speeds_copy[SPEEDS_LOG_LEN];
int directions_copy[DIRECTIONS_LOG_LEN];

void IRAM_ATTR onStoreWindData(void* arg) {
  uint32_t timestamp = get_log_timestamp(); // we need to get the timestamp before critical section!

  int speeds_count;
  int directions_count;
  bool set_start_time = false;

  portENTER_CRITICAL(&timerMux);
  // store the miliseconds of when the first data got stored 
  if(wind_data_start_time == 0) wind_data_start_time = lastNow;

  // copy the data in to temporery buffer to work with it 
  // not save to work with the data in the critical section due to using logError and sin functions
  speeds_count = speeds_log_i;
  if (speeds_count > SPEEDS_LOG_LEN) speeds_count = SPEEDS_LOG_LEN;
  memcpy(speeds_copy, speeds_log, speeds_count * sizeof(speeds_copy[0]));
  speeds_log_i = 0;

  directions_count = directions_log_i;
  if (directions_count > DIRECTIONS_LOG_LEN) directions_count = DIRECTIONS_LOG_LEN;
  memcpy(directions_copy, directions_log, directions_count * sizeof(directions_copy[0]));
  directions_log_i = 0;

  portEXIT_CRITICAL(&timerMux);

  // the log is full so we calculate average and save the measurement into the avg log
  //uint16_t maxSpeed = 0;
  int avgSpeedSum = 0;
  for(int i=0; i<speeds_count; i++) {
    //maxSpeed = max(maxSpeed, speeds_log[i]);
    avgSpeedSum += speeds_copy[i]; 
  }

  // if there is no speeds logged store -1
  int avgSpeed = speeds_count == 0 ? -1 : avgSpeedSum / speeds_count;

  // if there is no directions_log set the value to -1 so that we know that no value was read
  int16_t avgDir = directions_count == 0 ? -1 : average_direction(directions_log, directions_count);

  //windlog_push(avgSpeed, maxSpeed, avgDir, timestamp);
  windlog_push(avgSpeed, avgDir, timestamp);
  
  #ifdef PRINT_ON_STORE_WIND 
    Serial_print("Updated wind data, avg:"); Serial_print(avgSpeed);
    Serial_print(", dir:"); Serial_print(avgDir);
    Serial_println();
  #endif
}

WindSample wind_log_copy[WIND_LOG_STORE_LEN];
String getWindData() {
  portENTER_CRITICAL(&timerMux);
  uint16_t wind_log_copy_len = windlog_copy(wind_log_copy, WIND_LOG_STORE_LEN);
  portEXIT_CRITICAL(&timerMux);

  //int speed_avg_i_on_send = 0;
  //speed_avg_i_on_send = speed_avg_i; // we save the index on when we send the data so we can see if there is any new data when we restart the index after successful send 

  if(wind_log_copy_len == 0) {
    return "len=0;avg=;dir=;logFirst=;logLast=;"; 
  }

  String windData = "len=" + String(wind_log_copy_len);

  windData += ";avg=" + String(wind_log_copy[0].avg);
  for(int i=1; i<wind_log_copy_len; i++) {
    windData += ",";
    windData += String(wind_log_copy[i].avg);
  }

  windData += ";dir=" + String(wind_log_copy[0].dir);
  for(int i=1; i<wind_log_copy_len; i++) {
    windData += ",";
    windData += String(wind_log_copy[i].dir);
  }

  windData += ";logFirst=" + String(first_timestamp);
  windData += ";logLast=" + String(last_timestamp);
  return windData + ";"; 
}


bool isSleepHour(int start, int end, int hour) {
  if (start == 0 && end == 0 && prefs.sleep_enabled == 2) return true; // special case where we force the device to sleep all the time to basically never wake up
  if (start > end) 
    return start <= hour || hour < end;
  else
    return end > hour && hour >= start;
}

bool isDeepSleepTime() {
  if(prefs.sleep_enabled == 0) return false; // dont do anything if it is disabled 
  if(prefs.sleep_enabled == 2 && hasSendAfterTurnOn == false) return false; // we dont go to sleep if we dont try sending first // if sleep mode 2 (sends once after sleeping) and if there is no send after turn on meaning it hasnt tried sending yet dont go to deep sleep until the send it at least once 
  return isSleepTime();
}


bool isSleepTime() {
  if(!accurateTimeSet) return false; // the time was not set from the GMS module yet

  //Serial_print("sleep?"); Serial_println(isSleepHour(prefs.sleep_hour_start, prefs.sleep_hour_end, hour()));
  // we sleep at night ofcorse! from 8 PM to 6 AM
  return isSleepHour(prefs.sleep_hour_start, prefs.sleep_hour_end, current_hour());
}

void goToDeepSleep(uint64_t duration_min) {
    Serial_print("Current time is:"); Serial_println(getFormattedTimeLibString());
    Serial_print("It is time to go deep sleep for: "); Serial_print(duration_min);  Serial_println(" minutes!");
    delay(500); // delay for all the Serial_prints to finish 

    sleepUntil = (time_t)(now_rtc_s() + (int64_t)duration_min * 60);
    Serial_print("Scheduled wake up time is:"); Serial_println(getFormattedUnixTime(sleepUntil));

    esp_sleep_enable_timer_wakeup(duration_min * 60 * 1000000);

    pinMode(WAKE_PIN, INPUT);
    rtc_gpio_deinit((gpio_num_t)WAKE_PIN);
    rtc_gpio_pullup_dis((gpio_num_t)WAKE_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)WAKE_PIN);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_PIN, 1);  

    delay(100); // wait for all the logs to print
    esp_deep_sleep_start();  // after this, it won't return here — will restart from setup()
}

int calcMinUntilWake() {
  int currentTimeInMin = current_hour() * 60 + current_minute();
  int sleepEndInMin = prefs.sleep_hour_end * 60;
  int minUntilWake = (sleepEndInMin - currentTimeInMin + 24 * 60) % (24 * 60);
  return minUntilWake;
}

void evaluateIfDeepSleep() {
  if (!isDeepSleepTime()) return;

  int minUntilWake = calcMinUntilWake();
  if (minUntilWake == 0) return;           // already at wake time (HH:00)

  int sleepDurationMin = min((int) prefs.sleep_dur_min, minUntilWake);

  if (sleepDurationMin <= 0) return;       // safety
  goToDeepSleep(sleepDurationMin);
}

bool isTimeToSendData(uint32_t secSinceLastSend) {
  if(prefs.sleep_enabled == 2 && isSleepTime()) {
      // if the sleep mode 2 is on and we are currently in the time when we should be sleeping then we check the 
      // sleep2 send interval instead of the usal send_data interval :) 
      return secSinceLastSend > prefs.sleep_2_send_interval_s;
  } else {
    // normal case when the devices is on and sending data 
    return secSinceLastSend > prefs.send_data_interval_min*60;
  }
}

bool isLightSleep() {
  if(prefs.light_sleep_enabled == 1) return true;  // default state, sleep enabled
  if(prefs.light_sleep_enabled == 0) return false; // sleep disabled 
  
  if(prefs.light_sleep_enabled == 2) {
    // special debug case where the sleep is disabled in frist 2 seconds 
    bool enoughTimePassed = millis() > 1*60*1000; 
    return enoughTimePassed;
  }

  return true; // undefined state presumes light sleep is enabled
}

FloatRunningAverage<32> vBattAvg;
FloatRunningAverage<8> vSolarAvg;

bool isVoltageHighEnoughToPerformSend() {
  float vBatt = vBattAvg.get();
  return vBatt > 3.4;  
}


uint32_t lastSend = 0;
uint32_t lastSuccessfulSend = 0;
void loop() {
  RUN_ONCE({ Serial_println("Inisde the main loop!"); });

  buttonInfo.loop();
  buttonSend.loop();

  evaluateIfDeepSleep();

  static uint32_t lastPrint = 0;
  static uint32_t lastVBattIdeLog = 0;

  uint32_t secSinceLastSend = (millis() - lastSend)/1000;
  if(isTimeToSendData(secSinceLastSend)) {
    lastSend = millis();
    Serial_println(String(secSinceLastSend) + " s passed doing send");

    if(isVoltageHighEnoughToPerformSend()) {
      fullCycleSend();
    } else {
      Serial_println("ERROR: Battery voltage too low to send");
      elog.log(ErrorLogger::ERR_SEND_V_BATT_TOO_LOW);
    }
  }

  // read battery voltage every 5s
  if(millis() - lastVBattIdeLog > 5*1000) {
    if(lastVBattIdeLog == 0) {
      // skip the first read since it is for some reason too high :shrug:
      read_batt_v(); read_solar_v();
    } else {
      vBattAvg.addSample(read_batt_v());
      vSolarAvg.addSample(read_solar_v());
    }

    lastVBattIdeLog = millis();
  }

  // check if the station was unable to send the data for longer then 1 hour 
  #define NO_SEND_FORCE_RESET_TIME  1*60*60*1000
  if(millis() - lastSuccessfulSend > NO_SEND_FORCE_RESET_TIME) { 
    Serial1.println("ERROR: Unable to send the data for longer then 1 hour, force resetting.");
    elog.log(ErrorLogger::ERR_CANT_SEND_FORCE_RST); 
    esp_restart();  // software reset
  }


  lastNow = millis();

  //updateSerial();
  if(updateSerialEnabled) {
    updateSerial();
  }

  if(isLightSleep()){
    esp_sleep_enable_timer_wakeup(5*1000); esp_light_sleep_start();
    RUN_ONCE({ Serial_println("Entering light sleep  cylcle from now on."); });
  } else {
    delay(2);
    RUN_ONCE({ Serial_println("Sleep not enabled for now."); });
  }
}

float read_batt_v() {
  return analogRead(V_BATT_PIN) *  prefs.vbat_calib; 
}

float read_solar_v() {
  return analogRead(V_SOALR_PIN) * prefs.vsolar_calib; 
}

void turnOnModule() {
  Serial_println("Giving power to GPRS module");

  for(int i=0; i<50;i++){
    POWER_GPRS_BOARD_ON(); 
    POWER_GPRS_BOARD_OFF();
    delayMicroseconds(100);
  }

  for(int i=0; i<200;i++){
    POWER_GPRS_BOARD_ON();
    delayMicroseconds(10*((i)/10)); 
    POWER_GPRS_BOARD_OFF();
    delay(1);
  }
  POWER_GPRS_BOARD_ON();

  delay(1000);

  Serial_println("Turning on GPRS module!");
  digitalWrite(GPRS_POWER_PIN, LOW);  // the power pin has to be pulle to low for 1 second in order to turn
  Serial_println("waiting 1s ...");
  delay(1000);
  digitalWrite(GPRS_POWER_PIN, HIGH);
  Serial_println("Done");
}

void turnOffModule() {
  POWER_GPRS_BOARD_OFF();

  Serial_println("gprs high -> turning off");
}



bool isOn = false;
void clickInfo() {
  printPreferences();
  printDiagnosticInfo();
}

void clickSend() {
  if(buttonInfo.isPressedDown()) {
    updateSerialEnabled = true;
    digitalWrite(BLINK_LED_PIN, HIGH);
    Serial_println("updateSerial mode enabled");
    return;
  }

  fullCycleSend();
}

void readTempSensors() {
  temp_in = NAN;  
  hum_in = NAN;
  temp_out = NAN; 
  hum_out = NAN;
  bool tempReadSuccess = false;

  if ((prefs.read_temp_enabled & 0x01) > 0) {
    tempReadSuccess = readTempHum(sht1, temp_out, hum_out);
    if (!tempReadSuccess) {
      elog.log(ErrorLogger::ERR_TEMP_READ_OUT);
    } else {
      Serial_print("temp_out:"); Serial_println(String(temp_out, 1));
      Serial_print("humy_out:"); Serial_println(String(hum_out, 1));
    }
  } else {
    Serial_print("Temperature OUTside reading disabled by prefs.read_temp_enabled:");
    Serial_println(prefs.read_temp_enabled);
  }

  if ((prefs.read_temp_enabled & 0x02) > 0) {
    tempReadSuccess = readTempHum(sht2, temp_in, hum_in);
    if (!tempReadSuccess) {
      elog.log(ErrorLogger::ERR_TEMP_READ_IN);
    } else {
      Serial_print("temp_in:"); Serial_println(String(temp_in, 1));
      Serial_print("humy_in:"); Serial_println(String(hum_in, 1));
    }
  } else {
    Serial_print("Temperature INside reading disabled by prefs.read_temp_enabled:");
    Serial_println(prefs.read_temp_enabled);
  }

}


void fullCycleSend() {
  const int nSendRetrys = prefs.n_send_retries;
  bool sendOk = false;

  // before sending read temeprature sensors
  readTempSensors();
  
  hasSendAfterTurnOn = true; // we tried sending!
  for(int nTry=0; nTry<nSendRetrys && !sendOk; nTry++) {
    Serial_print("Sending try n:"); Serial_println(nTry);
    
    turnOnModule();
      
    httpGetStart = millis();
    if (sendDataToServer(nTry)) {
      Serial_println("Send successful!");
      sendOk = true;
      lastSuccessfulSend = millis(); 
    } else {
      elog.log(ErrorLogger::ERR_SEND_REPEAT);
      Serial_print("ERROR: Send failed");
      error_notify_led = 1;
    }

    Serial_println("Finished sending!");
    Serial_print("Duration:"); Serial_print((millis()-httpGetStart)/1000); Serial_println("s");

    turnOffModule();
    delay(1000);
  } 
}

void printDiagnosticInfo() {
  Serial_print("\n");
  Serial_print("WIND_LOG_STORE_LEN: "); Serial_println(WIND_LOG_STORE_LEN);
  Serial_print("w_count: "); Serial_println(w_count);
  Serial_print("Time: "); Serial_println(getFormattedTimeLibString());
  Serial_print("Last send:   "); Serial_print((millis() - lastSend)/(1000*60)); Serial_println(" min ago"); 
  
  Serial_println("\r\nAnalog measurements:");
  Serial_print("v batt avg:  "); Serial_println(String(vBattAvg.get(), 3));
  Serial_print("v solar avg: "); Serial_println(String(vSolarAvg.get(), 3));
  Serial_print("v batt:      "); Serial_println(String(read_batt_v(), 3)); 
  Serial_print("v solar:     "); Serial_println(String(read_solar_v(), 3));
  //Serial_print("v batt raw:  "); Serial_println(analogRead(V_BATT_PIN)); 
  //Serial_print("v solar raw: "); Serial_println(analogRead(V_SOALR_PIN));

  bool success;
  /*
  temp_in = NAN; hum_in = NAN;
  success = readTempHum(sht1, temp_in, hum_in);
  if (!success) {
    Serial_println("ERROR: temp1 failed to read"); 
  } else {
    Serial_print("temp_in:"); Serial_println(String(temp_in, 1));
    Serial_print("humy_in:"); Serial_println(String(hum_in, 1));
  }
  */
  readTempSensors();

  int angle = -1;
  performFullWindDirRead(angle);
  Serial_print(F("Wind dir angle: ")); Serial_println(angle);
  
  String postBody = getPostBody();
  Serial_print("Post body (len=");
  Serial_print(postBody.length());
  Serial_print("): ");
  Serial_println(postBody);

  Serial_println("All errors:"); Serial_println(elog.getAll());

  Serial_println();
}



String inputBuffer = "";
bool handlePrefSerialCommand(String& command) {
  if(command == "p" || command == "P") {
    printPreferences();
    return true;
  }

  if(command == "ps" || command == "PS") {
    savePreferences();
    return true;
  }

  if(!(command.startsWith("p:") || command.startsWith("P:"))) {
    return false;
  }

  int valueSep = command.indexOf(':', 2);
  if(valueSep < 0) {
    Serial_println("ERROR: Expected p:<prefs_name>:<value>");
    return true;
  }

  String key = command.substring(2, valueSep);
  String value = command.substring(valueSep + 1);

  if(key.length() == 0) {
    Serial_println("ERROR: Preference name is empty");
    return true;
  }

  if(!saveNewPrefValue(key, value)) {
    Serial_println("ERROR: Preference was not changed");
    return true;
  }

  Serial_print("Preference changed in RAM: ");
  Serial_print(key);
  Serial_print("=");
  Serial_println(value);
  return true;
}

bool updateSerial() {
  delay(30);
  
  while (SerialDBG.available()) {
    char c = SerialDBG.read();
    SerialDBG.write(c);
    //Serial_print("read:'");
    //Serial_print(c);
    //Serial_println("'");

    if (c == '\n' || c == '\r') {
      Serial_print("Command:'");
      Serial_print(inputBuffer);
      Serial_print("'\r\n");

      handlePrefSerialCommand(inputBuffer);
      //if (inputBuffer[0] == 'o' || inputBuffer[0] == 'O') {
      //  turnOnModule();
      //} else if (inputBuffer[0] == 'x' || inputBuffer[0] == 'X') {
      //  turnOffModule();
      //} else if (inputBuffer[0] == 's' || inputBuffer[0] == 'S') {
      //  fullCycleSend();
      //} else if (inputBuffer[0] == 'i' || inputBuffer[0] == 'I') {
      //  printDiagnosticInfo();
      //} else if (inputBuffer[0] == 'n') {
      //  setPhoneNumber(inputBuffer);
      //} else if (handlePrefSerialCommand(inputBuffer)) {
        // Handled locally.
      //} else {
      //  SerialAT.println(inputBuffer.c_str());
      //  //SerialAT.write(inputBuffer.c_str());       //Forward what Serial received to Software Serial Port
      //}
      
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  String atResponse = "";
  while (SerialAT.available()) {
    //Forward what Software Serial received to Serial Port
    char atRead = SerialAT.read();
    atResponse += atRead;
    delay(1); // we need to wait since SerialAt seems to send quite slowly 
  }
  if(atResponse.length() > 0) {
    SerialDBG.print("AT:");
    SerialDBG.print(atResponse);
    SerialDBG.println();
  }

  return true;
}







void windlog_shift_timestamps(int32_t delta)
{
    if (delta == 0 || w_count == 0) return;

    portENTER_CRITICAL(&timerMux);

    first_timestamp += delta;
    last_timestamp += delta;

    /*
    // compute index of oldest record
    uint16_t base = (w_head + WIND_LOG_STORE_LEN - w_count) % WIND_LOG_STORE_LEN;

    // iterate through all valid entries in logical order
    for (uint16_t i = 0; i < w_count; i++) {
        uint16_t idx = (base + i) % WIND_LOG_STORE_LEN;
        wind_log[idx].ts +=  delta;
    }
    */

    portEXIT_CRITICAL(&timerMux);
}


Preferences prefsStorage;
static const char *PREF_NAMESPACE = "time";
static const uint32_t ESTIMATED_RESET_SECONDS = 2; 

void saveTimeAndScheduleReset() {
    time_t current = (time_t)now_rtc_s();  // current UNIX time

    prefsStorage.begin(PREF_NAMESPACE, false);
    prefsStorage.putULong64("saved_time", (uint64_t)current);
    prefsStorage.putBool("sched_reset", true);
    prefsStorage.end();

    Serial_println("Scheduled reset, saving time and restarting...");
    delay(2000); // wait so that everything can print before reseting 

    esp_restart();  // software reset
}

void restoreTimeIfScheduledReset() {
    prefsStorage.begin(PREF_NAMESPACE, false);
    bool scheduled = prefsStorage.getBool("sched_reset", false);
    uint64_t saved = prefsStorage.getULong64("saved_time", 0);
    prefsStorage.putBool("sched_reset", false);   // clear flag so it’s one-shot
    prefsStorage.end();

    if (scheduled && saved > 0) {
        time_t restored = (time_t)saved + ESTIMATED_RESET_SECONDS;

        set_system_time_unix(restored); // Restore system time
        accurateTimeSet = true;
        Serial_print("Restored time after scheduled reset: ");  Serial_println(restored);
        Serial_print("Restored datetime: "); Serial_println(getFormattedTimeLibString());
    } else {
        Serial_println("No scheduled reset time to restore.");
    }
}


float getVBattAvg() {
  float battAvg = vBattAvg.get();
  return isnan(battAvg) ?  read_batt_v() : battAvg; 
}

float getVSolarAvg() {
  float solarAvg = vSolarAvg.get();
  return isnan(solarAvg) ? read_solar_v() : solarAvg;
}

String getPostBody() {
  String body = "";
  body.reserve(512);   // avoid fragmentation, improve speed
  body += "pref=" + String(prefs.pref_version) + ";";
  //body += "prefDate=" + getFormattedUnixTime(prefs.pref_set_date) + ";";
  body += "ver=" + String(prefs.version) + ";";
  //body += "imsi=" + imsiNum + ";";
  //body += "phoneNum=" + phoneNum + ";";

  if (!isnan(temp_in))  body += "temp_in=" + String(temp_in, 1) + ";";
  if (!isnan(hum_in))   body += "hum_in=" + String(hum_in, 0) + ";";
  if (!isnan(temp_out)) body += "temp_out=" + String(temp_out, 1) + ";";
  if (!isnan(hum_out))  body += "hum_out=" + String(hum_out, 0) + ";";
  body += "vbatIde=" + String(getVBattAvg(), 3) + ";";
  body += "vbatGprs=" + String(read_batt_v(), 3) + ";";
  body += "vsol=" + String(getVSolarAvg(), 3) + ";"; 
  body += "dur=" + String((millis() - httpGetStart) / 1000.0, 1) + ";";
  body += "signal=" + String(signalStrength) + ";";

  if (simDuration     > 3*1000) body += "simDur=" + String(simDuration / 1000.0, 1) + ";";
  if (regDuration     > 4*1000) body += "regDur=" + String(regDuration / 1000.0, 1) + ";";
  if (gprsRegDuration > 5*1000) body += "gprsRegDur=" + String(gprsRegDuration / 1000.0, 1) + ";";
  //body += "err_ver=" + String(ErrorLogger::ERROR_CODE_VERSION) + ";";
  if (elog.getNErrorsLogged() > 0) body += "errors=" + elog.getAllForSend() + ";";
  body += getWindData();

  return body;
}


bool sendDataToServer(int nTry) {
  signalStrength = -1;
  simDuration = -1;
  regDuration = -1;
  gprsRegDuration = -1;

  Serial_println("\n\nSending data to the server...");

  if(!establishConnection()) return false;

  if(prefs.send_over_tcp == 1) {
    if(!startTcpConnection()) {
      elog.log(ErrorLogger::ERR_TCP_CONN_START);
      return false;
    } 
    
    if(!sendOverTCP()) return false;
  } else {
    if(sendOverHttp() != SendResult::OK) return false;
  }

  elog.clearAll(); // clear all the errors so they are not send again
  windlog_clear();

  if(shouldReset) {
    Serial_println("Reseting the module to apply the settings soon.\n\n");
    saveTimeAndScheduleReset();
  } {
    Serial_println("No reset requested");
  }

  /*
  portENTER_CRITICAL(&timerMux);
  Serial_print("We need to move: ");
  Serial_print(speed_avg_i - speed_avg_i_on_send);
  Serial_print(" data ...");
  // move tail [speed_avg_i_on_send .. speed_avg_i-1] to front starting at 0
  int dst = 0;
  for (int src = speed_avg_i_on_send; src < speed_avg_i; ++src, ++dst) {
    speeds_avg[dst]  = speeds_avg[src];
    speeds_max[dst]  = speeds_max[src];
    speeds_time[dst] = speeds_time[src];
  }
  speed_avg_i = dst;
  speed_avg_i_on_send = 0;
  speeds_avg_time_start = millis();
  directions_avg_i = 0;
  portEXIT_CRITICAL(&timerMux);
    */

  return true;
}


















