// PrefBlob.h
#pragma once
#include <Arduino.h>
#include <Preferences.h>


template<typename T>
class PrefBlob {
  static_assert(std::is_trivially_copyable<T>::value,
                "T must be trivially copyable (POD-like).");

public:
  // ctor: optional compile_time tag you bump when we compile a new version
  explicit PrefBlob(uint32_t compile_time = 0) : _compile_time(compile_time) {}

  // Open NVS namespace (rw by default). Note: NVS key length <= 15 chars.
  bool begin() {
    bool readOnly = false;
    _begun = _prefs.begin(_nvsNamespace, readOnly);
    return _begun;
  }

  void end() {
    if (_begun) { _prefs.end(); _begun = false; }
  }

  int load(T& out) {
    if (!_begun) return -1;

    size_t len = _prefs.getBytesLength(_key);
    if (len < sizeof(Header) + sizeof(T)) return -2;

    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[len]);
    if (!buf) return -3;

    size_t got = _prefs.getBytes(_key, buf.get(), len);
    if (got != len) return -4;

    const Header* hdr = reinterpret_cast<const Header*>(buf.get());
    if (hdr->magic != MAGIC) return -5;
    if (hdr->size  != sizeof(T)) return -6;                    // struct size changed
    if (hdr->compile_time != _compile_time) return -7;         // stored newer than firmware

    const uint8_t* payload = buf.get() + sizeof(Header);
    uint32_t crc = crc32(payload, sizeof(T));
    if (crc != hdr->crc32) return -8;

    // Valid -> copy payload into caller-provided out
    memcpy(&out, payload, sizeof(T));
    return 1;
  }

  int save(const T& in) {
    if (!_begun) return -1;

    Packed p;
    p.h.magic   = MAGIC;
    p.h.compile_time = _compile_time;
    p.h.size    = sizeof(T);
    memcpy(&p.payload, &in, sizeof(T));
    p.h.crc32   = crc32(reinterpret_cast<const uint8_t*>(&p.payload), sizeof(T));

    size_t wrote = _prefs.putBytes(_key, &p, sizeof(Packed));
    if (wrote != sizeof(Packed)) return -2;

    return 1;
  }

  // Remove key from NVS
  bool erase() {
    if (!_begun) return false;
    bool ok = _prefs.remove(_key);
    return ok;
  }


private:
  struct Header {
    uint32_t magic;   // 'PBLB'
    uint16_t compile_time; // save when the program was compiled so that we know to bump it up when new version is installed
    uint16_t size;    // sizeof(T)
    uint32_t crc32;   // CR_namespaceC of payload (T bytes)
  };

  static constexpr uint32_t MAGIC = 0x50424C42; // 'PBLB'

  struct Packed {
    Header h;
    T      payload;
  };

  static uint32_t crc32(const uint8_t* data, size_t len) {
    // Small, tableless CRC32 (IEEE 802.3)
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (int b = 0; b < 8; ++b) {
        uint32_t mask = -(crc & 1u);
        crc = (crc >> 1) ^ (0xEDB88320u & mask);
      }
    }
    return ~crc;
  }

  Preferences _prefs;
  const char* _nvsNamespace = "app";
  const char* _key = "prefs_blob";
  uint16_t _compile_time;
  bool _begun = false;

  T  _loadedCopy{};
};


struct AppPrefs { 
  uint16_t pref_version;               // just an intiger to difirentiata between different versions 
  uint32_t pref_set_date;              // the date time when the preferences were set
  uint8_t  load_def_prefs;             // if not 0 it loads the default preferences defined in the code on the next boot
  char     version[8];                 // program/sw version
  char     url_data[128];              // url that is used to send the data to 
  char     url_prefs[128];             // url that is used to send the preferences to if requeste
  char     url_errors[128];            // url that is used to send all the error names 
  char     url_stream[128];            // url that is used to send stream (fast data to the server)

  uint8_t  light_sleep_enabled;        // light sleep between reads, 0 if disabled, and 1 if enabled and 2 if enabled only after 1 min after boot 
  uint8_t  sleep_enabled;              // 0 if disabled, and 1 if enabled, 2 sends data once after each sleep cycle 
  uint16_t sleep_dur_min;              // how long to go to sleep for each deep sleep 
  uint16_t sleep_2_send_interval_s;    // seconds to collect data before sending in sleep mode 2 
  int8_t   sleep_hour_start;           // which hour the device gets to sleep in 24 hour format, if start and stop is set to 100 the device is in sleep mode all the time 
  int8_t   sleep_hour_end;             // which hour the device is expected to wake up again

  uint16_t store_wind_data_interval_s; // seconds between storing wind data (0 disables; also disables hal/read speed events)
  uint8_t  send_data_interval_min;     // minutes between data sends 
  uint8_t  n_send_retries;             // how many times do we retry sending 
  uint16_t wind_log_store_len;         // how many wind data we can store, to be send on the next interaction 


  uint8_t  at_timeout_s;               // timeouts for crutical AT commands
  uint8_t  sim_timeout_s;
  uint8_t  csq_timeout_s;             
  uint8_t  creg_timeout_s;  
  uint8_t  cgreg_timeout_s;  

  uint8_t  error_led_on_time_ms;    // ms error LED stays on when blinking (<=0 disables)
  uint8_t  dir_led_on_time_ms;      // ms direction LED stays on when blinking (<=0 disables)
  uint8_t  spin_led_on_time_ms;     // ms spin LED stays on when blinking (<=0 disables)
  uint8_t  blink_led_on_time_ms;    // ms blink LED stays on when blinking (<=0 disables)
  uint8_t  blink_led_interval_ds;   // deciseconds (0.1 s) between blink cycles

  uint16_t wind_dir_read_interval_s;  // seconds between direction reads (0 disables)
  int8_t   enable_wind_speed_read;  // set to <= 0 to disable it 

  uint8_t  read_temp_enabled;       // 0 disables temp1/temp2 reads during send, 1-enables temp 1, 2-enables temp2, 3-enables both of them

  float  vbat_calib;                // calibration for converting the measured voltage on vbat to the actual voltage
  float  vsolar_calib;              // calibration for converting the measured voltage on vsolar to the actual voltage
};

// define default preferences:
AppPrefs prefs = {
  /*pref_version*/              0,
  /*pref_set_date*/             0, 
  /*load_def_prefs*/            0,      // should be always 0 unless we want to use default preferences every reset
  /*version*/                   "v2.2",

  /*url_data*/                  "http://46.224.24.144/veter/save/",
  /*url_prefs*/                 "http://46.224.24.144/veter/save_prefs/",
  /*url_errors*/                "http://46.224.24.144/veter/save_error/",
  /*url_stream*/                "http://46.224.24.144/veter/stream/",

  /*light_sleep_enabled*/       1, 
  /*sleep_enabled*/             0,
  /*sleep_dur_min*/             30,
  /*sleep_2_send_interval_s*/   20, 
  /*sleep_hour_start*/          18,     // time hours
  /*sleep_hour_end*/            6,      // time hours

  /*store_wind_data_interval_s*/  5,     
  /*send_data_interval_min*/      10,    
  /*n_send_retries*/              3,
  /*wind_log_store_len*/          600,    

  /*at_timeout_s*/              10,     
  /*sim_timeout_s*/             20,     
  /*csq_timeout_s*/             60,    
  /*creg_timeout_s*/            60,    
  /*cgreg_timeout_s*/           60,    

  /*error_led_on_time_ms*/      10,  
  /*dir_led_on_time_ms*/        10,  
  /*spin_led_on_time_ms*/       20,  
  /*blink_led_on_time_ms*/      20,  
  /*blink_led_interval_ds*/     20,  // deciseconds (0.1 s)

  /*wind_dir_read_interval_s*/    3,    
  /*enable_wind_speed_read*/      1,

  /*read_temp_enabled*/         1,

  /*vbat_calib*/                0.0006355, 
  /*vsolar_calib*/              0.0013695,
};

void printPreferences() {
  Serial_println("---- Preferences ----");
  Serial_print("  pref_version:   "); Serial_println(prefs.pref_version);
  Serial_print("  pref_set_date:  "); Serial_println(getFormattedUnixTime(prefs.pref_set_date));
  Serial_print("  load_def_prefs: "); Serial_println(prefs.load_def_prefs);
  Serial_print("  version:    ");      Serial_println(prefs.version);
  Serial_print("  url_data:   ");  Serial_println(prefs.url_data);
  Serial_print("  url_prefs:  ");  Serial_println(prefs.url_prefs);
  Serial_print("  url_errors: ");  Serial_println(prefs.url_errors);
  Serial_print("  url_stream: ");  Serial_println(prefs.url_stream);
  Serial_println();

  Serial_print("  light_sleep_enabled:     ");  Serial_println(prefs.light_sleep_enabled);
  Serial_print("  sleep_enabled:           ");  Serial_println(prefs.sleep_enabled);
  Serial_print("  sleep_dur_min:           ");  Serial_println(prefs.sleep_dur_min);
  Serial_print("  sleep_2_send_interval_s: ");  Serial_println(prefs.sleep_2_send_interval_s);
  Serial_print("  sleep_hour_start:        ");  Serial_println(prefs.sleep_hour_start);
  Serial_print("  sleep_hour_end:          ");  Serial_println(prefs.sleep_hour_end);
  Serial_println();

  Serial_print("  store_wind_data_interval_s: "); Serial_println(prefs.store_wind_data_interval_s);
  Serial_print("  wind_log_store_len:   "); Serial_println(prefs.wind_log_store_len);
  Serial_print("  send_data_interval_min:     "); Serial_println(prefs.send_data_interval_min);
  Serial_print("  n_send_retries:             "); Serial_println(prefs.n_send_retries);
  Serial_println();

  Serial_print("  at_timeout_s:    ");       Serial_println(prefs.at_timeout_s);
  Serial_print("  sim_timeout_s:   ");       Serial_println(prefs.sim_timeout_s);
  Serial_print("  csq_timeout_s:   ");       Serial_println(prefs.csq_timeout_s);
  Serial_print("  creg_timeout_s:  ");       Serial_println(prefs.creg_timeout_s);
  Serial_print("  cgreg_timeout_s: ");       Serial_println(prefs.cgreg_timeout_s);
  Serial_println();  

  Serial_print("  error_led_on_time_ms:  ");  Serial_println(prefs.error_led_on_time_ms);
  Serial_print("  dir_led_on_time_ms:    ");  Serial_println(prefs.dir_led_on_time_ms);
  Serial_print("  spin_led_on_time_ms:   ");  Serial_println(prefs.spin_led_on_time_ms);
  Serial_print("  blink_led_on_time_ms:  ");  Serial_println(prefs.blink_led_on_time_ms);
  Serial_print("  blink_led_interval_ds: ");  Serial_println(prefs.blink_led_interval_ds);
  Serial_println();

  Serial_print("  wind_dir_read_interval_s: "); Serial_println(prefs.wind_dir_read_interval_s);
  Serial_print("  enable_wind_speed_read:   "); Serial_println(prefs.enable_wind_speed_read);
  Serial_println();

  Serial_print("  read_temp_enabled:          "); Serial_println(prefs.read_temp_enabled);
  Serial_println();

  Serial_print("  vbat_calib:   ");   Serial_println(String(prefs.vbat_calib, 9));
  Serial_print("  vsolar_calib: "); Serial_println(String(prefs.vsolar_calib, 9));
  Serial_println("---------------------");
}


bool saveNewPrefValue(String key, String value) {
  Serial_print("Saving the new pref: '");
  Serial_print(key); Serial_print("':'"); Serial_print(value); Serial_println("'");

  if(key == "pref_version") {
    prefs.pref_version = value.toInt();
  } 
  else if(key == "version") {
    value.toCharArray(prefs.version, sizeof(prefs.version));
  } 
  else if(key == "load_def_prefs") {
    prefs.load_def_prefs = value.toInt();
  }
  else if(key == "url_data") {
    value.toCharArray(prefs.url_data, sizeof(prefs.url_data));
  }
  else if(key == "url_prefs") {
    value.toCharArray(prefs.url_prefs, sizeof(prefs.url_prefs));
  }
  else if(key == "url_errors") {
    value.toCharArray(prefs.url_errors, sizeof(prefs.url_errors));
  }
  else if(key == "url_stream") {
    value.toCharArray(prefs.url_stream, sizeof(prefs.url_stream));
  }
  else if(key == "store_wind_data_interval_s") {
    prefs.store_wind_data_interval_s = value.toInt();
  }
  else if(key == "read_temp_enabled") {
    prefs.read_temp_enabled = value.toInt();
  }
  else if(key == "error_led_on_time_ms") {
    prefs.error_led_on_time_ms = value.toInt();
  }
  else if(key == "dir_led_on_time_ms") {
    prefs.dir_led_on_time_ms = value.toInt();
  }
  else if(key == "spin_led_on_time_ms") {
    prefs.spin_led_on_time_ms = value.toInt();
  }
  else if(key == "wind_dir_read_interval_s") {
    prefs.wind_dir_read_interval_s = value.toInt();
  }
  else if(key == "enable_wind_speed_read") {
    prefs.enable_wind_speed_read = value.toInt();
  }
  else if(key == "light_sleep_enabled") {
    prefs.light_sleep_enabled = value.toInt();
  } 
  else if(key == "sleep_enabled") {
    prefs.sleep_enabled = value.toInt();
  } 
  else if(key == "sleep_dur_min") {
    prefs.sleep_dur_min = value.toInt();
  }
  else if(key == "sleep_2_send_interval_s") {
    prefs.sleep_2_send_interval_s = value.toInt();
  }
  else if(key == "sleep_hour_start") {
    prefs.sleep_hour_start = value.toInt();
  } 
  else if(key == "sleep_hour_end") {
    prefs.sleep_hour_end = value.toInt();
  }
  else if(key == "blink_led_on_time_ms") {
    prefs.blink_led_on_time_ms = value.toInt();
  } 
  else if(key == "blink_led_interval_ds") {
    prefs.blink_led_interval_ds = value.toInt();
  }
  else if(key == "send_data_interval_min") {
    prefs.send_data_interval_min = value.toInt();
  }
  else if(key == "n_send_retries") {
    prefs.n_send_retries = value.toInt();
  }
  else if(key == "at_timeout_s") {
    prefs.at_timeout_s = value.toInt();
  }
  else if(key == "sim_timeout_s") {
    prefs.sim_timeout_s = value.toInt();
  }
  else if(key == "csq_timeout_s") {
    prefs.csq_timeout_s = value.toInt();
  }
  else if(key == "creg_timeout_s") {
    prefs.creg_timeout_s = value.toInt();
  }
  else if(key == "cgreg_timeout_s") {
    prefs.cgreg_timeout_s = value.toInt();
  }
  else if(key == "vbat_calib") {
    prefs.vbat_calib = value.toFloat();
  }
  else if(key == "vsolar_calib") {
    prefs.vsolar_calib = value.toFloat();
  }
  else if(key == "wind_log_store_len") {
    prefs.wind_log_store_len = value.toInt();
  }

  else {
    Serial_print("Unable to find the prefs key: '"); Serial_print(key); Serial_println("'");
    return false; // no new key was set
  }

  return true; // a new prefs key was set
}


String getPostBodyPrefs() {
  String body;
  body.reserve(1024);   // avoid fragmentation, improve speed

  body += "compiled_on=" + getFormattedUnixTime(BUILD_UNIX_TIME) + ";";
  body += "pref_version=" + String(prefs.pref_version) + ";";
  body += "pref_set_date=" + getFormattedUnixTime(prefs.pref_set_date) + ";";
  body += "load_def_prefs=" + String(prefs.load_def_prefs) + ";";
  body += "version=" + String(prefs.version) + ";";
  body += "url_data=" + String(prefs.url_data) + ";";
  body += "url_prefs=" + String(prefs.url_prefs) + ";";
  body += "url_errors=" + String(prefs.url_errors) + ";";
  body += "url_stream=" + String(prefs.url_stream) + ";";

  body += "light_sleep_enabled=" + String(prefs.light_sleep_enabled) + ";";
  body += "sleep_enabled=" + String(prefs.sleep_enabled) + ";";
  body += "sleep_dur_min=" + String(prefs.sleep_dur_min) + ";";
  body += "sleep_2_send_interval_s=" + String(prefs.sleep_2_send_interval_s) + ";";
  body += "sleep_hour_start=" + String(prefs.sleep_hour_start) + ";";
  body += "sleep_hour_end=" + String(prefs.sleep_hour_end) + ";";

  body += "store_wind_data_interval_s=" + String(prefs.store_wind_data_interval_s) + ";";
  body += "read_temp_enabled=" + String(prefs.read_temp_enabled) + ";";
  body += "send_data_interval_min=" + String(prefs.send_data_interval_min) + ";";
  body += "n_send_retries=" + String(prefs.n_send_retries) + ";";

  body += "at_timeout_s=" + String(prefs.at_timeout_s) + ";";
  body += "sim_timeout_s=" + String(prefs.sim_timeout_s) + ";";
  body += "csq_timeout_s=" + String(prefs.csq_timeout_s) + ";";
  body += "creg_timeout_s=" + String(prefs.creg_timeout_s) + ";";
  body += "cgreg_timeout_s=" + String(prefs.cgreg_timeout_s) + ";";

  body += "error_led_on_time_ms=" + String(prefs.error_led_on_time_ms) + ";";
  body += "dir_led_on_time_ms=" + String(prefs.dir_led_on_time_ms) + ";";
  body += "spin_led_on_time_ms=" + String(prefs.spin_led_on_time_ms) + ";";
  body += "blink_led_on_time_ms=" + String(prefs.blink_led_on_time_ms) + ";";
  body += "blink_led_interval_ds=" + String(prefs.blink_led_interval_ds) + ";";

  body += "wind_dir_read_interval_s=" + String(prefs.wind_dir_read_interval_s) + ";";
  body += "enable_wind_speed_read=" + String(prefs.enable_wind_speed_read) + ";";

  body += "wind_log_store_len=" + String(prefs.wind_log_store_len) + ";";

  body += "vbat_calib=" + String(prefs.vbat_calib, 9) + ";";
  body += "vsolar_calib=" + String(prefs.vsolar_calib, 9) + ";";

  body += "err_ver=" + String(ErrorLogger::ERROR_CODE_VERSION) + ";";

  return body;
}
