class ErrorLogger {
public:

    static constexpr int ERROR_CODE_VERSION = 4;

    // --- Enum of all possible errors ---
    enum ErrorCode {
        ERR_NONE = 0,               // no error

        // ---- SEND / GSM / HTTP ----
        ERR_SEND_AT_FAIL                 = 1,
        ERR_SEND_NO_SIM                  = 2,
        ERR_SEND_CSQ_FAIL                = 3,
        ERR_SEND_REG_FAIL                = 4,
        ERR_SEND_CCLK_FAIL               = 5,
        ERR_SEND_CIMI_FAIL               = 6,
        ERR_SEND_GPRS_FAIL               = 7,
        ERR_SEND_HTTP_FAIL_DATA          = 8,
        ERR_SEND_UNKWN_FAIL              = 10,
        ERR_SEND_REPEAT                  = 11,
        ERR_SEND_FAIL_WRONG_RESPONSE     = 12,
        ERR_SEND_PREFS_HTTP_FAIL         = 13,
        ERR_SEND_PREFS_HTTP_FAIL_RESPONSE= 14,
        ERR_SEND_ERRORS_HTTP_FAIL        = 15,
        ERR_SEND_ERRORS_HTTP_FAIL_RESPONSE = 16,
        ERR_SEND_V_BATT_TOO_LOW          = 17, 

        ERR_TCP_CONN_START, // TODO asign numbers to all the errors and shitf other error names
        ERR_TCP_SEND_DATA,
        ERR_TCP_SEND_PREFS,
        ERR_TCP_SEND_ERRORS,
        ERR_TCP_CONN_CLOSED,
        ERR_TCP_CONN_NOT_OK,
        ERR_TCP_CIPSEND,
        ERR_TCP_NO_SRV_RESPONSE,
        ERR_TCP_WRONG_RESPONSE_LEN,
        ERR_TCP_NO_GOT_RESPONSE,

        // ---- direction wind vane ----
        ERR_DIR_READ             = 20,
        ERR_DIR_READ_ONCE        = 21,
        ERR_DIR_NOT_CONNECTED    = 22,
        ERR_DIR_SHORT_BUF_FULL   = 23,
        ERR_DIR_SDA_NOT_CONN     = 24,
        ERR_DIR_SCL_NOT_CONN     = 25,
        ERR_DIR_MAG_WEAK         = 26,

        // ---- WIND ----
        ERR_WIND_BUF_OVERWRITE   = 30,
        ERR_WIND_SHORT_BUF_FULL  = 31,
        ERR_WIND_NOT_CONNECTED   = 32,

        // ---- TEMP ----
        ERR_TEMP_READ_OUT        = 40,
        ERR_TEMP_READ_IN         = 41,

        // ---- POWER / RESET ----
        ERR_RESET_BROWNOUT       = 52,
        ERR_RESET_PANIC          = 53,
        ERR_RESET_WDT            = 54,
        ERR_RESET_UNEXPECTED     = 55,
        ERR_CANT_SEND_FORCE_RST  = 56,
        
        LOG_RESET_SW             = 70,
        LOG_RESET_POWERON        = 71,

        ERR_COUNT_MAX            = 100
    };

    ErrorLogger() {

    }

    void init() {
        _prefs.begin("errlog2", false);
        for (int i = 0; i < ERR_COUNT_MAX; ++i) {
            _errorCounts[i] = _prefs.getUShort(String(i).c_str(), 0);
        }
    }

    ~ErrorLogger() {
        _prefs.end();
    }

    // --- Log (increment) an error count and store in Preferences ---
    void log(ErrorCode code) {
        if (code >= 0 && code < ERR_COUNT_MAX) {
            _errorCounts[code]++;
            int n_written = _prefs.putUShort(String(code).c_str(), _errorCounts[code]);
            
            // debug prints:
            //Serial_print("Log error: "); Serial_println(code);
            //Serial_print("n bytes written: "); Serial_println(n_written);
            //uint16_t test = _prefs.getUShort(String(code).c_str(), 12345);
            //Serial_print("count: "); Serial_println(test);
            //Serial_print("err count: "); Serial_println(_errorCounts[code]);
        }
        nErrorsLogged ++;
    }

    void logTmp(ErrorCode code) {
        if (code >= 0 && code < ERR_COUNT_MAX) {
            _errorCountsTemp[code]++;
            //Serial_print("Log error tmp: "); Serial_println(code);
        }
        nErrorsLogged ++;
    }

    // --- Get number of times an error occurred ---
    int32_t get(ErrorCode code) {
        if (code >= 0 && code < ERR_COUNT_MAX) {
            int count = _prefs.getUShort(String(code).c_str(), 0);
            count += _errorCountsTemp[code];
            return count;
        }
        return -1;
    }

    // --- return all stored error counts used for debug prints ---
    String getAll() {
        String out;
        for (int code = ERR_NONE; code < ERR_COUNT_MAX; ++code) {
            int count = get((ErrorCode) code);

            if (count > 0) {
                String msg = String(code) + ": " +
                             errorToString((ErrorCode)code) +
                             " -> count=" + String(count) + "\r\n";
                out += msg;
            }
        }
        return out;
    }

    // --- return all stored error counts used for sending to server ---
    String getAllForSend() {
        String out;
        bool first = true;
        for (int code = 1; code < ERR_COUNT_MAX; ++code) {
            int count = get((ErrorCode)code);


            if (count > 0) {
                if(!first) out += ",";
                first = false;
                String msg = String(code) + ":" + String(count);
                out += msg;
            }
        }
        return out;
    }

    int getNErrorsLogged() {
        return nErrorsLogged;
    }

    // --- Clear a specific error counter (and in Preferences) ---
    void clear(ErrorCode code) {
        if (code >= 0 && code < ERR_COUNT_MAX) {
            if(_errorCounts[code] != 0) _prefs.putUShort(String(code).c_str(), 0);
            
            _errorCountsTemp[code] = 0;
            _errorCounts[code] = 0;
        }
    }

    // --- Clear all error counters (and in Preferences) ---
    void clearAll() {
        for (int i = 0; i < ERR_COUNT_MAX; ++i) {
            clear((ErrorCode)i);
        }
    }

    static String getPostErrorsList() {
        String body;
        body.reserve(900);
        body += "ERR_NONE=0;";

        // ---- SEND / GSM / HTTP ----
        body += "ERR_SEND_AT_FAIL=1;";
        body += "ERR_SEND_NO_SIM=2;";
        body += "ERR_SEND_CSQ_FAIL=3;";
        body += "ERR_SEND_REG_FAIL=4;";
        body += "ERR_SEND_CCLK_FAIL=5;";
        body += "ERR_SEND_CIMI_FAIL=6;";
        body += "ERR_SEND_GPRS_FAIL=7;";
        body += "ERR_SEND_HTTP_FAIL_DATA=8;";
        body += "ERR_SEND_UNKWN_FAIL=10;";
        body += "ERR_SEND_REPEAT=11;";
        body += "ERR_SEND_FAIL_WRONG_RESPONSE=12;";
        body += "ERR_SEND_PREFS_HTTP_FAIL=13;";
        body += "ERR_SEND_PREFS_HTTP_FAIL_RESPONSE=14;";
        body += "ERR_SEND_ERRORS_HTTP_FAIL=15;";
        body += "ERR_SEND_ERRORS_HTTP_FAIL_RESPONSE=16;";
        body += "ERR_SEND_V_BATT_TOO_LOW=17;";

        // ---- DIR / I2C ----
        body += "ERR_DIR_READ=20;";
        body += "ERR_DIR_READ_ONCE=21;";
        body += "ERR_DIR_NOT_CONNECTED=22;";
        body += "ERR_DIR_SHORT_BUF_FULL=23;";
        body += "ERR_DIR_SDA_NOT_CONN=24;";
        body += "ERR_DIR_SCL_NOT_CONN=25;";
        body += "ERR_DIR_MAG_WEAK=26;";

        // ---- WIND ----
        body += "ERR_WIND_BUF_OVERWRITE=30;";
        body += "ERR_WIND_SHORT_BUF_FULL=31;";
        body += "ERR_WIND_NOT_CONNECTED=32;";

        // ---- TEMP ----
        body += "ERR_TEMP_READ_OUT=40;";
        body += "ERR_TEMP_READ_IN=41;";

        // ---- POWER / RESET ----
        body += "ERR_RESET_BROWNOUT=52;";
        body += "ERR_RESET_PANIC=53;";
        body += "ERR_RESET_WDT=54;";
        body += "ERR_RESET_UNEXPECTED=55;";
        body += "ERR_CANT_SEND_FORCE_RST=56;";

        body += "LOG_RESET_SW=70;";
        body += "LOG_RESET_POWERON=71;";

        return body;
    }

    static const char* errorToString(ErrorCode code) {
        switch (code) {
            case ERR_NONE:
                return "No error";

            // ---- SEND / GSM / HTTP ----
            case ERR_SEND_AT_FAIL:
                return "AT command failure";
            case ERR_SEND_NO_SIM:
                return "No SIM card detected";
            case ERR_SEND_CSQ_FAIL:
                return "Signal quality (CSQ) check failed";
            case ERR_SEND_REG_FAIL:
                return "Network registration failed";
            case ERR_SEND_CCLK_FAIL:
                return "Network time (CCLK) retrieval failed";
            case ERR_SEND_CIMI_FAIL:
                return "IMSI (CIMI) retrieval failed";
            case ERR_SEND_GPRS_FAIL:
                return "GPRS / data connection failed";
            case ERR_SEND_HTTP_FAIL_DATA:
                return "HTTP data send failed";
            case ERR_SEND_UNKWN_FAIL:
                return "Unknown send failure";
            case ERR_SEND_REPEAT:
                return "Send had to be repeated";
            case ERR_SEND_FAIL_WRONG_RESPONSE:
                return "Send failed due to wrong response";

            case ERR_SEND_PREFS_HTTP_FAIL:
                return "HTTP prefs send failed";
            case ERR_SEND_PREFS_HTTP_FAIL_RESPONSE:
                return "HTTP prefs response invalid";
            case ERR_SEND_ERRORS_HTTP_FAIL:
                return "HTTP error-log send failed";
            case ERR_SEND_ERRORS_HTTP_FAIL_RESPONSE:
                return "HTTP error-log response invalid";
            case ERR_SEND_V_BATT_TOO_LOW:
                return "Battery voltage too low to send";

            // ---- DIR / I2C ----
            case ERR_DIR_READ:
                return "Direction read error";
            case ERR_DIR_READ_ONCE:
                return "Direction read error (single occurrence)";
            case ERR_DIR_NOT_CONNECTED:
                return "Direction sensor not connected";
            case ERR_DIR_SHORT_BUF_FULL:
                return "Direction short buffer full";
            case ERR_DIR_SDA_NOT_CONN:
                return "Direction sensor SDA not connected";
            case ERR_DIR_SCL_NOT_CONN:
                return "Direction sensor SCL not connected";
            case ERR_DIR_MAG_WEAK:
                return "Magnet too weak";

            // ---- WIND ----
            case ERR_WIND_BUF_OVERWRITE:
                return "Wind buffer overwrite";
            case ERR_WIND_SHORT_BUF_FULL:
                return "Wind short buffer full";
            case ERR_WIND_NOT_CONNECTED:
                return "Wind speed senosr not connected";

            // ---- TEMP ----
            case ERR_TEMP_READ_OUT:
                return "Temperature OUTside read error";
            case ERR_TEMP_READ_IN:
                return "Temperature INside read error";

            // ---- POWER / RESET ----
            case ERR_RESET_BROWNOUT:
                return "Brown-out / low-voltage reset";
            case ERR_RESET_PANIC:
                return "Software panic / abort reset";
            case ERR_RESET_WDT:
                return "Watchdog reset";
            case ERR_RESET_UNEXPECTED:
                return "Unexpected / unclassified reset";
            case ERR_CANT_SEND_FORCE_RST:
                return "Unable to send for long time, force reseting";

            case LOG_RESET_SW:
                return "Log software reset";

            case LOG_RESET_POWERON:
                return "Log reset poweron";

            default:
                return "Unknown ErrorCode";
        }
    }

private:
    static constexpr int ERROR_COUNT = ERR_COUNT_MAX;
    int nErrorsLogged = 0;
    uint16_t _errorCounts[ERROR_COUNT];
    uint16_t _errorCountsTemp[ERROR_COUNT];
    Preferences _prefs;
};
