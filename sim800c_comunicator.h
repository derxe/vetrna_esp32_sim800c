#define SerialAT Serial1

#define PRINT_SIM_COMM

#ifdef PRINT_SIM_COMM
  #define DBG_CMD(...) do { __VA_ARGS__; } while (0)
#else
  #define DBG_CMD(...) do {} while (0)
#endif

enum class SendResult : int {
  OK                   =  1,   // success
  AT_FAIL              = -1,   // "AT" didn't respond
  NO_SIM               = -2,   // CSMINS? => no SIM
  CSQ_FAIL             = -3,   // CSQ failed
  REG_FAIL             = -4,   // CREG? or CGREG? failed (both map here)
  CCLK_FAIL            = -5,   // failed getting time
  CLTS_NOT_SET         = -6,   // CLTS value was set to 0 (setting for getting time from the server )
  CIMI_FAIL            = -7,   // CIMI failed
  GPRS_SETUP_FAIL      = -8,   // any SAPBR step failed
  HTTP_FAIL            = -9,   // sendPOSTData / HTTPREAD failed
};

enum class TcpStatus : int {
  CONNECTING = 1,
  CLOSED,
  CONNECT_OK,
  UNKNOWN,
  TIMEOUT,
};

String sendCommand(const String& command, int timeoutMs, const String& expectedResponse);
void setPhoneNumber(String& inputBuffer);
const char* regStatusToStr(int status);
bool parseCSQResponse(String& response);
bool parseCGREGResponse(String& response);
bool parseCIMIResponse(String& response);
void readPhoneNum();
bool parseCCLKResponse(String& response);
bool parseCLTSResponse(String& response);
bool parseHTTPREADResponse(String& response);
bool parseCSMINSResponse(String& response);
bool startTcpConnection();
bool establishConnection();
void sendStreamHttp(int durationSec);
String getPostBodyPrefsAll();
String emptySerialAT();
bool parseCIPRXGET4(String& response);
TcpStatus getTCPStatus(unsigned long timeoutMs);
TcpStatus checkTCPstatus(unsigned long timeoutS);
String sendTCPData(const String& dataToSend, bool waitForReply);
bool sendTCP_break_into_packets(const String& data);
void sendStreamTCP(int durationSec);
String waitForATResponse(int timeoutMs, const String& expectedResponse);
bool sendPOST(const String &url, const String &body);
bool sendGET(const String &url);
String waitForHttpActionResponse(unsigned long timeoutMs);


// extranls function from main: 
uint32_t get_log_timestamp(int hour, int minute, int second);
uint32_t get_log_timestamp();
void windlog_shift_timestamps(int32_t delta);
float read_batt_v();
float read_solar_v();
void savePreferences();
String getPostBody();
String getPostBodyShort();
extern bool accurateTimeSet;
extern time_t accurateTimeSetAt;
extern volatile bool enableErrorLedBlinking;
extern int16_t lastSpeedRead;
extern int last_direction_read;
extern ErrorLogger elog;


unsigned long httpGetStart = 0;
int signalStrength = -1;
int simDuration = -1;
int regDuration = -1;
int gprsRegDuration = -1;
float temp_in = NAN;
float hum_in = NAN; 
float temp_out = NAN;
float hum_out = NAN; 


void shiftTimestampsOnNewTime(int newHour, int newMinute, int newSecond) {
  uint32_t oldTimestamp = get_log_timestamp();                              // Old timestamp (before correction)
  uint32_t newTimestamp = get_log_timestamp(newHour, newMinute, newSecond); // New correct time

  int32_t delta = newTimestamp - oldTimestamp;                      // How much the clock moved
  Serial_print("Shifting timestamps by: "); Serial_println(delta);
  windlog_shift_timestamps(delta);
}

String waitForATResponse(int timeoutMs, const String& expectedResponse) {
  unsigned long start = millis();
  String response = "";

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      response += c;

      char c_print = c;
      if(c_print == '\n') c_print = '.';
      if(c_print == '\r') c_print = ',';
      DBG_CMD( Serial_write(c_print); ); 

      // Early exit if we detect "OK"
      if (response.indexOf(expectedResponse) != -1) {
        DBG_CMD( Serial_println(";"); );
        return response;
      }
    }
    delay(1); // Yield to avoid tight spinning
  }

  #ifdef PRINT_SIM_COMM
  Serial_println("norsps?");
  //Serial_print("Response: '"); Serial_print(response); Serial_println("'");
  #endif 

  return response;
}

String sendCommand(const String& command, int timeoutMs = 1000, const String& expectedResponse = "OK") {
  DBG_CMD( Serial_println() );
  emptySerialAT();
  SerialAT.println(command);

  DBG_CMD( Serial_print("#Command: "); Serial_println(command); );
  
  String response = waitForATResponse(timeoutMs, expectedResponse);

  //DBG_CMD( Serial_print("##Response: '"); Serial_print(response); Serial_println("'") );
  return response;
}


bool waitForResponse(const String& command,
                     unsigned long timeoutS,
                     bool (*parseFn)(String&),
                     unsigned long retryDelay = 1000) {
  String response = "";
  unsigned long start = millis();
  bool responseOk = false;

  do {
    response = sendCommand(command);
    responseOk = parseFn ? parseFn(response) : (response != "");

    if (!responseOk) {
      if (millis() - start > timeoutS*1000) {
        Serial_print("ERROR: Command: '"); Serial_print(command); 
        Serial_println("' timeouted, aborting.");
        return false;
      }
      delay(retryDelay); // wait before retrying
    }
  } while (!responseOk);

  Serial_print(command); Serial_print(" done. Duration: "); 
  Serial_print(String((millis() - start)/1000.0, 2)); Serial_println("s");
  return true;
}





void setPhoneNumber(String& inputBuffer) {
  String digits = "";
  for(int i=0; i<inputBuffer.length(); i++) {
    if (isDigit(inputBuffer[i])) digits += inputBuffer[i];
  }

  Serial_print("Setting phone number:");
  Serial_println(digits);

  Serial_println(sendCommand("AT+CPBS=\"ON\""));
  Serial_println(sendCommand("AT+CPBW=1,\"" + digits + "\",129,\"mojast\""));

  Serial_println(sendCommand("AT+CNUM"));

  Serial_println("Done!");
}

const char* regStatusToStr(int status) {
  switch (status) {
    case 0: return "Not registered, not searching";
    case 1: return "Registered, home network";
    case 2: return "Not registered, searching";
    case 3: return "Registration denied";
    case 4: return "Unknown";
    case 5: return "Registered, roaming";
    default: return "Invalid status";
  }
}

bool parseCSQResponse(String& response) {
  int idx = response.indexOf("+CSQ:");
  if (idx == -1) return false;

  // Extract substring starting after "+CSQ:"
  String sub = response.substring(idx + 5);
  sub.trim(); // remove leading/trailing whitespace

  int commaIdx = sub.indexOf(',');
  if (commaIdx == -1) return false;

  int rssi = sub.substring(0, commaIdx).toInt();
  Serial_print("s:");
  Serial_print(rssi);

  if(rssi > 0) {
    Serial_print("; signal strenght: ");
    Serial_print((rssi * 827 + 127) / 256);
    Serial_println("%");
  }
  signalStrength = rssi;
  return rssi > 0 && rssi < 99; // 99 = unknown or no signal
}

bool parseCGREGResponse(String& response) {
  int idx = response.indexOf("REG:");
  if (idx == -1) return false;

  String sub = response.substring(idx + 4);
  sub.trim();

  int commaIdx = sub.indexOf(',');
  if (commaIdx == -1) return false;

  int status = sub.substring(commaIdx + 1).toInt();

  //Serial_print("GPRS registration status: ");
  //Serial_println(regStatusToStr(status));
  Serial_print(status);
  return status == 1 || status == 5; // Registered (home or roaming)
}


String imsiNum; 

bool parseCIMIResponse(String& response) {
  int cmd = response.indexOf("AT+CIMI");
  if (cmd == -1) return false;

  int ok = response.indexOf("\nOK", cmd);
  if (ok == -1) ok = response.indexOf("\r\nOK", cmd);
  if (ok == -1) return false;

  // Work only inside the CIMI block
  String block = response.substring(cmd, ok);

  // Split into lines, find the first line that looks like an IMSI
  int start = 0;
  while (start < block.length()) {
    int end = block.indexOf('\n', start);
    if (end == -1) end = block.length();

    String line = block.substring(start, end);
    line.trim();

    // Skip echoes and empty lines
    if (line == "AT+CIMI" || line.length() == 0) {
      start = end + 1;
      continue;
    }

    // Keep only digits in this line
    String d = "";
    for (int i = 0; i < line.length(); i++) {
      if (isDigit(line[i])) d += line[i];
    }

    // IMSI is usually exactly 15 digits
    if (d.length() == 15) {
      imsiNum = d;
      return true;
    }

    start = end + 1;
  }

  return false;
}
String phoneNum; 

void readPhoneNum() {
  String response = sendCommand("AT+CNUM");
  int idx;
  idx = response.indexOf("+CNUM"); if (idx == -1) return;
  idx = response.indexOf("OK"); if (idx == -1) return;

  int firstQuote = response.indexOf(",\""); if (firstQuote == -1) return;
  int secondQuote = response.indexOf('"', firstQuote + 2); if (secondQuote == -1) return;

  phoneNum = response.substring(firstQuote + 2, secondQuote);
}


bool parseCCLKResponse(String& response) {
  // expected response example: +CCLK: "25/08/01,00:19:52+08"
  int idx = response.indexOf("CCLK:");
  if (idx == -1) return false;

  int quoteStart = response.indexOf('"', idx);
  int quoteEnd = response.indexOf('"', quoteStart + 1);
  if (quoteStart == -1 || quoteEnd == -1) return false;

  String datetime = response.substring(quoteStart + 1, quoteEnd);  // "25/08/01,00:19:52+08"

  // Split by delimiters: / , :
  int year   = datetime.substring(0, 2).toInt() + 2000;
  int month  = datetime.substring(3, 5).toInt();
  int day    = datetime.substring(6, 8).toInt();
  int hour   = datetime.substring(9, 11).toInt();
  int minute = datetime.substring(12, 14).toInt();
  int second = datetime.substring(15, 17).toInt();

  // Check for default time (00/01/01 etc.)
  if (year < 2023) return false;  // Reject obviously invalid time

  Serial_println("Got new date!");
  // save the time inside the ESP32 system clock
  shiftTimestampsOnNewTime(hour, minute, second); 
  set_system_time_ymdhms(year, month, day, hour, minute, second);
  accurateTimeSet = true;
  accurateTimeSetAt = (time_t)now_rtc_s();

  // Print the parsed time
  Serial_print("New date:"); Serial_println(getFormattedTimeLibString());
  
  return true;
}

bool parseCLTSResponse(String& response) {
  // expected response example: +CLTS: 1 or +CLTS: 0
  int idx = response.indexOf("+CLTS:");
  if (idx == -1) return false;

  // Extract the char after "+CLTS:"
  char clts_val = response[idx + 7];

  //Serial_println("got clts value: " + String(clts_val));
  return clts_val == '1';
}

bool shouldUpdateAccurateTime() {
  int64_t secondsSinceAccurateTimeSet = now_rtc_s() - (int64_t)accurateTimeSetAt;
  return !accurateTimeSet || accurateTimeSetAt == 0 ||
         secondsSinceAccurateTimeSet < 0 ||
         secondsSinceAccurateTimeSet >= 2 * 60 * 60; // check every 2 hours if the time is still accurate
}

bool parseCSMINSResponse(String& response) {
  int idx = response.indexOf("+CSMINS:");
  if (idx == -1) return false;

  return response.indexOf("0,1") != -1;
}


/*
This function parses the default format in the form of:
+CIPRXGET: 4,19
or
+CREG: 1,1

response: the full unparsed string ("+CIPRXGET: 4,19") 
expectedResCommand: expected command to look for insde the response string ("+CIPRXGET: ")
parameters: where to write the int paramters 
maxParameters: how long is the paramters list
paramtersCount: returns how many paramters have we found
*/
bool parseParamsResponse(
    String& response,
    const String& expectedResCommand,
    int parameters[],
    int maxParameters,
    int& parametersCount
) {
    parametersCount = 0;

    int okPos = response.lastIndexOf("\r\nOK");
    if (okPos == -1) return false;

    int cmdPos = response.lastIndexOf(expectedResCommand);
    if (cmdPos == -1) return false;

    int lineEnd = response.indexOf("\n", cmdPos);
    if (lineEnd == -1) lineEnd = response.length();

    String params = response.substring(cmdPos + expectedResCommand.length(), lineEnd);
    params.trim();

    while (params.length() > 0 && parametersCount < maxParameters) {
        int commaPos = params.indexOf(',');

        if (commaPos == -1) { // handle the last paramters in the line
            parameters[parametersCount++] = params.toInt();;
            break;
        }

        parameters[parametersCount++] = params.substring(0, commaPos).toInt();
        params = params.substring(commaPos + 1);
    }

    response = response.substring(lineEnd+1, okPos); // return the remaining string after the paramters

    // print the parsed parameters
    //for(int i=0; i<parametersCount; i++) {
    //  Serial_print("Param: "); Serial_println(parameters[i]);
    //} 

    return true;
}


bool parseCIPRXGET4(String& response) {
  int params_int[4]; 
  int params_cnt = 0;

  // expected response: 
  // +CIPRXGET: 4,19 where the second int is the num of bytes that has been sent by the server and not yet read
  bool parseSuccess  = parseParamsResponse(response, "+CIPRXGET: ", params_int, 4, params_cnt);
  if(!parseSuccess) return false;
  if(params_cnt != 2) return false;

  int data_to_read = params_int[1];
  Serial_print("Got server response ");  Serial_print(data_to_read);  Serial_println(" bytes");  
  return data_to_read > 0;
}



/*
Parse the response data:
The expected payload is: 
saved: <num written>\n
params:\n
<param name>:<param_value>\n
<param name>:<param_value>\n
<param name>:<param_value>\n
<param name>:<param_value>\n
*/
bool shouldSendPrefs = false; 
bool shouldSendErrorNames = false; 
bool shouldReset = false; 
int send_stream_for_s = 0;
void parseReturnData(String& data) {
  shouldReset = true; 
  shouldSendPrefs = true; // on default we always reset the esp after changing preferences
  shouldSendErrorNames = false;
  bool newPrefsSet = false;
  int prefsPos = data.indexOf("prefs:");
  if (prefsPos < 0) {
    Serial_println("No 'prefs:' section found.");
    shouldReset = false; 
    shouldSendPrefs = false;
    return;
  }

  // Start after "prefs"
  int pos = data.indexOf('\n', prefsPos);
  if (pos < 0) {
    Serial_println("No newline after 'params'.");
    shouldReset = false; 
    shouldSendPrefs = false;
    return;
  }
  pos++; // move past newline


  while (pos < data.length()) {
    // Find the next colon — separates key from value
    int colonPos = data.indexOf(':', pos);
    if (colonPos < 0) break;

    // Find the next newline — end of this line
    int lineEnd = data.indexOf('\n', colonPos);
    if (lineEnd < 0) lineEnd = data.length();

    // Extract key and value
    String key = data.substring(pos, colonPos);
    String value = data.substring(colonPos + 1, lineEnd);

    // Trim simple whitespace and trailing commas
    key.trim();
    value.trim();

    Serial_print(" - pref key:"); Serial_println(key); 

    if(key == "no_reset") shouldReset = false; 
    else if(key == "no_send_prefs") shouldSendPrefs = false; 
    else if(key == "set_phone_num" || key == "phoneNum") setPhoneNumber(value);
    else if(key == "send_error_names") shouldSendErrorNames = true;
    else if(key == "send_stream_for_s") send_stream_for_s = value.toInt();
    else {
      bool prefsSet = saveNewPrefValue(key, value); 
      if(prefsSet) newPrefsSet = true;      
    }         
    
    // Advance to next line
    pos = lineEnd + 1;
  }

  if(newPrefsSet) {
    prefs.pref_set_date = (uint32_t)now_rtc_s();  // save when the preferences were set
    savePreferences();
    Serial_println("Done params section.\n");
  } else {
    Serial_println("No new preferences set.\n");
  }
  
}



String zeros(int length) {
  String result;
  result.reserve(length);
  for (int i = 0; i < length; ++i) {
    result += '0';
  }
  return result;
}



// >>>>>>>>>>>>>>< HTTP FUNCTIONS ><<<<<<<<<<<<<<<<<<
String waitForHttpActionResponse(unsigned long timeoutMs) {
  unsigned long start = millis();
  String line = "";

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      char c_print = c;
      if(c_print == '\n') c_print = '.';
      if(c_print == '\r') c_print = ',';
      DBG_CMD( Serial_write(c_print); ); 

      if (c == '\n') {
        if (line.startsWith("+HTTPACTION:")) {
          return line;
        }
        line = "";  // reset for next line
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  Serial_println("Timeout!");
  return "";  // timeout
}

String postReturnData;
bool parseHTTPREADResponse(String& response) {
  int startIdx = response.indexOf("+HTTPREAD:");
  if (startIdx == -1) {
    Serial_println("No +HTTPREAD header found");
    return false;
  }

  // Find the first line break after the header
  int dataStart = response.indexOf('\n', startIdx);
  if (dataStart == -1) return false;

  // Trim until actual data
  String data = response.substring(dataStart + 1);
  data.trim();

  // Remove any trailing "OK" or extra content
  int okIdx = data.indexOf("OK");
  if (okIdx != -1) {
    data = data.substring(0, okIdx);
    data.trim();
  }

  // Print the data
  Serial_print("Parsing return data:'");
  for (char c : data) {
      if (c == '\n') Serial_println(); 
      else Serial_write(c);
  }
  Serial_println("'");

  // store globally
  postReturnData = data;
  return true;
}

void sendStreamHttp(int durationSec) {
  enableErrorLedBlinking = false;

  uint32_t start = millis();
  Serial_println("Starting stream sending for " + String(durationSec) + " seconds...");
  while(millis() - start < durationSec*1000) {
    digitalWrite(ERROR_LED_PIN, HIGH); 

    String url = prefs.url_stream + imsiNum + 
                  "?spd=" + String(lastSpeedRead) + 
                  "&dir=" + String(last_direction_read) +
                  "&bat=" + String(read_batt_v(), 2);

    bool postSuccess = sendGET(url);

    if (!postSuccess) {
      Serial_print("ERROR: Sending stream data failed!");
    } else {
      Serial_print("Stream data sent OK");
    }
    

    digitalWrite(ERROR_LED_PIN, LOW);
    delay(300);
  }

  enableErrorLedBlinking = true;
}


bool sendPOST(const String &url, const String &body) {
    if (sendCommand("AT+HTTPPARA=\"CID\",1", 500) == "") return false;
    if (sendCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"", 500) == "") return false;

    if (sendCommand("AT+HTTPDATA=" + String(body.length()) + ",10000", 500, "DOWNLOAD") == "") return false;
    if (sendCommand(body, 1000) == "") return false;

    if (sendCommand("AT+HTTPACTION=1", 5000) == "") return false;

    String actionResult = waitForHttpActionResponse(10000);
    return (actionResult.indexOf(",2") > 0);
}

bool sendGET(const String &url) {
    if (sendCommand("AT+HTTPPARA=\"CID\",1", 500) == "") return false;
    if (sendCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"", 500) == "") return false;

    if (sendCommand("AT+HTTPACTION=0", 5000) == "") return false;

    String actionResult = waitForHttpActionResponse(10000);
    Serial_print("HTTPACTION result: '"); Serial_print(actionResult); Serial_println("'");
    return (actionResult.indexOf(",200,") > 0);  // HTTP 200 OK
}


bool sendShortMessage = false;
void setSendShortMessage(bool value) {
  sendShortMessage = value;
}


SendResult sendOverHttp() {
    // Step 1: Configure GPRS connection
  if (sendCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 1000) == "") return SendResult::GPRS_SETUP_FAIL;
  
  // telekom
  //if (sendCommand("AT+SAPBR=3,1,\"APN\",\"internet\"", 1000) == "") return SendResult::GPRS_SETUP_FAIL;
  //if (sendCommand("AT+SAPBR=3,1,\"USER\",\"mobitel\"", 1000) == "") return SendResult::GPRS_SETUP_FAIL;
  //if (sendCommand("AT+SAPBR=3,1,\"PWD\",\"internet\"", 1000) == "") return SendResult::GPRS_SETUP_FAIL;

  // hofer
  if (sendCommand("AT+SAPBR=3,1,\"APN\",\"internet.simobil.si\"") == "") return SendResult::GPRS_SETUP_FAIL;
  if (sendCommand("AT+SAPBR=3,1,\"USER\",\"simobil\"") == "") return SendResult::GPRS_SETUP_FAIL;
  if (sendCommand("AT+SAPBR=3,1,\"PWD\",\"internet\"", 2000) == "") return SendResult::GPRS_SETUP_FAIL;

  // Step 2: Open GPRS bearer
  if (sendCommand("AT+SAPBR=1,1", 15000) == "") return SendResult::GPRS_SETUP_FAIL;
  if (sendCommand("AT+SAPBR=2,1", 1000) == "") return SendResult::GPRS_SETUP_FAIL;

  if (sendCommand("AT+HTTPINIT", 1000) == "") return SendResult::HTTP_FAIL;

  String postBody = sendShortMessage? getPostBodyShort() : getPostBody();
  sendShortMessage = false; // reset the flag after sending
  if (!sendPOST(prefs.url_data + imsiNum, postBody)) return SendResult::HTTP_FAIL;

  waitForResponse("AT+HTTPREAD", 5, parseHTTPREADResponse); // TODO make this http read timeout configurable

  if(postReturnData.indexOf("saved:") < 0) {
    // expected "saved:" in the response if it is not there the response from the server was incorrect
    elog.log(ErrorLogger::ERR_SEND_FAIL_WRONG_RESPONSE);
    return SendResult::HTTP_FAIL;
  }

  if(!postReturnData.isEmpty()) {
    parseReturnData(postReturnData);
  }

  if(send_stream_for_s > 0) {
    sendStreamHttp(send_stream_for_s);
  }
  send_stream_for_s = 0;

  if(shouldSendPrefs) {
    Serial_println();
    Serial_println("Sending preferences");
    bool postSuccess = sendPOST(prefs.url_prefs + imsiNum, getPostBodyPrefsAll());
    if (!postSuccess) {
      Serial_print("ERROR: Sending preferences failed!");
      elog.log(ErrorLogger::ERR_SEND_PREFS_HTTP_FAIL);
    } else {
      waitForResponse("AT+HTTPREAD", 5, parseHTTPREADResponse);

      if (postReturnData.indexOf("saved:") < 0) {
        Serial_println("ERROR: Sending preferences failed wrong response!");
        elog.log(ErrorLogger::ERR_SEND_PREFS_HTTP_FAIL_RESPONSE);
      } else {
        Serial_println("Sending prefs OK!");
      }
    }
  }

  if(shouldSendErrorNames) {
    Serial_println();
    Serial_println("Sending error names");
    String errorsList = ErrorLogger::getPostErrorsList();
    bool postSuccess = sendPOST(prefs.url_errors + imsiNum, errorsList);
    if (!postSuccess) {
      Serial_println("ERROR: Sending errors failed!");
      elog.log(ErrorLogger::ERR_SEND_ERRORS_HTTP_FAIL);
    } else {
      waitForResponse("AT+HTTPREAD", 5, parseHTTPREADResponse);

      if (postReturnData.indexOf("saved:") < 0) {
        Serial_println("ERROR: Sending errors failed wrong response!");
        elog.log(ErrorLogger::ERR_SEND_ERRORS_HTTP_FAIL_RESPONSE);
      } else {
        Serial_println("Sending errors OK!");
      }
    }
  }

  // Step 5: Cleanup
  sendCommand("AT+HTTPTERM");
  sendCommand("AT+SAPBR=0,1");

  Serial_print("End sending over HTTP. Success!");
  return SendResult::OK;
}



// >>>>>>>>>>>>>>>>>< TCP functions ><<<<<<<<<<<<<<<<<<<<

bool startTcpConnection() {
  // start TCP connection 
  // prepare for the TCP communication 
  if (sendCommand("AT+CIPSHUT", 1000) == "") return false;  // reset the TCP stack 
  if (sendCommand("AT+CIPRXGET=1", 1000) == "") return false;  // set the data to be read manually
  if (sendCommand("AT+CSTT=\"internet.simobil.si\",\"simobil\",\"internet\"", 1000) == "") return false; 
  if (sendCommand("AT+CIICR", 2000) == "") return false;  // bring up wireless data (activate PDP context)
  if (sendCommand("AT+CIFSR", 1000, "\r\n") == "") return false; // Get your IP address, we should get our IP address. if not If it fails 
  
  // connect to the TCP server
  String command;
  command.reserve(48);
  command = "AT+CIPSTART=\"TCP\",\"";
  command += prefs.tcp_server_ip;
  command += "\",\"";
  command += prefs.tcp_server_port;
  command += '"';
  if (sendCommand(command, 1000) == "") return false;
  
  return true;
}



bool establishConnection() {
  if (!waitForResponse("AT", prefs.at_timeout_s, nullptr)) {
    elog.log(ErrorLogger::ERR_SEND_AT_FAIL);
    return false;
  }

  unsigned long start = millis();
  if (!waitForResponse("AT+CSMINS?", prefs.sim_timeout_s, parseCSMINSResponse, 200)) {
    Serial_println("ERROR: No Sim detected!.");
    elog.log(ErrorLogger::ERR_SEND_NO_SIM);
    return false;
  }
  simDuration = millis() - start;

  if (!waitForResponse("AT+CSQ", prefs.csq_timeout_s, parseCSQResponse, 500)) {
    elog.log(ErrorLogger::ERR_SEND_CSQ_FAIL);
    return false;
  }

  start = millis();
  if (!waitForResponse("AT+CREG?", prefs.creg_timeout_s, parseCGREGResponse, 500)) {
    elog.log(ErrorLogger::ERR_SEND_REG_FAIL);
    return false;
  }
  regDuration = millis() - start;

  start = millis();
  if (!waitForResponse("AT+CGREG?", prefs.cgreg_timeout_s, parseCGREGResponse, 500)) {
    elog.log(ErrorLogger::ERR_SEND_GPRS_FAIL);
    return false;
  }
  gprsRegDuration = millis() - start;

  if (!waitForResponse("AT+CLTS?", 30, parseCLTSResponse, 500)) {
    // clts is not set to 1, setting it and restarting the module
    sendCommand("AT+CLTS=1");
    sendCommand("AT&W");

    elog.log(ErrorLogger::ERR_SEND_CLTS_NOT_SET);
    return false;
  }


  if (shouldUpdateAccurateTime()) {
    if (!waitForResponse("AT+CCLK?", 30, parseCCLKResponse, 500)) {
      elog.log(ErrorLogger::ERR_SEND_CCLK_FAIL);
      return false;
    }
  }

  // get IMSI number 
  if (!waitForResponse("AT+CIMI", 10, parseCIMIResponse, 500)) {
    elog.log(ErrorLogger::ERR_SEND_CIMI_FAIL);
    return false;
  }
  Serial_print("Got IMSI:"); Serial_println(imsiNum);

  // get phone number
  readPhoneNum();
  Serial_print("Got phone Num:"); Serial_println(phoneNum);

  return true;
}


String getPostBodyPrefsAll() {
  String body = getPostBodyPrefs();
  body += "imsi=" + imsiNum + ";";
  body += "phoneNum=" + phoneNum + ";";
  return body;
}

String emptySerialAT() {
  String data_before_send = "";
  while (SerialAT.available()) {
    char c = SerialAT.read();
    DBG_CMD( Serial_write(c); );
    if(c == '\n') c = '.';
    if(c == '\r') c = ',';
    data_before_send += c;
  }

  #ifdef PRINT_SIM_COMM
  if(data_before_send.length() > 0) {
    Serial_print("data before send: '"); Serial_print(data_before_send); Serial_println("'");
  }
  #endif 
  return data_before_send;
}

TcpStatus getTCPStatus(unsigned long timeoutMs=3000) {
  String response = sendCommand("AT+CIPSTATUS", 1000);

  unsigned long start = millis();
  String line = "";

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      //char c_print = c;       
      //if(c_print == '\n') c_print = '.';
      //if(c_print == '\r') c_print = ',';
      //DBG_CMD( Serial_write(c_print); ); 

      if (c == '\n') {
        if (line.startsWith("STATE:")) {
          String tcpStatusStr = line.substring(7); // remove the "state:" from the string 
          Serial_print("Got tcpStatus: '");  Serial_print(tcpStatusStr);  Serial_println("'"); 
          
          if(tcpStatusStr == "CONNECT OK") return TcpStatus::CONNECT_OK;
          if(tcpStatusStr == "TCP CLOSED") return TcpStatus::CLOSED;
          if(tcpStatusStr == "TCP CONNECTING") return TcpStatus::CONNECTING;
          else return TcpStatus::UNKNOWN;
        }
        line = "";  // reset for next line
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  Serial_println("Timeout getting tcpStatus!");
  return TcpStatus::TIMEOUT;  // timeout
}


TcpStatus checkTCPstatus(unsigned long timeoutS = 15) {
  unsigned long start = millis();
  TcpStatus tcpStatus;

  while (millis() - start < timeoutS*1000) {
    tcpStatus = getTCPStatus();

    if (tcpStatus == TcpStatus::CONNECT_OK) return TcpStatus::CONNECT_OK;
    if (tcpStatus == TcpStatus::CLOSED) return TcpStatus::CLOSED;

    delay(300);  // dont do anything on any other statues wait a bit and ask again
  }

  return tcpStatus; // last known state after timeout
}


String sendTCPData(const String& dataToSend, bool waitForReply=true) {
  Serial_println("\r\n>> Sending TCP data:");

  TcpStatus tcpStatus = checkTCPstatus(); // TODO make this waiting for tcp status configurable
  if(tcpStatus == TcpStatus::CLOSED) {
    elog.log(ErrorLogger::ERR_TCP_CONN_CLOSED);
    Serial_print("TCP status closed, failed to send");
    return "";
  }

  if(tcpStatus != TcpStatus::CONNECT_OK) {
    elog.log(ErrorLogger::ERR_TCP_CONN_NOT_OK);
    Serial_print("TCP not connect ok, failed to send");
    return "";
  }
 
  String response = sendCommand("AT+CIPSEND", 1000, "> ");
  if(!response.endsWith("> ")) {
    Serial_print("Got wrong response from CIPSEND. Response: '");  Serial_print(response);  Serial_println("'");   
    elog.log(ErrorLogger::ERR_TCP_CIPSEND);
    return "";
  }
  
  Serial_print("Ready to send ");  Serial_print(dataToSend.length());  Serial_println(" bytes");   
  Serial_print("Sending: '");  Serial_print(dataToSend);  Serial_println("'"); 
  sendCommand(dataToSend + "\x1A");
  Serial_println("Done sending!");

  if(!waitForReply) {
    return "";
  }

  // TODO make 10 seconds wait time from server configurable
  //delay(1000);
  if(!waitForResponse("AT+CIPRXGET=4", 10, parseCIPRXGET4, 300)) {
    elog.log(ErrorLogger::ERR_TCP_NO_SRV_RESPONSE);
    return "";
  }

  response = sendCommand("AT+CIPRXGET=2,256", 2000);
  int params_int[4]; 
  int params_cnt = 0;
  parseParamsResponse(response, "+CIPRXGET: ", params_int, 4, params_cnt);

  return response;
}


bool sendTCP_break_into_packets(const String& data) {
  int offset = 0;
  int pos = 0;
  const int packetSize = constrain((int)prefs.tcp_packet_size, TCP_PACKET_SIZE_MIN, TCP_PACKET_SIZE_MAX);

  while (pos < data.length()) {
      int len = min(packetSize, ((int) data.length()) - pos);
      String packet = data.substring(pos, pos + len);
      
      String response = sendTCPData(packet);

      if(response == "") return false;

      int pckStart = response.indexOf("GOT|");
      if(pckStart == -1) {
        elog.log(ErrorLogger::ERR_TCP_NO_GOT_RESPONSE);
        Serial_println("Failed to get GOT from the response. Error sending!");
        return false;
      }

      int numEnd = response.indexOf("|", pckStart+4);
      int srvRecievedLen = response.substring(pckStart+4, numEnd).toInt();

      if(srvRecievedLen != packet.length()) {
        elog.log(ErrorLogger::ERR_TCP_WRONG_RESPONSE_LEN);
        Serial_println("Failed sneding TCP packet not matching len from the server!");
        return false;
      }

      Serial_print("Srver response: '"); Serial_print(response); Serial_println("'");
      pos += len;
      offset += len;
  }

  return true;
}


void sendStreamTCP(int durationSec) {
  enableErrorLedBlinking = false;

  uint32_t start = millis();
  Serial_println("Starting stream sending for " + String(durationSec) + " seconds...");
  while(millis() - start < durationSec*1000) {
    digitalWrite(ERROR_LED_PIN, HIGH); 

    String streamData = 
                  "ts=" + String(get_log_timestamp()) + 
                  "&spd=" + String(lastSpeedRead) + 
                  "&dir=" + String(last_direction_read) +
                  "&bat=" + String(read_batt_v(), 2);

    const bool waitForResponse = false;
    sendTCPData("stream|" + streamData + "|done", waitForResponse);
    
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(500);
  }

  enableErrorLedBlinking = true;
}

bool sendOverTCP() {
  String postBodyData = sendShortMessage? getPostBodyShort() : getPostBody();
  sendShortMessage = false; // reset the flag after sending

  const String dataPacket = "data|" + imsiNum + "|" + String(postBodyData.length()) + "|" + postBodyData + "|done";
  if(!sendTCP_break_into_packets(dataPacket)) {
    elog.log(ErrorLogger::ERR_TCP_SEND_DATA);
    Serial_println("Failed to send data");
    return false;
  }

  String prefsResponse = sendTCPData("gotprefs?|done");
  Serial_print("Got preferences response: '"); Serial_print(prefsResponse); Serial_println("'");
  parseReturnData(prefsResponse);

  if(shouldSendPrefs) {
    Serial_println();
    Serial_println("Sending preferences");

    String prefsBody = getPostBodyPrefsAll();
    String prefsPacket = "prefs|" + String(prefsBody.length()) + "|" + prefsBody + "|done";
    if(!sendTCP_break_into_packets(prefsPacket)) {
      elog.log(ErrorLogger::ERR_TCP_SEND_PREFS);
      return false;
    }
  }

  if(shouldSendErrorNames) {
    Serial_println();
    Serial_println("Sending error names");

    String errorsBody = ErrorLogger::getPostErrorsList();
    String errorsPacket = "errors|" + String(errorsBody.length()) + "|" + errorsBody + "|done";
    if(!sendTCP_break_into_packets(errorsPacket)) {
      elog.log(ErrorLogger::ERR_TCP_SEND_ERRORS);
      return false;
    }
  }

  if(send_stream_for_s > 0) {
    sendStreamTCP(send_stream_for_s);
  }
  send_stream_for_s = 0;

  const bool waitForReply = false;
  sendTCPData("END", waitForReply); // send the server so that it know to close the connection
  sendCommand("AT+CIPCLOSE=1", 1000); // close the connection 
  Serial_print("End sending over TCP. Success!");

  return true;
}
