#include "SharedFunctions.h"
#include "GatewayFunctions.h"

SharedFunctions sf_gwcpp;

// Objects
WiFiClient Lopy4Client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
PubSubClient client(Lopy4Client);
File logFileWriteGW;
CayenneLPP lppGW(4096);

time_t prevTimeStampGW = 0;                  // Stores the previous timestamp

// Initialize Gateway configs
void GatewayFunctions::setup(boolean SerialDebug) {
  _SerialDebug = SerialDebug;
}

// Initialize MQTT
void GatewayFunctions::start_MQTT(char* mqtt_server, int mqtt_port) {

  client.setServer(mqtt_server, mqtt_port); // Set server address and port
  client.setKeepAlive(0);                   // Mechanism for checking the status of the TCP/IP connection
  client.setBufferSize(500);                // Max mqtt message

  // Waits until gets a connection with the server
  if (!client.connected()) {
    GatewayFunctions::reconnect_MQTT();
    client.loop();
  }
}

// Initialize NTP
void GatewayFunctions::start_NTP(int GMT) {
  // Initialize a NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(GMT);
  GatewayFunctions::update_NTP();
}

// Reconnects to MQTT server
void GatewayFunctions::reconnect_MQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    sf_gwcpp.displayLED(CRGB::Red, 255);
    Serial.println(F("\n\u21D2 Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect("Lopy4Client")) {
      sf_gwcpp.displayLED(CRGB::Green, 255);
      Serial.println(F("  \u00BB connected to broker..."));
    } else {
      sf_gwcpp.displayLED(CRGB::Red, 255);
      Serial.print(F("  \u00BB failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 1 seconds"));
      // Wait 1 second before retrying
      delay(1000);
    }
  }
}

// Gets NTP timestamp
void GatewayFunctions::update_NTP() {
  // Updates the time through NTP
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }

  _formattedDate = timeClient.getFormattedDate();   // Gets formatted Date

  int splitT = _formattedDate.indexOf("T");
  _dayStamp = _formattedDate.substring(0, splitT);  // Extract date

  _timeStamp = timeClient.getFormattedTime();        // Gets formatted Time

  // Extract time
  _hour_int = _timeStamp.substring(0, 2).toInt();
  _minute_int = _timeStamp.substring(4, 6).toInt();
  _second_int = _timeStamp.substring(8, 10).toInt();

  // Stores prev hour
  _hour_int_prev = _hour_int;

  _DayFolder = "/" + _dayStamp;

  // Checks if the day folder already exists
  if (!SD_MMC.exists(_DayFolder)) {
    if (!SD_MMC.mkdir(_DayFolder)) {
      Serial.println(F("  \u00BB Create DayFolder failed!"));
      sf_gwcpp.fatalBlink(CRGB::Red);
    }
  }

  // Extract date to specific variables
  uint8_t day_ = _dayStamp.substring(0, 2).toInt();
  uint8_t month_ = _dayStamp.substring(4, 6).toInt();
  uint8_t year_ = _dayStamp.substring(8, 10).toInt();

  setTime(_hour_int, _minute_int, _second_int, day_, month_, year_); // Updates the "internal" timer
}

// Updates the "internal" timer
void GatewayFunctions::updateTimeStamp() {

  if (timeStatus() != timeNotSet) {
    if (now() != prevTimeStampGW) {     // Checks if the timestamp has changed

      _hour_int = hour();
      _minute_int = minute();
      _second_int = second();

      prevTimeStampGW = now();
    }
  }
}

// Creates the Day Folder
String GatewayFunctions::listDir(const char * dirname, byte DataBuoy_ID) {

  String FileExistDirec;

  File root = SD_MMC.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return FileExistDirec;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return FileExistDirec;
  }

  // Checks if there is already a file with the DataBuoy_ID. If already exists, re-open.
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {

      String FileNameString = String(file.name()).substring(0, String(file.name()).length());
      FileNameString.remove((String(file.name()).length() - (_timeStamp.length() + 5)), String(file.name()).length());

      String FileNameDataBuoyID = FileNameString.substring(20, FileNameString.length());

      if (FileNameDataBuoyID.toInt() == DataBuoy_ID) {
        FileExistDirec = String(file.name()).substring(0, String(file.name()).length());
      }
    }
    file = root.openNextFile();
  }
  return FileExistDirec;
}

// Creates File
void GatewayFunctions::CreateFile(char filename[]) {

  // Create file and prepare it ============================================================
  logFileWriteGW = SD_MMC.open(filename, FILE_WRITE);
  if (!logFileWriteGW ) {
    Serial.print(F("  \u00BB Could not create..."));
    Serial.println(filename);
    sf_gwcpp.fatalBlink(CRGB::Red);
  }

  Serial.print(F("\u21D2 Ready to write to:"));
  Serial.println(filename);
}

// Write File Headers
void GatewayFunctions::WriteFileHeaders(String* Fields, int Fields_Size, String* Fields_Units) {

  // Column headers
  logFileWriteGW.print("DataBuoy_ID");
  for (size_t i = 0; i < Fields_Size; i++) { // Function used to write the csv file headers
    logFileWriteGW.print(",");
    if (Fields[i] == "Epoch_Time") {
      logFileWriteGW.print("Hour");
    } else {
      logFileWriteGW.print(Fields[i] + " " + Fields_Units[i]);
    }
  }

  logFileWriteGW.print("\n");
  logFileWriteGW.close();
}

// Checks if new file, if necessary
void GatewayFunctions::CSV_File(byte DataBuoy_ID, String* Fields, int Fields_Size, String* Fields_Units, String* LoRa_Values, int LoRa_Values_Size) {

  char filename[50];
  String FileDir;

  // If the hour changes, creates a new CSV file
  if (_hour_int != _hour_int_prev) {
    GatewayFunctions::update_NTP();

    FileDir = _DayFolder + "/DataBuoy" + String(DataBuoy_ID) + "_" + _timeStamp + ".csv";
    FileDir.toCharArray(filename, 50);

    if (SD_MMC.exists(filename)) {
      Serial.println(F("  \u00BB A file with that name already exists"));
      sf_gwcpp.fatalBlink(CRGB::Red);
    }

    GatewayFunctions::CreateFile(filename);                                 // Creates new file
    GatewayFunctions::WriteFileHeaders(Fields, Fields_Size, Fields_Units);  // Writes headers
  }

  _hour_int_prev = _hour_int;

  char DayFolderChar[50];
  _DayFolder.toCharArray(DayFolderChar, 50);
  String FileExits = GatewayFunctions::listDir(DayFolderChar, DataBuoy_ID);

  if (FileExits.length() == 0) {

    GatewayFunctions::update_NTP();

    FileDir = _DayFolder + "/DataBuoy" + String(DataBuoy_ID) + "_" + _timeStamp + ".csv";
    FileDir.toCharArray(filename, 50);

    if (SD_MMC.exists(filename)) {
      Serial.println(F("  \u00BB A file with that name already exists"));
      sf_gwcpp.fatalBlink(CRGB::Red);
    }

    GatewayFunctions::CreateFile(filename);
    GatewayFunctions::WriteFileHeaders(Fields, Fields_Size, Fields_Units);

  } else {
    FileExits.toCharArray(filename, FileExits.length() + 1);
  }

  // Opens the CSV file in append mode
  logFileWriteGW = SD_MMC.open(filename, FILE_APPEND);

  // Writes the values into the CSV File
  logFileWriteGW.print(DataBuoy_ID);
  logFileWriteGW.print(',');
  for (size_t i = 0; i < LoRa_Values_Size; i++) {
    if (i == (LoRa_Values_Size - 1)) {
      logFileWriteGW.print(LoRa_Values[i]);
      logFileWriteGW.print("\n");
    } else {
      if (i == 0) {
        unsigned long t = LoRa_Values[0].toInt();
        char buff[10];
        sprintf(buff, "%02d:%02d:%02d", hour(t), minute(t), second(t));
        logFileWriteGW.print(buff);
      } else {
        logFileWriteGW.print(LoRa_Values[i]);
      }
      logFileWriteGW.print(',');
    }
  }
  logFileWriteGW.close();
}

// Function thar receives the data from the DataBuoys
bool GatewayFunctions::onReceive(int packetSize, byte Gateway_ID, int* DataBouyIDWhiteList, int DataBouyIDWhiteListSize, boolean SendServer, boolean SendNodeRed) {

  if (packetSize == 0) {
    return false;                             // If there's no packet, return
  }

  // read packet header bytes:
  byte DataBuoy_ID = LoRa.read();             // Sender DataBuoy ID
  byte incomingLength = LoRa.read();          // Incoming msg length

  String payloadString;

  // LoRa Packet
  while (LoRa.available()) {
    payloadString += (char)LoRa.read();
  }

  if (incomingLength != payloadString.length()) {   // Check length for error
    return false;                                   // Skip rest of function
  }

  // Checks if the DataBuoy_ID belongs to the WhiteList
  boolean DataBouyIDAccepted  = false;
  for (int i = 0; i < DataBouyIDWhiteListSize; i++) {
    if (DataBouyIDWhiteList[i] == DataBuoy_ID) {
      DataBouyIDAccepted = true;
    }
  }

  if (!DataBouyIDAccepted) {
    return false;
  }

  float LoRa_RSSI = LoRa.packetRssi();        //Received Signal Strength Indication (RSSI): -120db:weak, -30db:strong
  float LoRa_SNR = LoRa.packetSnr();          //Signal to Noise Ratio (SNR): -20db: more corrupted, 10db: less corrupted

  // Blinks the LED when receives data from an expected DataBuoy
  sf_gwcpp.displayLED(CRGB::Blue, 255);
  vTaskDelay( 200 / portTICK_PERIOD_MS);
  sf_gwcpp.displayLED(CRGB::Black, 0);
  vTaskDelay( 100 / portTICK_PERIOD_MS);
  sf_gwcpp.displayLED(CRGB::Green, 255);

  uint8_t PayloadBuffer [incomingLength];

  for (int i = 0; i < incomingLength; i++) {
    PayloadBuffer[i] = (uint8_t) payloadString[i];
  }

  // Decodes the Cayenne LPP Packet
  DynamicJsonDocument jsonBuffer(4096);
  JsonArray doc = jsonBuffer.to<JsonArray>();

  lppGW.decode(PayloadBuffer, incomingLength, doc);

  int NumberFields = 0;
  for (JsonVariant v : doc) {
    if (v["name"].as<String>() == "GPS") {
      NumberFields++;
    }
    NumberFields++;
  }

  String LoRa_Values[NumberFields];
  String Fields[NumberFields];
  String Fields_Units[NumberFields];

  // Stores DataBuoy Data on an array
  int k = 0;
  for (JsonVariant v : doc) {
    if (v["name"].as<String>() != "GPS") {
      Fields[k] = v["name"].as<String>();
      if (v["error_data"].as<String>() == "0") {
        if (v["value"].as<String>() != "null") {
          LoRa_Values[k] = v["value"].as<String>();
        } else {
          LoRa_Values[k] = "NULL";
        }
      } else {
        LoRa_Values[k] = "NO DATA";
      }
      if (v["typeUnit"].as<String>() != "null") {
        Fields_Units[k] = v["typeUnit"].as<String>();
      } else {
        Fields_Units[k] = "";
      }
    } else {

      if (v["error_data"].as<String>() == "0") {
        LoRa_Values[k] = v["value"]["latitude"].as<String>();
        LoRa_Values[k + 1] = v["value"]["longitude"].as<String>();
      } else {
        LoRa_Values[k] = "NO DATA";
        LoRa_Values[k + 1] = "NO DATA";
      }

      Fields[k] = "Latitude";
      Fields_Units[k] = v["typeUnit"].as<String>();

      Fields[k + 1] = "Longitude";
      Fields_Units[k + 1] = v["typeUnit"].as<String>();
    }
    k++;
  }

  // Checks if is necessary to create a new file
  GatewayFunctions::CSV_File(DataBuoy_ID, Fields, ARRAY_SIZE(Fields), Fields_Units, LoRa_Values, ARRAY_SIZE(LoRa_Values));

  // Sends data to server (HTTP)
  if (SendServer) {
    String jsonStrTotalServer = GatewayFunctions::createLoRaJSON(DataBuoy_ID, Fields, ARRAY_SIZE(Fields), LoRa_Values, LoRa_RSSI, LoRa_SNR, Gateway_ID);

    if (_SerialDebug) {
      Serial.println("\n\u21D2 jsonStrTotalServer: " + jsonStrTotalServer);
    }

    GatewayFunctions::sendPOST(jsonStrTotalServer);
  }

  // Sends data to server (Node Red)
  if (SendNodeRed) {
    // MQTT Topic with the DataBuoy_ID
    String each_mqtt_topic = "DataBuoy" + String(DataBuoy_ID) + "/Telemetry";
    char mqtt_topic_char[each_mqtt_topic.length() + 1];
    each_mqtt_topic.toCharArray(mqtt_topic_char, each_mqtt_topic.length() + 1);

    String jsonStrTotalNodeRed = GatewayFunctions::createLoRaJSON_MQTT(Fields, ARRAY_SIZE(Fields), LoRa_Values, LoRa_RSSI, LoRa_SNR);

    char mqtt_json [jsonStrTotalNodeRed.length() + 1];
    jsonStrTotalNodeRed.toCharArray(mqtt_json, jsonStrTotalNodeRed.length() + 1);

    // Publish the json into the MQTT Topic
    GatewayFunctions::sendMQTT(mqtt_topic_char, mqtt_json);

    if (_SerialDebug) {
      Serial.println("\n\u21D2 jsonStrTotalNodeRed: " + jsonStrTotalNodeRed);
    }
  }
}

// Gets the respective DataBuoy device number to the server (HTTP)
byte GatewayFunctions::GetDataBuoyDeviceNum(byte DataBuoy_ID) {

  byte DataBuoyDeviceNum;

  switch (DataBuoy_ID) {
    case 1:
      DataBuoyDeviceNum = 12;
      break;
    case 2:
      DataBuoyDeviceNum = 18;
    case 3:
      DataBuoyDeviceNum = 41;
      break;
    default:
      sf_gwcpp.fatalBlink(CRGB::Yellow);
      break;
  }

  return DataBuoyDeviceNum;
}

// Gets the respective Gateway device number to the server (HTTP)
byte GatewayFunctions::GetGatewayDeviceNum(byte Gateway_ID) {

  byte GatewayDeviceNum;

  switch (Gateway_ID) {
    case 0xAA:
      GatewayDeviceNum = 16;
      break;
    case 0xBB:
      GatewayDeviceNum = 17;
      break;
    default:
      sf_gwcpp.fatalBlink(CRGB::Yellow);
      break;
  }

  return GatewayDeviceNum;
}

// Function that sends data to the server (HTTP)
void GatewayFunctions::sendPOST(String postBody) {
  HTTPClient httpClient;
  int jsonSize = postBody.length();

  httpClient.begin("http://wave-labs.org/api/iot-report");

  httpClient.addHeader("Content-Type", "application/json");
  int response = httpClient.POST(postBody);

  httpClient.end();

  if (_SerialDebug) {
    if (response == 201) {
      Serial.println("  \u00BB Server Response: OK!");
    } else {
      Serial.println("  \u00BB Server Response: NOT OK! -> " + String(response));
    }
  }
}

// Function that sends data to the server (Node Red)
void GatewayFunctions::sendMQTT(char topic[], char message[]) {
  client.publish(topic, message);
}

// Creates the JSON for the server (HTTP)
String GatewayFunctions::createLoRaJSON(byte DataBuoy_ID, String* Fields, int Fields_Size, String* LoRa_Values, float LoRa_RSSI, float LoRa_SNR, byte Gateway_ID) {

  String jsonStr;
  StaticJsonDocument<500> doc;

  doc["device"] = GatewayFunctions::GetDataBuoyDeviceNum(DataBuoy_ID);

  JsonObject fields = doc["fields"].createNestedObject();

  for (size_t i = 0; i < Fields_Size; i++) {
    fields[Fields[i]] = LoRa_Values[i];
    if (i == Fields_Size - 1) {
      fields["RSSI"] = LoRa_RSSI;
      fields["SNR"] = LoRa_SNR;
      fields["Gateway"] = GatewayFunctions::GetGatewayDeviceNum(Gateway_ID);
    }
  }

  jsonStr = "";
  serializeJson(doc, jsonStr);
  jsonStr = "[" + jsonStr + "]";

  return jsonStr;
}

// Creates the JSON for the server (MQTT)
String GatewayFunctions::createLoRaJSON_MQTT(String* Fields, int Fields_Size, String* LoRa_Values, float LoRa_RSSI, float LoRa_SNR) {

  // Creates the json that is going be send to the server
  String json;
  for (size_t i = 0; i < Fields_Size; i++) {
    if (i == 0) {
      if (LoRa_Values[i] == "NO DATA") {
        json = "{\"" + Fields[i] + "\":" + "\"NO DATA\"";
      } else {
        json = "{\"" + Fields[i] + "\":" + LoRa_Values[i];
      }
    } else {
      if (LoRa_Values[i] == "NO DATA") {
        json = json + ",\"" + Fields[i] + "\":" + "\"NO DATA\"";
      } else {
        json = json + ",\"" + Fields[i] + "\":" + LoRa_Values[i];
      }
      if (i == Fields_Size - 1) {
        json = json + ",\"RSSI\":" + String(LoRa_RSSI);
        json = json + ",\"SNR\":" + String(LoRa_SNR) + "}";
      }
    }
  }

  return json;
}

// Creates the HeartBeatJSON to send to the servers
String GatewayFunctions::createHeartBeatJSON(byte Gateway_ID) {

  String jsonStrHeartBeat;
  StaticJsonDocument<500> doc;

  doc["device"] = GatewayFunctions::GetGatewayDeviceNum(Gateway_ID);

  JsonObject fields = doc["fields"].createNestedObject();

  fields["active"] = true;

  jsonStrHeartBeat = "";
  serializeJson(doc, jsonStrHeartBeat);
  jsonStrHeartBeat = "[" + jsonStrHeartBeat + "]";

  if (_SerialDebug) {
    Serial.println("\n\u21D2 jsonStrHeartBeat: " + jsonStrHeartBeat);
  }

  return jsonStrHeartBeat;
}
