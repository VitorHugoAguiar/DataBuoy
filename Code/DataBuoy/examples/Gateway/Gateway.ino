// Includes
#include "SharedFunctions.h"
#include "GatewayFunctions.h"

SharedFunctions sf_gw;
GatewayFunctions gwf;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int WiFiRSSI = -100;                                  // Above this RSSI value, it's not worth connecting to the WiFi Network
const char* StrongSSID;                               // Choosen WiFi SSID based on the Network White List
const char* SSIDPassword;                             // Choosen Password based on the Network White List
int retryWiFi;                                        // Variable used to store the number of attempts to connect to a WiFi Network

// Tasks
void TaskNTP( void *pvParameters );
void TaskLoRaReceiver( void *pvParameters );
void TaskGatewayHeartBeat( void *pvParameters );
TaskHandle_t xNTP;
TaskHandle_t xLoRaReceiver;
TaskHandle_t xGatewayHeartBeat;

//##############################################################################
boolean SerialDebug = true;

// WiFi White List. The WiFI will try to connect to one of thesse Networks. If more than one Nework in the list is found,
// WiFi connects to the one with best signal (RSSI)
const char* const WiFiWhiteList[][2] = {
  {"SSID1", "PASSWORD1"},
  {"SSID2", "PASSWORD2"}
};

byte Gateway_ID = 0xAA;                     // Gateway ID

// DataBuoy White List. The Gateway only receives data one of the DataBuoys IDs on the list
int DataBouyIDWhiteList[] = {1, 2, 3, 4};   // int until 99

const boolean SendServer = true;            // Enables sending data to a server by http
const boolean SendNodeRed = true;           // Enables sending data to a Node Red server

// MQTT server
char mqtt_server[] = "***.***.***.***";       // MQTT broker IP Address
int mqtt_port = ****;                         // MQTT broker port

//##############################################################################


// WiFi Events.
// This function is called when there is a successful connection to a WiFi Network
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println(F("\n\u21D2 Successful connection to a WiFi network!"));
  Serial.println("  \u00BB Connected to: " + String(StrongSSID));
  retryWiFi = 0;
}

// This function shows the IP address
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print(F("  \u00BB IP address: "));
  Serial.println(WiFi.localIP());
}

// This function is called when a connection to a WiFi Network is lost
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  sf_gw.displayLED(CRGB::Red, 255);
  Serial.print(F("\n\u21D2 WiFi not connected. Reason: "));
  Serial.println(info.disconnected.reason);
  Serial.println(F("  \u00BB Trying to Reconnect"));
  WiFi.begin(StrongSSID, SSIDPassword);

  // Increase the number os attempts to connect to the WiFi Network
  retryWiFi++;

  // When has passed 5min, the ESP restarts to find a new network to connect
  if (retryWiFi == 300) {
    ESP.restart();
  }
  delay(1000);

}

void scanWiFiNetworks() {

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();

  if (n == 0) {
    Serial.println(F("\n\u21D2 No networks found!"));
  } else {
    Serial.print(F("\n\u21D2 "));
    Serial.print(n);
    Serial.println(F(" networks found"));
    for (int i = 0; i < n; ++i) {
      Serial.print(F("  \u00BB "));
      Serial.print(i + 1);
      Serial.print(F(": "));
      Serial.print(WiFi.SSID(i));
      Serial.print(F(" ("));
      Serial.print(WiFi.RSSI(i));
      Serial.println(F(")"));
      delay(10);

      // Checks if the Networks found are part of the white list and connects to the one with the best signal
      for (int k = 0; k < ARRAY_SIZE(WiFiWhiteList); k++) {
        if (WiFi.SSID(i) == WiFiWhiteList[k][0]) {
          if (WiFi.RSSI(i) > WiFiRSSI) {
            StrongSSID = WiFiWhiteList[k][0];
            SSIDPassword = WiFiWhiteList[k][1];
            WiFiRSSI = WiFi.RSSI(i);
          }
        }
      }
    }
  }

  WiFi.begin(StrongSSID, SSIDPassword);
  // Max WiFi trasmission power for best connection
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

void setupWiFi() {

  // Delete old config
  WiFi.disconnect(true);

  delay(1000);

  // Start the WiFI events
  WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);

  // Checks the WiFi Networks available
  scanWiFiNetworks();
}

void TaskGatewayHeartBeat(void *pvParameters) {
  (void) pvParameters;

  TickType_t xTimerInterval;
  const TickType_t xFrequencyInterval = 15000;  // Time between readings

  xTimerInterval = xTaskGetTickCount ();

  for (;;) // A Task shall never return or exit.
  {

    // If the trasmission to Server (by http) is enable, creates the corresponding Gateway HeartBeat JSON
    if (SendServer) {

      gwf.sendPOST(gwf.createHeartBeatJSON(Gateway_ID));

    }

    // If the trasmission to Node Red (by MQTT) is enable, creates the corresponding Gateway HeartBeat JSON
    if (SendNodeRed) {
      gwf.start_MQTT(mqtt_server, mqtt_port);

      // Gateway HeartBeat with the Gateway ID
      String Gateway_ID_String = String(Gateway_ID, HEX);
      Gateway_ID_String.toUpperCase();

      String gateway_heartbeat = "Gateway" + Gateway_ID_String + "_HeartBeat";
      char gateway_heartbeat_char[gateway_heartbeat.length() + 1];
      gateway_heartbeat.toCharArray(gateway_heartbeat_char, gateway_heartbeat.length() + 1);

      gwf.sendMQTT(gateway_heartbeat_char, "connected");
    }

    vTaskDelayUntil( &xTimerInterval, xFrequencyInterval);
  }
  vTaskDelete( NULL );
}

void TaskNTP(void *pvParameters) {
  (void) pvParameters;

  TickType_t xNTPInterval;

  const TickType_t xFrequencyInterval = 1000;  // Time between readings

  xNTPInterval = xTaskGetTickCount ();

  for (;;) // A Task shall never return or exit.
  {

    // Updates the "internal" timer
    gwf.updateTimeStamp();

    vTaskDelayUntil( &xNTPInterval, xFrequencyInterval);
  }
  vTaskDelete( NULL );
}

void TaskLoRaReceiver(void *pvParameters) {
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    int packetSize = LoRa.parsePacket();

    // Checks if received a new LoRa packet
    if (packetSize != 0) {

      // Decodes the new LoRa packet
      gwf.onReceive(packetSize, Gateway_ID, DataBouyIDWhiteList, ARRAY_SIZE(DataBouyIDWhiteList), SendServer, SendNodeRed); 

      vTaskDelay( 100 / portTICK_PERIOD_MS);

    }
    vTaskDelay( 1 / portTICK_PERIOD_MS);
  }
  vTaskDelete( NULL );
}

void setup() {

  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  // Initialize the gateway setup with all the needed configs
  gwf.setup(SerialDebug);
  // Initialize the gateway shared configs
  sf_gw.setup();

  Serial.println(F("\n################################"));
  Serial.println(F("##                            ##"));
  Serial.println(F("##   DataBuoy LoPy4 Gateway   ##"));
  Serial.println(F("##                            ##"));
  Serial.println(F("################################"));

  Serial.println(F("\n################################"));
  Serial.println(F("##        Main Settings       ##"));
  Serial.println(F("################################\n"));

  int cpuSpeed = getCpuFrequencyMhz();
  Serial.println("\u21D2 CPU Frequency: " + String(cpuSpeed) + "MHz");

  Serial.println(F("\n################################"));
  Serial.println(F("##  Components initialization ##"));
  Serial.println(F("################################"));

  sf_gw.displayLED(CRGB::Red, 255);
  sf_gw.start_LoRa();     // Initialize LoRa with the choosen settings
  sf_gw.start_SDCard();   // Initialize SD Card Reader

  setupWiFi(); // Initialize WiFi Network scan and connection

  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  gwf.start_NTP(3600);  // Sets the offset time

  // If the Node Red is enable, initialize MQTT
  if (SendNodeRed) {
    gwf.start_MQTT(mqtt_server, mqtt_port);
  }

  sf_gw.displayLED(CRGB::Green, 255);

  Serial.println(F("\n################################"));
  Serial.println(F("##     Waiting for data...    ##"));
  Serial.println(F("################################\n"));

  // Setup up Tasks and where to run ============================================================
  // Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskLoRaReceiver
    ,  "TaskLoRaReceiver"
    ,  15000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  &xLoRaReceiver
    ,  TaskCore0);

  xTaskCreatePinnedToCore(
    TaskNTP
    ,  "TaskNTP"
    ,  15000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  &xNTP
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskGatewayHeartBeat
    ,  "TaskGatewayHeartBeat"
    ,  15000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  &xGatewayHeartBeat
    ,  TaskCore1);
}

void loop() {
  // Empty. Things are done in Tasks.
  /*--------------------------------------------------*/
  /*---------------------- Tasks ---------------------*/
  /*--------------------------------------------------*/
}
