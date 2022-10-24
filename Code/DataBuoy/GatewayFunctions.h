#ifndef gf_h
#define gf_h

// Includes
#include <WiFi.h>
#include <LoRa.h>
#include <CayenneLPP.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "SD_MMC.h"

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])                // Returns an array size
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class GatewayFunctions {
  public:
    void setup(boolean SerialDebug);
    void start_MQTT(char* mqtt_server, int mqtt_port);
    void start_NTP(int GMT);
    void reconnect_MQTT();
    void update_NTP();
    void updateTimeStamp();
    String listDir(const char * dirname, byte DataBuoy_ID);
    void CreateFile(char filename[]);
    void WriteFileHeaders(String* Fields, int Fields_Size, String* Fields_Units);
    void CSV_File(byte DataBuoy_ID, String* Fields, int Fields_Size, String* Fields_Units, String* LoRa_Values, int LoRa_Values_Size);
    bool onReceive(int packetSize, byte Gateway_ID, int* DataBouyIDWhiteList, int DataBouyIDWhiteListSize, boolean SendServer, boolean SendNodeRed);
    byte GetDataBuoyDeviceNum(byte DataBuoy_ID);
    byte GetGatewayDeviceNum(byte Gateway_ID);
    void sendPOST(String postBody);
    void sendMQTT(char topic[], char message[]);
    String createLoRaJSON(byte DataBuoy_ID, String* Fields, int Fields_Size, String* LoRa_Values, float LoRa_RSSI, float LoRa_SNR, byte Gateway_ID);
    String createLoRaJSON_MQTT(String* Fields, int Fields_Size, String* LoRa_Values, float LoRa_RSSI, float LoRa_SNR);
    String createHeartBeatJSON(byte Gateway_ID);

  private:
    boolean _SerialDebug;
    String _DayFolder;
    uint8_t _hour_int_prev, _hour_int, _minute_int, _second_int;
    String _dayStamp, _timeStamp;
    String _formattedDate;
};

#endif
