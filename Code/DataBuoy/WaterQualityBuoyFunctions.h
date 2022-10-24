#ifndef WaterQualityBuoyFunctions_h
#define WaterQualityBuoyFunctions_h

//Libraries
#include "Arduino.h"
#include <WiFi.h>
#include <RtcDS3231.h>
#include <CayenneLPP.h>
#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>

// Custom binary loader
#include "esp32/ulp.h"
#include "esp32-hal-cpu.h"
#include "esp_sleep.h"
#include "ulp_main.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "timestamp32bits.h"

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <HardwareSerial.h>
#include <MadgwickAHRS.h>
#include <filters.h>

// Gps module pins
#define GPS_Reset   22
#define GPS_ONOFF   21
#define GPS_WakeUP  35
#define GPS_Tx      33
#define GPS_Rx      32

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0]) // Function used to get an array size
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class WaterQualityBuoyFunctions {
  public:
    // Methods
    void setup(boolean SerialDebug, boolean SleepMode, uint8_t WaterQualityBuoy_ID, uint8_t SampleTime);
    void start_MAX17048();
    void start_GPS();
    void start_RTC(uint8_t StartTimeArray[6], uint8_t AverageTimeArray[3], uint8_t SampleLengthMinutes, uint8_t* Alarm_WakeUp_Minutes, int Alarm_WakeUp_Minutes_Size);
    void start_WhiteBoxT1();
    void GetDataWhiteBoxT1();
    String CheckError(uint8_t SensorResponse, float value);
    bool checkLeapYear(uint8_t the_year);
    void updateTimeStamp();
    void init_ulp_program(void);
    void SendLoRaPacket (String payload);
    void decode41(byte payload[], boolean SleepModeGPS);
    void GetGPSData(boolean SleepModeGPS);
    void CSV_File(boolean GetNewValue);
    void ParseDataCSV (String* CSV_Values_First, int CSV_Values_First_Size, String* CSV_Values_Second, int CSV_Values_Second_Size,  float* CSV_Values_Minute, int CSV_Values_Minute_Size, String* CSV_Values_Always, int CSV_Values_Always_Size, float GPSData[2]);
    void WriteSD();

  private:
    boolean _SerialDebug;
    boolean _SleepMode;
    boolean _justOnce;
    int _NumberOfCycles;
    uint8_t _WaterQualityBuoy_ID;
    uint8_t _SampleTime;
    uint8_t _DayNow, _MonthNow, _YearNow;
    uint8_t _HourNow, _MinuteNow, _SecondNow;
    uint8_t _HourNowPrev, _MinuteNowPrev, _SecondNowPrev;
    uint8_t _StartTimeArray[6];
    uint8_t _AverageTimeArray[3];
    int _SamplesCount;
    int _TotalCount;
    uint8_t _SampleLengthMinutes;
    boolean _FirstSecondDetected = false;
    uint8_t _GPS_NO_DATA;
    float _pH_AVG;
    float _DO_AVG;
    float _EC_AVG;
    float _ORP_AVG;
    float _RTD_AVG;
    int _index_ORP;
    int _index_RTD;
    int _index_pH;
    int _index_EC;
    int _index_DO;
    uint8_t _ORP_RESPONSE;
    uint8_t _RTD_RESPONSE;
    uint8_t _pH_RESPONSE;
    uint8_t _EC_RESPONSE;
    uint8_t _DO_RESPONSE;
    uint8_t _pH_NO_DATA;
    uint8_t _DO_NO_DATA;
    uint8_t _EC_NO_DATA;
    uint8_t _ORP_NO_DATA;
    uint8_t _RTD_NO_DATA;
};

#endif
