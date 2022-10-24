#ifndef WaveBuoyFunctions_h
#define WaveBuoyFunctions_h

//Libraries
#include "Arduino.h"
#include <WiFi.h>
#include <EEPROM.h>
#include <RtcDS3231.h>
#include <CayenneLPP.h>

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

#include "MPU9250.h"
#include "timestamp32bits.h"

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <HardwareSerial.h>
#include <driver/adc.h>
#include <Adafruit_BMP280.h>
#include <MadgwickAHRS.h>
#include <filters.h>

// Gps module pins
#define GPS_Reset   22
#define GPS_ONOFF   21
#define GPS_WakeUP  35
#define GPS_Tx      33
#define GPS_Rx      32

//MPU9250 settings
#define MPU9250_ADDRESS 0x69    // Default is 0x68 but it's being used by the RTC
#define FSR SMPL_250HZ          // SMPL_1000HZ, SMPL_500HZ, SMPL_333HZ, SMPL_250HZ, SMPL_200HZ, SMPL_167HZ, SMPL_143HZ, SMPL_125HZ
//Acc
#define AFS A16G                 // A2G, A4G, A8G, A16G
#define AFC 0x03
#define ADLPF DLPF_420HZ         // DLPF_218HZ_0, DLPF_218HZ_1, DLPF_99HZ, DLPF_45HZ, DLPF_21HZ, DLPF_10HZ, DLPF_5HZ, DLPF_420HZ,
//Gyro
#define GFS G250DPS             // G250DPS, G500DPS, G1000DPS, G2000DPS
#define GFC 0x03
#define GDLPF DLPF_41HZ         // DLPF_250HZ, DLPF_184HZ, DLPF_92HZ, DLPF_41HZ, DLPF_20HZ, DLPF_10HZ, DLPF_5HZ, DLPF_3600HZ    
//AK8963 settings
#define MOB M16BITS             // M14BITS, M16BITS

#define TurnSensorsOn 25

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0]) // Function used to get an array size
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class WaveBuoyFunctions {
  public:
    // Methods
    void setup(boolean SerialDebug, boolean SleepMode, uint8_t WaveBuoy_ID, uint8_t SampleRate);
    void start_Madgwick_Filter(uint8_t SampleRate);
    void start_MAX17048();
    void start_GPS();
    void start_RTC(uint8_t StartTimeArray[6], uint8_t AverageTimeArray[3], uint8_t SampleLengthMinutes, uint8_t* Alarm_WakeUp_Minutes, int Alarm_WakeUp_Minutes_Size);
    void start_MPU9250();
    void start_BMP280();
    bool checkLeapYear(uint8_t the_year);
    void updateTimeStamp();
    void GetData(float WaveHeightThreshold, float Gravity_Acceleration, float Magnetic_Declination);
    void initLowPassFilterAccZ (float LPCutOffFreqAccZ, uint8_t SampleRate);
    void initHighPassFilterVelocity (float HPCutOffFreqVelocity, uint8_t SampleRate);
    void initHighPassFilterDisplacement (float HPCutOffFreqDisplacement, uint8_t SampleRate);
    void init_ulp_program(void);
    void EEPROM_WRITE(float dataToStore, unsigned int address);
    float EEPROM_READ(unsigned int address);
    void SendLoRaPacket (String payload);
    void decode41(byte payload[], boolean SleepModeGPS);
    void GetGPSData(boolean SleepModeGPS);
    void CSV_File(boolean GetNewValue);
    void ParseDataCSV (String* CSV_Values_First, int CSV_Values_First_Size, String* CSV_Values_Second, int CSV_Values_Second_Size,  float* CSV_Values_Minute, int CSV_Values_Minute_Size, float* CSV_Values_Always, int CSV_Values_Always_Size, float GPSData[2]);
    void WriteSD();

  private:
    boolean _SerialDebug;
    boolean _SleepMode;
    boolean _justOnce;
    int _NumberOfCycles;
    uint8_t _WaveBuoy_ID;
    uint8_t _SampleRate;
    uint8_t _DayNow, _MonthNow, _YearNow;
    uint8_t _HourNow, _MinuteNow, _SecondNow;
    uint8_t _HourNowPrev, _MinuteNowPrev, _SecondNowPrev;
    uint8_t _StartTimeArray[6];
    uint8_t _AverageTimeArray[3];
    int _SamplesCount;
    int _SamplesCountToWrite;
    int _TotalCount;
    uint8_t _SampleLengthMinutes;
    boolean _FirstSecondDetected;
    int _indexIMU, _indexWaitGPS;
    float _WaveDirection_AVG, _Height_Average_AVG, _WavePeriod_AVG;
    float _filtered_LinAccZ_prev;
    float _velocity;
    float  _filtered_velocity_prev;
    float _displacement;
    float _peak, _index_peak, _previous_peak;
    float _maximum, _minimum;
    float _height_prev;
    float _Height_Average_Samples[1000];
    int _samplesPeak;
    uint8_t _GPS_NO_DATA;
};

#endif
