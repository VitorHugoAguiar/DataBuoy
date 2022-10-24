// Adapted from https://developer.mbed.org/teams/myDevicesIoT/code/Cayenne-LPP/

// Copyright © 2017 The Things Network
// Use of this source code is governed by the MIT license that can be found in the LICENSE file.

#ifndef CAYENNE_LPP_H
#define CAYENNE_LPP_H

#include <Arduino.h>
#include <ArduinoJson.h>

#define LPP_DIGITAL_INPUT               0     // 1 byte
#define LPP_DIGITAL_OUTPUT              1     // 1 byte
#define LPP_ANALOG_INPUT                2     // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT               3     // 2 bytes, 0.01 signed
#define LPP_GENERIC_SENSOR              100   // 4 bytes, unsigned
#define LPP_LUMINOSITY                  101   // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE                    102   // 1 byte, bool
#define LPP_TEMPERATURE                 103   // 2 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY           104   // 1 byte, 0.5% unsigned
#define LPP_ACCELEROMETER               113   // 2 bytes per axis, 0.001G
#define LPP_BAROMETRIC_PRESSURE         115   // 2 bytes 0.1hPa unsigned
#define LPP_VOLTAGE                     116   // 2 bytes 0.01V unsigned
#define LPP_CURRENT                     117   // 2 bytes 0.001A unsigned
#define LPP_FREQUENCY                   118   // 4 bytes 1Hz unsigned
#define LPP_PERCENTAGE                  120   // 1 byte 1-100% unsigned
#define LPP_ALTITUDE                    121   // 2 byte 1m signed
#define LPP_CONCENTRATION               125   // 2 bytes, 1 ppm unsigned
#define LPP_POWER                       128   // 2 byte, 1W, unsigned
#define LPP_DISTANCE                    130   // 4 byte, 0.001m, unsigned
#define LPP_ENERGY                      131   // 4 byte, 0.001kWh, unsigned
#define LPP_DIRECTION                   132   // 2 bytes, 1deg, unsigned
#define LPP_UNIXTIME                    133   // 4 bytes, unsigned
#define LPP_GYROMETER                   134   // 2 bytes per axis, 0.01 °/s
#define LPP_COLOUR                      135   // 1 byte per RGB Color
#define LPP_GPS                         136   // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01 meter
#define LPP_SWITCH                      142   // 1 byte, 0/1

// Custom
#define LPP_BATTERY_VOLTAGE             147   // 2 bytes
#define LPP_WAVE_DIRECTION              148   // 2 bytes
#define LPP_BMP280_TEMPERATURE          149   // 2 bytes
#define LPP_BMP280_PRESSURE             150   // 2 bytes
#define LPP_HEIGHT_AVERAGE              151   // 2 bytes
#define LPP_HEIGHT_MAX                  152   // 2 bytes
#define LPP_HEIGHT_SIGN                 153   // 2 bytes
#define LPP_WAVE_PERIOD                 154   // 2 bytes
////////////////////////////////////////////////////////
#define LPP_PH                          155   // 2 bytes
#define LPP_DO                          156   // 2 bytes
#define LPP_EC                          157   // 4 bytes
#define LPP_ORP                         158   // 4 bytes
#define LPP_RTD                         159   // 3 bytes

// Only Data Size
#define LPP_DIGITAL_INPUT_SIZE          1
#define LPP_DIGITAL_OUTPUT_SIZE         1
#define LPP_ANALOG_INPUT_SIZE           2
#define LPP_ANALOG_OUTPUT_SIZE          2
#define LPP_GENERIC_SENSOR_SIZE         4
#define LPP_LUMINOSITY_SIZE             2
#define LPP_PRESENCE_SIZE               1
#define LPP_TEMPERATURE_SIZE            2
#define LPP_RELATIVE_HUMIDITY_SIZE      1
#define LPP_ACCELEROMETER_SIZE          6
#define LPP_BAROMETRIC_PRESSURE_SIZE    2
#define LPP_VOLTAGE_SIZE                2
#define LPP_CURRENT_SIZE                2
#define LPP_FREQUENCY_SIZE              4
#define LPP_PERCENTAGE_SIZE             1
#define LPP_ALTITUDE_SIZE               2
#define LPP_POWER_SIZE                  2
#define LPP_DISTANCE_SIZE               4
#define LPP_ENERGY_SIZE                 4
#define LPP_DIRECTION_SIZE              2
#define LPP_UNIXTIME_SIZE               4
#define LPP_GYROMETER_SIZE              6
#define LPP_GPS_SIZE                    6   //9 //14
#define LPP_SWITCH_SIZE                 1
#define LPP_CONCENTRATION_SIZE          2
#define LPP_COLOUR_SIZE                 3

// Custom
#define LPP_BATTERY_VOLTAGE_SIZE        2   // 2 bytes
#define LPP_WAVE_DIRECTION_SIZE         2   // 2 bytes
#define LPP_BMP280_TEMPERATURE_SIZE     2   // 2 bytes
#define LPP_BMP280_PRESSURE_SIZE        3   // 3 bytes
#define LPP_HEIGHT_AVERAGE_SIZE         2   // 2 bytes
#define LPP_HEIGHT_MAX_SIZE             2   // 2 bytes
#define LPP_HEIGHT_SIGN_SIZE            2   // 2 bytes
#define LPP_WAVE_PERIOD_SIZE            2   // 2 bytes
////////////////////////////////////////////////////////
#define LPP_PH_SIZE                     2   // 2 bytes
#define LPP_DO_SIZE                     2   // 2 bytes
#define LPP_EC_SIZE                     4   // 3 bytes
#define LPP_ORP_SIZE                    3   // 4 bytes
#define LPP_RTD_SIZE                    2   // 2 bytes

// Multipliers
#define LPP_DIGITAL_INPUT_MULT          1
#define LPP_DIGITAL_OUTPUT_MULT         1
#define LPP_ANALOG_INPUT_MULT           100
#define LPP_ANALOG_OUTPUT_MULT          100
#define LPP_GENERIC_SENSOR_MULT         1
#define LPP_LUMINOSITY_MULT             1
#define LPP_PRESENCE_MULT               1
#define LPP_TEMPERATURE_MULT            10
#define LPP_RELATIVE_HUMIDITY_MULT      2
#define LPP_ACCELEROMETER_MULT          1000
#define LPP_BAROMETRIC_PRESSURE_MULT    10
#define LPP_VOLTAGE_MULT                100
#define LPP_CURRENT_MULT                1000
#define LPP_FREQUENCY_MULT              1
#define LPP_PERCENTAGE_MULT             1
#define LPP_ALTITUDE_MULT               1
#define LPP_POWER_MULT                  1
#define LPP_DISTANCE_MULT               1000
#define LPP_ENERGY_MULT                 1000
#define LPP_DIRECTION_MULT              1
#define LPP_UNIXTIME_MULT               1
#define LPP_GYROMETER_MULT              100
#define LPP_GPS_LAT_LON_MULT            100000 //1 //10000
#define LPP_GPS_ALT_MULT                100
#define LPP_SWITCH_MULT                 1
#define LPP_CONCENTRATION_MULT          1
#define LPP_COLOUR_MULT                 1

// Custom
#define LPP_BATTERY_VOLTAGE_MULT        100   
#define LPP_WAVE_DIRECTION_MULT         100  
#define LPP_BMP280_TEMPERATURE_MULT     100   
#define LPP_BMP280_PRESSURE_MULT        100
#define LPP_HEIGHT_AVERAGE_MULT         100   
#define LPP_HEIGHT_MAX_MULT             100   
#define LPP_HEIGHT_SIGN_MULT            100   
#define LPP_WAVE_PERIOD_MULT            100
////////////////////////////////////////////////////////
#define LPP_PH_MULT                     100     // 2 bytes
#define LPP_DO_MULT                     100     // 2 bytes
#define LPP_EC_MULT                     100000  // 4 bytes
#define LPP_ORP_MULT                    100     // 4 bytes
#define LPP_RTD_MULT                    100     // 3 bytes

#define LPP_ERROR_OK                    0
#define LPP_ERROR_OVERFLOW              1
#define LPP_ERROR_UNKOWN_TYPE           2

class CayenneLPP {

public:

  CayenneLPP(uint8_t size);
  ~CayenneLPP();

  void reset(void);
  uint8_t getSize(void);
  uint8_t *getBuffer(void);
  uint8_t copy(uint8_t *buffer);
  uint8_t getError();

  // Decoder methods
  const char * getTypeName(uint8_t type);
  const char * getTypeUnit(uint8_t type);
  uint8_t decode(uint8_t *buffer, uint8_t size, JsonArray& root);
  uint8_t decodeTTN(uint8_t *buffer, uint8_t size, JsonObject& root);

  // Original LPPv1 data types
  uint8_t addDigitalInput(uint32_t value);
  uint8_t addDigitalOutput(uint32_t value);
  uint8_t addAnalogInput(float value);
  uint8_t addAnalogOutput(float value);
  uint8_t addLuminosity(uint32_t value);
  uint8_t addPresence(uint32_t value);
  uint8_t addTemperature(float value);
  uint8_t addRelativeHumidity(float value);
  uint8_t addAccelerometer(float x, float y, float z, uint8_t error_data = 0);
  uint8_t addBarometricPressure(float value);
  uint8_t addGyrometer(float x, float y, float z, uint8_t error_data = 0);
  uint8_t addGPS(float latitude, float longitude, uint8_t error_data = 0);
  //uint8_t addGPS(float latitudeFirstHalf, uint32_t latitudeSecondHalf, float longitudeFirstHalf, uint32_t longitudeSecondHalf);
  //uint8_t addGPS(float latitude, float longitude, float altitude);

  // Additional data types
  uint8_t addUnixTime(uint32_t value);
  uint8_t addGenericSensor(float value);
  uint8_t addVoltage(float value);
  uint8_t addCurrent(float value);
  uint8_t addFrequency(uint32_t value);
  uint8_t addPercentage(uint32_t value);
  uint8_t addAltitude(float value);
  uint8_t addPower(uint32_t value);
  uint8_t addDistance(float value);
  uint8_t addEnergy(float value);
  uint8_t addDirection(float value);
  uint8_t addSwitch(uint32_t value);
  uint8_t addConcentration(uint32_t value);
  uint8_t addColour(uint8_t r, uint8_t g, uint8_t b, uint8_t error_data = 0);
  
  // Custom
  uint8_t addBatteryVoltage(float value);
  uint8_t addWaveDirection(float value);
  uint8_t addBMP280Temperature(float value);
  uint8_t addBMP280Pressure(float value);
  uint8_t addHeightAverage(float value);
  uint8_t addHeightMax(float value);
  uint8_t addHeightSign(float value);
  uint8_t addWavePeriod(float value);
  /////////////////////////////////////////////////////
  uint8_t addPH(float value, uint8_t error_data = 0);
  uint8_t addDO(float value, uint8_t error_data = 0);
  uint8_t addEC(float value, uint8_t error_data = 0);
  uint8_t addORP(float value, uint8_t error_data = 0);
  uint8_t addRTD(float value, uint8_t error_data = 0);

protected:

  bool isType(uint8_t type);
  uint8_t getTypeSize(uint8_t type);
  uint32_t getTypeMultiplier(uint8_t type);
  bool getTypeSigned(uint8_t type);

  float getValue(uint8_t * buffer, uint8_t size, uint32_t multiplier, bool is_signed);
  uint32_t getValue32(uint8_t * buffer, uint8_t size);
  template <typename T> uint8_t addField(uint8_t type, T value, uint8_t error_data = 0);

  uint8_t * _buffer;
  uint8_t _maxsize;
  uint8_t _cursor;
  uint8_t _error = LPP_ERROR_OK;

};

#endif