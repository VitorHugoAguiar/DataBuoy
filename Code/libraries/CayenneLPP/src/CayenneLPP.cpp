// Adapted from https://developer.mbed.org/teams/myDevicesIoT/code/Cayenne-LPP/

// Copyright © 2017 The Things Network
// Use of this source code is governed by the MIT license that can be found in the LICENSE file.

#include "CayenneLPP.h"

// ----------------------------------------------------------------------------

CayenneLPP::CayenneLPP(uint8_t size) : _maxsize(size) {
  _buffer = (uint8_t *)malloc(size);
  _cursor = 0;
}

CayenneLPP::~CayenneLPP(void) {
  free(_buffer);
}

void CayenneLPP::reset(void) {
  _cursor = 0;
}

uint8_t CayenneLPP::getSize(void) {
  return _cursor;
}

uint8_t *CayenneLPP::getBuffer(void) {
  return _buffer;
}

uint8_t CayenneLPP::copy(uint8_t *dst) {
  memcpy(dst, _buffer, _cursor);
  return _cursor;
}

uint8_t CayenneLPP::getError(void) {
  uint8_t error = _error;
  _error = LPP_ERROR_OK;
  return error;
}

// ----------------------------------------------------------------------------

bool CayenneLPP::isType(uint8_t type) {

  switch (type) { 
    case LPP_DIGITAL_INPUT:
    case LPP_DIGITAL_OUTPUT:
    case LPP_ANALOG_INPUT:
    case LPP_ANALOG_OUTPUT:
    case LPP_GENERIC_SENSOR:
    case LPP_LUMINOSITY:
    case LPP_PRESENCE:
    case LPP_TEMPERATURE:
    case LPP_RELATIVE_HUMIDITY:
    case LPP_ACCELEROMETER:
    case LPP_BAROMETRIC_PRESSURE:
    case LPP_VOLTAGE:
    case LPP_CURRENT:
    case LPP_FREQUENCY:
    case LPP_PERCENTAGE:
    case LPP_ALTITUDE:
    case LPP_POWER:
    case LPP_DISTANCE:
    case LPP_ENERGY:
    case LPP_DIRECTION:
    case LPP_UNIXTIME:
    case LPP_GYROMETER:
    case LPP_GPS:
    case LPP_SWITCH:
    case LPP_CONCENTRATION:
    case LPP_COLOUR:
    // Custom
    case LPP_BATTERY_VOLTAGE:
    case LPP_WAVE_DIRECTION:
    case LPP_BMP280_TEMPERATURE:
    case LPP_BMP280_PRESSURE:
    case LPP_HEIGHT_AVERAGE:
    case LPP_HEIGHT_MAX:
    case LPP_HEIGHT_SIGN:
    case LPP_WAVE_PERIOD:
    ///////////////////////////
    case LPP_PH:
    case LPP_DO:
    case LPP_EC:
    case LPP_ORP:
    case LPP_RTD:
    return true;
  }

  return false;
}

const char * CayenneLPP::getTypeName(uint8_t type) {

    switch (type) {

        case LPP_DIGITAL_INPUT:
          return "digital_in";

        case LPP_DIGITAL_OUTPUT:
          return "digital_out";

        case LPP_ANALOG_INPUT:
          return "analog_in";

        case LPP_ANALOG_OUTPUT:
          return "analog_out";

        case LPP_GENERIC_SENSOR:
          return "generic";

        case LPP_LUMINOSITY:
          return "luminosity";

        case LPP_PRESENCE:
          return "presence";

        case LPP_TEMPERATURE:
          return "temperature";

        case LPP_RELATIVE_HUMIDITY:
          return "humidity";

        case LPP_ACCELEROMETER:
          return "accelerometer";

        case LPP_BAROMETRIC_PRESSURE:
          return "pressure";

        case LPP_VOLTAGE:
          return "voltage";

        case LPP_CURRENT:
          return "current";

        case LPP_FREQUENCY:
          return "frequency";

        case LPP_PERCENTAGE:
          return "percentage";

        case LPP_ALTITUDE:
          return "altitude";

        case LPP_POWER:
          return "power";

        case LPP_DISTANCE:
          return "distance";

        case LPP_ENERGY:
          return "energy";

        case LPP_DIRECTION:
          return "direction";

        case LPP_UNIXTIME:
          return "Epoch_Time";

        case LPP_GYROMETER:
          return "gyrometer";

        case LPP_GPS:
          return "GPS";

        case LPP_SWITCH:
          return "switch";

        case LPP_CONCENTRATION:
          return "concentration";

        case LPP_COLOUR:
          return "colour";

        // Custom
        case LPP_BATTERY_VOLTAGE:
          return "BatteryVoltage";

        case LPP_WAVE_DIRECTION:
          return "WaveDirection";

        case LPP_BMP280_TEMPERATURE:
          return "BMP280_Temperature";

        case LPP_BMP280_PRESSURE:
          return "BMP280_Pressure";

        case LPP_HEIGHT_AVERAGE:
          return "Height_Average";
        
        case LPP_HEIGHT_MAX:
          return "Height_Max";

        case LPP_HEIGHT_SIGN:
          return "Height_Significant";

        case LPP_WAVE_PERIOD:
          return "WavePeriod";
        ///////////////////////////////////
        case LPP_PH:
          return "PH";
        case LPP_DO:
          return "DO";
        case LPP_EC:
          return "EC";
        case LPP_ORP:
          return "ORP";
        case LPP_RTD:
          return "RTD";                     
        default:
          return nullptr;
    }

}

//Custom
const char * CayenneLPP::getTypeUnit(uint8_t type) {
// Custom
    switch (type) {
        case LPP_UNIXTIME:
          return "(sec)";
        
        case LPP_GPS:
          return "(deg)";
        
        case LPP_BATTERY_VOLTAGE:
          return "(V)";

        case LPP_WAVE_DIRECTION:
          return "(deg)";

        case LPP_BMP280_TEMPERATURE:
          return "(C)";

        case LPP_BMP280_PRESSURE:
          return "(hPa)";

        case LPP_HEIGHT_AVERAGE:
          return "(m)";
        
        case LPP_HEIGHT_MAX:
          return "(m)";

        case LPP_HEIGHT_SIGN:
          return "(m)";

        case LPP_WAVE_PERIOD:
          return "(sec)";

        //////////////////////////////////
        case LPP_DO:
          return "(mg/L)";
        case LPP_EC:
          return "(ppt)";
        case LPP_ORP:
          return "(mV)";
        case LPP_RTD:
          return "(C)";         
        default:
          return nullptr;
    }

}

uint8_t CayenneLPP::getTypeSize(uint8_t type) {

  switch (type) {

    case LPP_DIGITAL_INPUT:
      return LPP_DIGITAL_INPUT_SIZE;

    case LPP_DIGITAL_OUTPUT:
      return LPP_DIGITAL_OUTPUT_SIZE;

    case LPP_ANALOG_INPUT:
      return LPP_ANALOG_INPUT_SIZE;

    case LPP_ANALOG_OUTPUT:
      return LPP_ANALOG_OUTPUT_SIZE;

    case LPP_GENERIC_SENSOR:
      return LPP_GENERIC_SENSOR_SIZE;

    case LPP_LUMINOSITY:
      return LPP_LUMINOSITY_SIZE;

    case LPP_PRESENCE:
      return LPP_PRESENCE_SIZE;

    case LPP_TEMPERATURE:
      return LPP_TEMPERATURE_SIZE;

    case LPP_RELATIVE_HUMIDITY:
      return LPP_RELATIVE_HUMIDITY_SIZE;

    case LPP_ACCELEROMETER:
      return LPP_ACCELEROMETER_SIZE;

    case LPP_BAROMETRIC_PRESSURE:
      return LPP_BAROMETRIC_PRESSURE_SIZE;

    case LPP_VOLTAGE:
      return LPP_VOLTAGE_SIZE;

    case LPP_CURRENT:
      return LPP_CURRENT_SIZE;

    case LPP_FREQUENCY:
      return LPP_FREQUENCY_SIZE;

    case LPP_PERCENTAGE:
      return LPP_PERCENTAGE_SIZE;

    case LPP_ALTITUDE:
      return LPP_ALTITUDE_SIZE;

    case LPP_POWER:
      return LPP_POWER_SIZE;

    case LPP_DISTANCE:
      return LPP_DISTANCE_SIZE;

    case LPP_ENERGY:
      return LPP_ENERGY_SIZE;

    case LPP_DIRECTION:
      return LPP_DIRECTION_SIZE;

    case LPP_UNIXTIME:
      return LPP_UNIXTIME_SIZE;

    case LPP_GYROMETER:
      return LPP_GYROMETER_SIZE;

    case LPP_GPS:
      return LPP_GPS_SIZE;

    case LPP_SWITCH:
      return LPP_SWITCH_SIZE;

    case LPP_CONCENTRATION:
      return LPP_CONCENTRATION_SIZE;

    case LPP_COLOUR:
      return LPP_COLOUR_SIZE;
    
    // Custom
    case LPP_BATTERY_VOLTAGE:
      return LPP_BATTERY_VOLTAGE_SIZE;

    case LPP_WAVE_DIRECTION:
      return LPP_WAVE_DIRECTION_SIZE;

    case LPP_BMP280_TEMPERATURE:
      return LPP_BMP280_TEMPERATURE_SIZE;

    case LPP_BMP280_PRESSURE:
      return LPP_BMP280_PRESSURE_SIZE;

    case LPP_HEIGHT_AVERAGE:
      return LPP_HEIGHT_AVERAGE_SIZE;
      
    case LPP_HEIGHT_MAX:
      return LPP_HEIGHT_MAX_SIZE;

    case LPP_HEIGHT_SIGN:
      return LPP_HEIGHT_SIGN_SIZE;

    case LPP_WAVE_PERIOD:
      return LPP_WAVE_PERIOD_SIZE;

    /////////////////////////////////////////////
    case LPP_PH:
      return LPP_PH_SIZE;  

    case LPP_DO:
      return LPP_DO_SIZE; 

    case LPP_EC:
      return LPP_EC_SIZE; 

    case LPP_ORP:
      return LPP_ORP_SIZE; 

    case LPP_RTD:
      return LPP_RTD_SIZE; 
          
    default:
      return 0;
  }
}

uint32_t CayenneLPP::getTypeMultiplier(uint8_t type) {

  switch (type) {

    case LPP_DIGITAL_INPUT:
      return LPP_DIGITAL_INPUT_MULT;

    case LPP_DIGITAL_OUTPUT:
      return LPP_DIGITAL_OUTPUT_MULT;

    case LPP_ANALOG_INPUT:
      return LPP_ANALOG_INPUT_MULT;

    case LPP_ANALOG_OUTPUT:
      return LPP_ANALOG_OUTPUT_MULT;

    case LPP_GENERIC_SENSOR:
      return LPP_GENERIC_SENSOR_MULT;

    case LPP_LUMINOSITY:
      return LPP_LUMINOSITY_MULT;

    case LPP_PRESENCE:
      return LPP_PRESENCE_MULT;

    case LPP_TEMPERATURE:
      return LPP_TEMPERATURE_MULT;

    case LPP_RELATIVE_HUMIDITY:
      return LPP_RELATIVE_HUMIDITY_MULT;

    case LPP_ACCELEROMETER:
      return LPP_ACCELEROMETER_MULT;

    case LPP_BAROMETRIC_PRESSURE:
      return LPP_BAROMETRIC_PRESSURE_MULT;

    case LPP_VOLTAGE:
      return LPP_VOLTAGE_MULT;

    case LPP_CURRENT:
      return LPP_CURRENT_MULT;

    case LPP_FREQUENCY:
      return LPP_FREQUENCY_MULT;

    case LPP_PERCENTAGE:
      return LPP_PERCENTAGE_MULT;

    case LPP_ALTITUDE:
      return LPP_ALTITUDE_MULT;

    case LPP_POWER:
      return LPP_POWER_MULT;

    case LPP_DISTANCE:
      return LPP_DISTANCE_MULT;

    case LPP_ENERGY:
      return LPP_ENERGY_MULT;

    case LPP_DIRECTION:
      return LPP_DIRECTION_MULT;

    case LPP_UNIXTIME:
      return LPP_UNIXTIME_MULT;

    case LPP_GYROMETER:
      return LPP_GYROMETER_MULT;

    case LPP_SWITCH:
      return LPP_SWITCH_MULT;

    case LPP_CONCENTRATION:
      return LPP_CONCENTRATION_MULT;

    case LPP_COLOUR:
      return LPP_COLOUR_MULT;

    // Custom
    case LPP_BATTERY_VOLTAGE:
      return LPP_BATTERY_VOLTAGE_MULT;

    case LPP_WAVE_DIRECTION:
      return LPP_WAVE_DIRECTION_MULT;

    case LPP_BMP280_TEMPERATURE:
      return LPP_BMP280_TEMPERATURE_MULT;

    case LPP_BMP280_PRESSURE:
      return LPP_BMP280_PRESSURE_MULT;

    case LPP_HEIGHT_AVERAGE:
      return LPP_HEIGHT_AVERAGE_MULT;

    case LPP_HEIGHT_MAX:
      return LPP_HEIGHT_MAX_MULT;

    case LPP_HEIGHT_SIGN:
      return LPP_HEIGHT_SIGN_MULT;
      
    case LPP_WAVE_PERIOD:
      return LPP_WAVE_PERIOD_MULT;

    ////////////////////////////////////////////////
    case LPP_PH:
      return LPP_PH_MULT;  

    case LPP_DO:
      return LPP_DO_MULT; 

    case LPP_EC:
      return LPP_EC_MULT;

    case LPP_ORP:
      return LPP_ORP_MULT;

    case LPP_RTD:
      return LPP_RTD_MULT;

    default:
      return 0;
  }
}

bool CayenneLPP::getTypeSigned(uint8_t type) {
	
  switch (type) {
    case LPP_ANALOG_INPUT:
    case LPP_ANALOG_OUTPUT:
    case LPP_TEMPERATURE:
    case LPP_ACCELEROMETER:
    case LPP_ALTITUDE:
    case LPP_GYROMETER:
    case LPP_GPS:
    // Custom
    case LPP_BMP280_TEMPERATURE:
    case LPP_ORP:
    case LPP_RTD:
    return true;
  }
  return false;
}

// ----------------------------------------------------------------------------

template <typename T> uint8_t CayenneLPP::addField(uint8_t type, T value, uint8_t error_data) {

  // Check type
  if (!isType(type)) {
    _error = LPP_ERROR_UNKOWN_TYPE;
    return 0;
  }

  // Type definition
  uint8_t size = getTypeSize(type);
  uint32_t multiplier = getTypeMultiplier(type);
  bool is_signed = getTypeSigned(type);

  // check buffer overflow
  if ((_cursor + size + 1) > _maxsize) {
    _error = LPP_ERROR_OVERFLOW;
    return 0;
  }

  // check sign  
  bool sign = value < 0;
  if (sign) value = -value;

  // get value to store
  uint32_t v = value * multiplier;

  // format an uint32_t as if it was an int32_t
  if (is_signed & sign) {
    uint32_t mask = (1 << (size * 8)) - 1;
    v = v & mask;
    if (sign) v = mask - v + 1;
  }

  // header
  _buffer[_cursor++] = type;
  _buffer[_cursor++] = error_data;

  // add bytes (MSB first)
  for (uint8_t i=1; i<=size; i++) {
    _buffer[_cursor + size - i] = (v & 0xFF);
    v >>= 8;
  }

  // update & return _cursor
  _cursor += size;
  return _cursor;

}

uint8_t CayenneLPP::addDigitalInput(uint32_t value) {
  return addField(LPP_DIGITAL_INPUT, value);
}

uint8_t CayenneLPP::addDigitalOutput(uint32_t value) {
  return addField(LPP_DIGITAL_OUTPUT, value);
}

uint8_t CayenneLPP::addAnalogInput(float value) {
  return addField(LPP_ANALOG_INPUT, value);
}

uint8_t CayenneLPP::addAnalogOutput(float value) {
  return addField(LPP_ANALOG_OUTPUT, value);
}

uint8_t CayenneLPP::addGenericSensor(float value)  {
  return addField(LPP_GENERIC_SENSOR, value);
}

uint8_t CayenneLPP::addLuminosity(uint32_t value) {
  return addField(LPP_LUMINOSITY, value);
}

uint8_t CayenneLPP::addPresence(uint32_t value) {
  return addField(LPP_PRESENCE, value);
}

uint8_t CayenneLPP::addTemperature(float value) {
  return addField(LPP_TEMPERATURE, value);
}

uint8_t CayenneLPP::addRelativeHumidity(float value) {
  return addField(LPP_RELATIVE_HUMIDITY, value);
}

uint8_t CayenneLPP::addVoltage(float value) {
  return addField(LPP_VOLTAGE, value);
}

uint8_t CayenneLPP::addCurrent(float value) {
  return addField(LPP_CURRENT, value);
}

uint8_t CayenneLPP::addFrequency(uint32_t value) {
  return addField(LPP_FREQUENCY, value);
}

uint8_t CayenneLPP::addPercentage(uint32_t value) {
  return addField(LPP_PERCENTAGE, value);
}

uint8_t CayenneLPP::addAltitude(float value) {
  return addField(LPP_ALTITUDE, value);
}

uint8_t CayenneLPP::addPower(uint32_t value) {
  return addField(LPP_POWER, value);
}

uint8_t CayenneLPP::addDistance(float value) {
  return addField(LPP_DISTANCE, value);
}

uint8_t CayenneLPP::addEnergy(float value) {
  return addField(LPP_ENERGY, value);
}

uint8_t CayenneLPP::addBarometricPressure(float value) {
  return addField(LPP_BAROMETRIC_PRESSURE, value);
}

uint8_t CayenneLPP::addUnixTime(uint32_t value) {
  return addField(LPP_UNIXTIME, value);
}

uint8_t CayenneLPP::addDirection(float value) {
  return addField(LPP_DIRECTION, value);
}

uint8_t CayenneLPP::addSwitch(uint32_t value) {
  return addField(LPP_SWITCH, value);
}

uint8_t CayenneLPP::addConcentration(uint32_t value) {
  return addField(LPP_CONCENTRATION, value);
}

uint8_t CayenneLPP::addColour(uint8_t r, uint8_t g, uint8_t b, uint8_t error_data)
{
  // check buffer overflow
  if ((_cursor + LPP_COLOUR_SIZE + 1) > _maxsize) {
    _error = LPP_ERROR_OVERFLOW;
    return 0;
  }
  _buffer[_cursor++] = LPP_COLOUR;
  _buffer[_cursor++] = error_data;
  _buffer[_cursor++] = r;
  _buffer[_cursor++] = g;
  _buffer[_cursor++] = b;
 

  return _cursor;
}

uint8_t CayenneLPP::addAccelerometer(float x, float y, float z, uint8_t error_data) {
  
  // check buffer overflow
  if ((_cursor + LPP_ACCELEROMETER_SIZE + 1) > _maxsize) {
    _error = LPP_ERROR_OVERFLOW;
    return 0;
  }

  int16_t vx = x * LPP_ACCELEROMETER_MULT;
  int16_t vy = y * LPP_ACCELEROMETER_MULT;
  int16_t vz = z * LPP_ACCELEROMETER_MULT;

  _buffer[_cursor++] = LPP_ACCELEROMETER; 
  _buffer[_cursor++] =  error_data;
  _buffer[_cursor++] = vx >> 8;
  _buffer[_cursor++] = vx;
  _buffer[_cursor++] = vy >> 8;
  _buffer[_cursor++] = vy;
  _buffer[_cursor++] = vz >> 8;
  _buffer[_cursor++] = vz;

  return _cursor;

}

uint8_t CayenneLPP::addGyrometer(float x, float y, float z, uint8_t error_data) {

  // check buffer overflow
  if ((_cursor + LPP_GYROMETER_SIZE + 1) > _maxsize) {
    _error = LPP_ERROR_OVERFLOW;
    return 0;
  }

  int16_t vx = x * LPP_GYROMETER_MULT;
  int16_t vy = y * LPP_GYROMETER_MULT;
  int16_t vz = z * LPP_GYROMETER_MULT;

  _buffer[_cursor++] = LPP_GYROMETER;
  _buffer[_cursor++] = error_data;
  _buffer[_cursor++] = vx >> 8;
  _buffer[_cursor++] = vx;
  _buffer[_cursor++] = vy >> 8;
  _buffer[_cursor++] = vy;
  _buffer[_cursor++] = vz >> 8;
  _buffer[_cursor++] = vz;

  return _cursor;

}

uint8_t CayenneLPP::addGPS(float latitude, float longitude, uint8_t error_data) {
  // check buffer overflow
  if ((_cursor + LPP_GPS_SIZE + 1) > _maxsize) {
    _error = LPP_ERROR_OVERFLOW;
    return 0;
  }

  int32_t lat = latitude * LPP_GPS_LAT_LON_MULT;
  int32_t lon = longitude * LPP_GPS_LAT_LON_MULT;

  _buffer[_cursor++] = LPP_GPS;
  _buffer[_cursor++] = error_data;  
  _buffer[_cursor++] = lat >> 16;
  _buffer[_cursor++] = lat >> 8;
  _buffer[_cursor++] = lat;
  _buffer[_cursor++] = lon >> 16;
  _buffer[_cursor++] = lon >> 8;
  _buffer[_cursor++] = lon;
  return _cursor;
}

/*uint8_t CayenneLPP::addGPS(float latitudeFirstHalf, uint32_t latitudeSecondHalf, float longitudeFirstHalf, uint32_t longitudeSecondHalf) {
  
  // check buffer overflow
  if ((_cursor + LPP_GPS_SIZE + 1) > _maxsize) {
    _error = LPP_ERROR_OVERFLOW;
    return 0;
  }

  int32_t lat1 = latitudeFirstHalf * LPP_GPS_LAT_LON_MULT;
  int32_t lat2 = latitudeSecondHalf * LPP_GPS_LAT_LON_MULT;
  int32_t lon1 = longitudeFirstHalf * LPP_GPS_LAT_LON_MULT;
  int32_t lon2 = longitudeSecondHalf * LPP_GPS_LAT_LON_MULT;

  _buffer[_cursor++] = LPP_GPS;
  _buffer[_cursor++] = lat1 >> 16;
  _buffer[_cursor++] = lat1 >> 8;
  _buffer[_cursor++] = lat1;
  _buffer[_cursor++] = lat2 >> 24;
  _buffer[_cursor++] = lat2 >> 16;
  _buffer[_cursor++] = lat2 >> 8;
  _buffer[_cursor++] = lat2;
  _buffer[_cursor++] = lon1 >> 16;
  _buffer[_cursor++] = lon1 >> 8;
  _buffer[_cursor++] = lon1;
  _buffer[_cursor++] = lon2 >> 24;
  _buffer[_cursor++] = lon2 >> 16;
  _buffer[_cursor++] = lon2 >> 8;
  _buffer[_cursor++] = lon2; 

  return _cursor;

}*/

/*uint8_t CayenneLPP::addGPS(float latitude, float longitude, float altitude) {
  // check buffer overflow
  if ((_cursor + LPP_GPS_SIZE + 1) > _maxsize) {
    _error = LPP_ERROR_OVERFLOW;
    return 0;
  }
  int32_t lat = latitude * LPP_GPS_LAT_LON_MULT;
  int32_t lon = longitude * LPP_GPS_LAT_LON_MULT;
  int32_t alt = altitude * LPP_GPS_ALT_MULT;
  _buffer[_cursor++] = LPP_GPS;  
  _buffer[_cursor++] = lat >> 16;
  _buffer[_cursor++] = lat >> 8;
  _buffer[_cursor++] = lat;
  _buffer[_cursor++] = lon >> 16;
  _buffer[_cursor++] = lon >> 8;
  _buffer[_cursor++] = lon;
  _buffer[_cursor++] = alt >> 16;
  _buffer[_cursor++] = alt >> 8;
  _buffer[_cursor++] = alt;
  return _cursor;
}*/

// Custom
uint8_t CayenneLPP::addBatteryVoltage(float value) {
  return addField(LPP_BATTERY_VOLTAGE, value);
}

uint8_t CayenneLPP::addWaveDirection(float value) {
  return addField(LPP_WAVE_DIRECTION, value);
}

uint8_t CayenneLPP::addBMP280Temperature(float value) {
  return addField(LPP_BMP280_TEMPERATURE, value);
}

uint8_t CayenneLPP::addBMP280Pressure(float value) {
  return addField(LPP_BMP280_PRESSURE, value);
}

uint8_t CayenneLPP::addHeightAverage(float value) {
  return addField(LPP_HEIGHT_AVERAGE, value);
}

uint8_t CayenneLPP::addHeightMax(float value) {
  return addField(LPP_HEIGHT_MAX, value);
}

uint8_t CayenneLPP::addHeightSign(float value) {
  return addField(LPP_HEIGHT_SIGN, value);
}

uint8_t CayenneLPP::addWavePeriod(float value) {
  return addField(LPP_WAVE_PERIOD, value);
}

/////////////////////////////////////////////////////////////
uint8_t CayenneLPP::addPH(float value, uint8_t error_data) {
  return addField(LPP_PH, value, error_data);
}

uint8_t CayenneLPP::addDO(float value, uint8_t error_data) {
  return addField(LPP_DO, value, error_data);
}

uint8_t CayenneLPP::addEC(float value, uint8_t error_data) {
  return addField(LPP_EC, value, error_data);
}

uint8_t CayenneLPP::addORP(float value, uint8_t error_data) {
  // check buffer overflow
  if ((_cursor + LPP_ORP_SIZE + 1) > _maxsize) {
    _error = LPP_ERROR_OVERFLOW;
    return 0;
  }

  int32_t ORP_Value = value * LPP_ORP_MULT;

  _buffer[_cursor++] = LPP_ORP; 
  _buffer[_cursor++] = error_data; 
  _buffer[_cursor++] = ORP_Value >> 16;
  _buffer[_cursor++] = ORP_Value >> 8;
  _buffer[_cursor++] = ORP_Value;
  return _cursor;
}

uint8_t CayenneLPP::addRTD(float value, uint8_t error_data) {
  return addField(LPP_RTD, value, error_data);
}
// ----------------------------------------------------------------------------

float CayenneLPP::getValue(uint8_t * buffer, uint8_t size, uint32_t multiplier, bool is_signed) {

    uint32_t value = 0;
    for (uint8_t i=0; i<size; i++) {
      value = (value << 8) + buffer[i];
    }

    int sign = 1;
    if (is_signed) {
      uint32_t bit = 1ul << ((size * 8) - 1);
      if ((value & bit) == bit) {
        value = (bit << 1) - value;
        sign = -1;
      }
    }

    return sign * ((float) value / multiplier);

}

uint32_t CayenneLPP::getValue32(uint8_t * buffer, uint8_t size) {

    uint32_t value = 0;
    for (uint8_t i=0; i<size; i++) {
      value = (value << 8) + buffer[i];
    }

    return value;

}

uint8_t CayenneLPP::decode(uint8_t *buffer, uint8_t len, JsonArray& root) {

  uint8_t count = 0;
  uint8_t index = 0;

  while ((index + 2) < len) {

    count++;
    
    // Get data type
    uint8_t type = buffer[index++];
    uint8_t error_data = buffer[index++];
    if (!isType(type)) {
      _error = LPP_ERROR_UNKOWN_TYPE;
      return 0;
    }

    // Type definition
    uint8_t size = getTypeSize(type);
    uint32_t multiplier = getTypeMultiplier(type);
    bool is_signed = getTypeSigned(type);

    // Check buffer size
    if (index + size > len) {
      _error = LPP_ERROR_OVERFLOW;
      return 0;
    }

    // Init object
    JsonObject data = root.createNestedObject();
    data["type"] = type;
    data["typeUnit"]  = String(getTypeUnit(type));
    data["name"] = String(getTypeName(type));
    data["error_data"] = String(error_data);

    // Parse types
	if (LPP_COLOUR == type) {

      JsonObject object = data.createNestedObject("value");
      object["r"] = getValue(&buffer[index], 1, multiplier, is_signed);
      object["g"] = getValue(&buffer[index+1], 1, multiplier, is_signed);
      object["b"] = getValue(&buffer[index+2], 1, multiplier, is_signed);

    } else if (LPP_ACCELEROMETER == type || LPP_GYROMETER == type) {

      JsonObject object = data.createNestedObject("value");
      object["x"] = getValue(&buffer[index], 2, multiplier, is_signed);
      object["y"] = getValue(&buffer[index+2], 2, multiplier, is_signed);
      object["z"] = getValue(&buffer[index+4], 2, multiplier, is_signed);

    } else if (LPP_GPS == type) {

      JsonObject object = data.createNestedObject("value");
      
      object["latitude"] = getValue(&buffer[index], 3, 100000, is_signed);
      object["longitude"] = getValue(&buffer[index+3], 3, 100000, is_signed);
      

      /*object["latitudeFirstHalf"] = getValue(&buffer[index], 3, 1, is_signed);
      object["latitudeSecondHalf"] = getValue32(&buffer[index+3], 4);
      object["longitudeFirstHalf"] = getValue(&buffer[index+7], 3, 1, is_signed);
      object["longitudeSecondHalf"] = getValue32(&buffer[index+10], 4);*/

      /*object["latitude"] = getValue(&buffer[index], 3, 10000, is_signed);
      object["longitude"] = getValue(&buffer[index+3], 3, 10000, is_signed);
      object["altitude"] = getValue(&buffer[index+6], 3, 100, is_signed);*/
    
    } else if (LPP_GENERIC_SENSOR == type || LPP_UNIXTIME == type) {

      data["value"] = getValue32(&buffer[index], size);

    } else {

      data["value"] = getValue(&buffer[index], size, multiplier, is_signed);

    }

    index += size;

  }

  return count;

}

uint8_t CayenneLPP::decodeTTN(uint8_t *buffer, uint8_t len, JsonObject& root) {

  uint8_t count = 0;
  uint8_t index = 0;

  while ((index + 2) < len) {

    count++;

    // Get data type
    uint8_t type = buffer[index++];
    if (!isType(type)) {
      _error = LPP_ERROR_UNKOWN_TYPE;
      return 0;
    }

    // Type definition
    uint8_t size = getTypeSize(type);
    uint32_t multiplier = getTypeMultiplier(type);
    bool is_signed = getTypeSigned(type);

    // Check buffer size
    if (index + size > len) {
      _error = LPP_ERROR_OVERFLOW;
      return 0;
    }

    // Init object
    String name = String(getTypeName(type));

    // Parse types
	if (LPP_COLOUR == type) {
      JsonObject object = root.createNestedObject(name);
      object["r"] = getValue(&buffer[index], 1, multiplier, is_signed);
      object["g"] = getValue(&buffer[index+1], 1, multiplier, is_signed);
      object["b"] = getValue(&buffer[index+2], 1, multiplier, is_signed);

    } else if (LPP_ACCELEROMETER == type || LPP_GYROMETER == type) {

      JsonObject object = root.createNestedObject(name);
      object["x"] = getValue(&buffer[index], 2, multiplier, is_signed);
      object["y"] = getValue(&buffer[index+2], 2, multiplier, is_signed);
      object["z"] = getValue(&buffer[index+4], 2, multiplier, is_signed);

    } else if (LPP_GPS == type) {

      JsonObject object = root.createNestedObject(name);
      object["latitude"] = getValue(&buffer[index], 3, 10000, is_signed);
      object["longitude"] = getValue(&buffer[index+3], 3, 10000, is_signed);
      object["altitude"] = getValue(&buffer[index+6], 3, 100, is_signed);

    } else if (LPP_GENERIC_SENSOR == type || LPP_UNIXTIME == type ) {

      root[name] = getValue32(&buffer[index], size);

    } else {

      root[name] = getValue(&buffer[index], size, multiplier, is_signed);

    }

    index += size;

  }

  return count;

}