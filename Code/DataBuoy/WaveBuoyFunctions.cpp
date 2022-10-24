#include "SharedFunctions.h"
#include "WaveBuoyFunctions.h"

SharedFunctions sf_Wave;

// Objects definition
RtcDS3231<TwoWire> RtcWave(Wire);                 // Create RTC
File logFileWriteWave;                            // Create SD Card
SFE_MAX1704X lipoWave(MAX1704X_MAX17048);         // Create a MAX17048
MPU9250 mpu;
Adafruit_BMP280 bmp280; // I2C Interface
timestamp32bits stampWave = timestamp32bits();
Madgwick Madgwick_Filter;
CayenneLPP lppWave(160);
HardwareSerial gpsSerialWave(2);

Filter* LowPassFilterAccZ;
Filter* HighPassFilterVelocity;
Filter* HighPassFilterDisplacement;

// GPIO used for wake up.
gpio_num_t gpio_WakeUpWave = GPIO_NUM_36;

// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

char filenameWave[30];                            // Buffer used to store the name of CSV file

// CSV file headers
String CSV_Header_WaveBuoy[] = {"WaveBuoy_ID", "SampleRate (Hz)", "RTC_Date", "RTC_Hour", "BatteryVoltage (V)", "BMP280_Temperature (C)", "BMP280_Pressure (hPa)", "WaveDirection (deg)", \
                                "Latitude", "Longitude", "Height (m)", "Height_Average (m)", "Height_Max (m)", "Height_Significant (m)", "WavePeriod (sec)"
                               };

// Data type for Queue item
struct Data_WaveBuoy {
  String RTC_Date;
  String RTC_Hour;
  uint32_t Epoch_Time;
  float BatteryVoltage;
  float BMP280_Temperature;
  float BMP280_Pressure;
  float WaveDirection;
  float Height;
  float Height_Average;
  float Height_Max;
  float Height_Sign;
  float WavePeriod;
  float Latitude = 0;
  float Longitude = 0;
} TX_Data_WaveBuoy, RX_Data_WaveBuoy;

boolean wrongDateWave = false;      // When the starting date is not defined or wrong

// GPS variables
byte DataWave[1000];
int SizeWave;
bool canSleepWave = false;
union
{
  int i;
  byte b[4];
} gpsLon, gpsLat;

// Declare Queue data type for FreeRTOS
QueueHandle_t DataQueueWaveBuoyWriteSD;
// Create Interrupt Semaphores
SemaphoreHandle_t SDWriteSemaphoreWave;

time_t prevTimeStampWave = 0;     // Stores the previous timestamp

float R[3][3];                    // Rotation Matrix


// WaveBuoy setup
void WaveBuoyFunctions::setup(boolean SerialDebug, boolean SleepMode, uint8_t WaveBuoy_ID, uint8_t SampleRate) {

  WiFi.mode(WIFI_OFF);                                                      // Turn Off WiFi

  // Turn on sensors
  pinMode(TurnSensorsOn, OUTPUT);
  digitalWrite(TurnSensorsOn, HIGH);
  delay(1000);

  Wire.begin();
  EEPROM.begin(512);                                                        // EEPROM begin

  SDWriteSemaphoreWave = xSemaphoreCreateBinary();                          // Create semaphore
  DataQueueWaveBuoyWriteSD = xQueueCreate(100, sizeof(Data_WaveBuoy));      // Queue with the number of struct variables

  // Convert the arguments into global variables
  _SerialDebug = SerialDebug;
  _SleepMode = SleepMode;

  _WaveBuoy_ID = WaveBuoy_ID;
  _SampleRate = SampleRate;
}

// Start Madqwick
void WaveBuoyFunctions::start_Madgwick_Filter(uint8_t SampleRate) {
  Madgwick_Filter.begin(SampleRate);      // Madgwick_Filter with the sample rate
}

// Start LP Filter (AccZ)
void WaveBuoyFunctions::initLowPassFilterAccZ (float LPCutOffFreqAccZ, uint8_t SampleRate) {
  LowPassFilterAccZ = new Filter(LPCutOffFreqAccZ, (1 / (float)SampleRate), IIR::ORDER::OD2, IIR::TYPE::LOWPASS);
}

// Start HP Filter (Velocity)
void WaveBuoyFunctions::initHighPassFilterVelocity (float HPCutOffFreqVelocity, uint8_t SampleRate) {
  HighPassFilterVelocity = new Filter(HPCutOffFreqVelocity, (1 / (float)SampleRate), IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
}

// Start HP Filter (Displacement)
void WaveBuoyFunctions::initHighPassFilterDisplacement (float HPCutOffFreqDisplacement, uint8_t SampleRate) {
  HighPassFilterDisplacement = new Filter(HPCutOffFreqDisplacement, (1 / (float)SampleRate), IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
}

// Start GPS
void WaveBuoyFunctions::start_GPS() {

  Serial.println(F("\n\u21D2 Initializing GPS..."));

  //Boot the GPS module
  Serial.println(F("  \u00BB Booting it up..."));
  sf_Wave.displayLED(CRGB::Blue, 15);

  //GPS Reset
  pinMode(GPS_Reset, OUTPUT);
  digitalWrite(GPS_Reset, HIGH);

  //GPS OnOff
  pinMode(GPS_ONOFF, OUTPUT);

  //GPS WakeUp
  pinMode(GPS_WakeUP, INPUT);

  delay(1000);
  while (digitalRead(GPS_WakeUP) == LOW) {
    digitalWrite(GPS_ONOFF, LOW);
    digitalWrite(GPS_ONOFF, HIGH);
    delay(200);
    digitalWrite(GPS_ONOFF, LOW);
    delay(1000);
  }

  Serial.println(F("  \u00BB GPS is booted!"));

  //GPS Serial (Tx, Rx)
  Serial.println(F("  \u00BB Connecting to GPS Serial..."));
  gpsSerialWave.begin(115200, SERIAL_8N1, GPS_Tx, GPS_Rx);
  while (!gpsSerialWave);
  Serial.println(F("  \u00BB GPS Serial is connected!"));

  sf_Wave.displayLED(CRGB::Black, 0);
}

// Start RTC
void WaveBuoyFunctions::start_RTC(uint8_t StartTimeArray[6], uint8_t AverageTimeArray[3], uint8_t SampleLengthMinutes, uint8_t* Alarm_WakeUp_Minutes, int Alarm_WakeUp_Minutes_Size) {

  Serial.println(F("\n\u21D2 Initializing RTC..."));
  RtcWave.Begin();

  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

  if (!RtcWave.IsDateTimeValid())
  {
    if (RtcWave.LastError() != 0)
    {
      // we have a communications error
      // see https://www.arduino.cc/en/Reference/WireEndTransmission for
      // what the number means
      Serial.print(F("  \u00BB RTC communications error = "));
      Serial.println(RtcWave.LastError());
      sf_Wave.fatalBlink(CRGB::Purple);
    } else {
      Serial.println(F("  \u00BB RTC lost confidence in the DateTime!"));
      RtcWave.SetDateTime(compiled);
    }
  }

  if (!RtcWave.GetIsRunning())
  {
    Serial.println(F("  \u00BB RTC was not actively running, starting now"));
    RtcWave.SetIsRunning(true);
  }

  RtcDateTime nowRTC = RtcWave.GetDateTime();
  if (nowRTC < compiled)
  {
    Serial.println(F("  \u00BB RTC is older than compile time!  (Updating DateTime)"));
    RtcWave.SetDateTime(compiled);
  }

  // never assume the RtcWave was last configured by you, so
  // just clear them to your needed state
  RtcWave.Enable32kHzPin(false);
  RtcWave.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne);

  Serial.println(F("  \u00BB RTC initialized with success."));

  // Checks the Start Time
  for (int i = 0; i < 6; i++) {
    _StartTimeArray[i] = StartTimeArray[i];
    if ((_StartTimeArray[0] > 31) || (_StartTimeArray[1] > 12) || (_StartTimeArray[3] > 24) || (_StartTimeArray[4] > 60) || (_StartTimeArray[5] > 60)) {
      Serial.println(F("  \u00BB Wrong Start Time"));
      sf_Wave.fatalBlink(CRGB::Purple);
    }
  }

  for (int i = 0; i < 3; i++) {
    _AverageTimeArray[i] = AverageTimeArray[i];
    if ((_AverageTimeArray[0] > 24) || (_AverageTimeArray[1] > 60) || (_AverageTimeArray[2] > 60)) {
      Serial.println(F("  \u00BB Wrong Average Time"));
      sf_Wave.fatalBlink(CRGB::Purple);
    }
  }

  _SampleLengthMinutes = SampleLengthMinutes;
  _TotalCount = _SampleLengthMinutes * 60 * _SampleRate;

  Serial.print(F("  \u00BB Current Date: "));
  Serial.print(String(nowRTC.Day()) + "/" + String(nowRTC.Month()) + "/" + String(nowRTC.Year() % 100));
  Serial.print(" ");
  Serial.println(String(nowRTC.Hour()) + ":" + String(nowRTC.Minute()) + ":" + String(nowRTC.Second()));

  Serial.print(F("  \u00BB Starting Date: "));
  Serial.print(String(_StartTimeArray[0]) + "/" + String(_StartTimeArray[1]) + "/" + String(_StartTimeArray[2]));
  Serial.print(" ");
  Serial.println(String(_StartTimeArray[3]) + ":" + String(_StartTimeArray[4]) + ":" + String(_StartTimeArray[5]));

  if ((_StartTimeArray[2] < nowRTC.Year() % 100) || ((_StartTimeArray[2] == nowRTC.Year() % 100) && ((_StartTimeArray[1] == nowRTC.Month()) && (_StartTimeArray[0] < nowRTC.Day()) || (_StartTimeArray[1] < nowRTC.Month()))) || ((_StartTimeArray[2] == nowRTC.Year() % 100) && (_StartTimeArray[1] == nowRTC.Month()) && (_StartTimeArray[0] == nowRTC.Day()) && ((_StartTimeArray[3] < nowRTC.Hour()) || ((_StartTimeArray[3] == nowRTC.Hour()) && (_StartTimeArray[4] < nowRTC.Minute()))))) {
    Serial.println(F("  \u00BB Starting Date is in the past..."));
    wrongDateWave = true;
  }

  if ((_StartTimeArray[1] == 4) || (_StartTimeArray[1] == 11) || (_StartTimeArray[1] == 9 ) || (_StartTimeArray[1] == 6)) {
    if (_StartTimeArray[1] > 30) {
      Serial.println(F("  \u00BB Starting Month with only 30 days"));
      wrongDateWave = true;
    }
  }

  if (_StartTimeArray[1] != 2) {
    if (_StartTimeArray[1] > 31) {
      Serial.println(F("  \u00BB Starting Month with only 31 days"));
      wrongDateWave = true;
    }
  }

  if (WaveBuoyFunctions::checkLeapYear(_StartTimeArray[2])) {
    if (_StartTimeArray[1] == 2) {
      if (_StartTimeArray[0] > 29) {
        Serial.println(F("  \u00BB Starting Month with only 29 days (Leap Year)"));
        wrongDateWave = true;
      }
    }
  } else {
    if (_StartTimeArray[1] == 2) {
      if (_StartTimeArray[0] > 28) {
        Serial.println(F("  \u00BB Starting Month with only 28 days (Normal Year)"));
        wrongDateWave = true;
      }
    }
  }

  Serial.println(F("  \u00BB Waiting for the right time to start..."));

  // Waits for the right time to start
  while (true) {
    RtcDateTime nowRTC = RtcWave.GetDateTime();
    if (wrongDateWave) {
      if (nowRTC.Second() == 0) {

        _DayNow = nowRTC.Day();
        _MonthNow = nowRTC.Month();
        _YearNow = nowRTC.Year() % 100;
        _HourNow = nowRTC.Hour();
        _MinuteNow = nowRTC.Minute();
        _SecondNow = nowRTC.Second();

        break;
      }

    } else {
      if ((nowRTC.Day() == _StartTimeArray[0]) && (nowRTC.Month() == _StartTimeArray[1]) && (nowRTC.Year() % 100 == _StartTimeArray[2]) && (nowRTC.Hour() == _StartTimeArray[3]) && (nowRTC.Minute() == _StartTimeArray[4]) && (nowRTC.Second() == _StartTimeArray[5])) {

        _DayNow = nowRTC.Day();
        _MonthNow = nowRTC.Month();
        _YearNow = nowRTC.Year() % 100;
        _HourNow = nowRTC.Hour();
        _MinuteNow = nowRTC.Minute();
        _SecondNow = nowRTC.Second();

        break;
      }
    }
    delay(1);
  }

  setTime(_HourNow, _MinuteNow, _SecondNow, _DayNow, _MonthNow, _YearNow); // Updates the "internal" timer
  prevTimeStampWave = now();
  _HourNowPrev = _HourNow;

  if (_SleepMode) {

    uint8_t Alarm_Minutes;

    // Function to sort the Alarm_WakeUp_Minutes array in ascending order
    float holder;
    for (int i = 0; i < Alarm_WakeUp_Minutes_Size; i++) {
      for (int k = 0; k < Alarm_WakeUp_Minutes_Size - 1; k++) {
        if (Alarm_WakeUp_Minutes[k] > Alarm_WakeUp_Minutes[k + 1]) {
          holder = Alarm_WakeUp_Minutes[k + 1];
          Alarm_WakeUp_Minutes[k + 1] = Alarm_WakeUp_Minutes[k];
          Alarm_WakeUp_Minutes[k] = holder;
        }
      }
    }

    // Function where selects the minute from the Alarm_WakeUp_Minutes array that the WaveBuoy is going to sleep based on the current RTC timestamp
    for (int i = 0; i < Alarm_WakeUp_Minutes_Size; i++) {
      if ((_MinuteNow  + _SampleLengthMinutes) >= Alarm_WakeUp_Minutes[i]) {
        if (i == Alarm_WakeUp_Minutes_Size - 1) {
          Alarm_Minutes = Alarm_WakeUp_Minutes[0];
        } else {
          Alarm_Minutes = Alarm_WakeUp_Minutes[i + 1];
        }
      }
    }

    if (Alarm_Minutes == 0) {
      Alarm_Minutes = 60;
    }

    Alarm_Minutes = Alarm_Minutes - 1;

    Serial.println("  \u00BB RTC WakeUP Minute: " + String(Alarm_Minutes));

    DS3231AlarmOne alarm1(
      0,
      0,
      Alarm_Minutes,
      0,
      DS3231AlarmOneControl_MinutesSecondsMatch);

    RtcWave.SetAlarmOne(alarm1);
    RtcWave.LatchAlarmsTriggeredFlags();

    // Initializes the ULP
    Serial.println(F("\n\u21D2 Initializing ULP..."));
    esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
    if (wakeupCause != ESP_SLEEP_WAKEUP_ULP)
    {
      Serial.println(F("  \u00BB Not ULP wakeup (First boot)"));
      WaveBuoyFunctions::init_ulp_program();
    } else {
      Serial.println(F("  \u00BB ULP wakeup"));
      //Start the program
      esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
      ESP_ERROR_CHECK(err);
    }
  }
}

// Start MPU9250
void WaveBuoyFunctions::start_MPU9250() {

  Serial.println(F("\n\u21D2 Initializing MPU9250..."));
  MPU9250Setting setting;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::FSR;
  setting.accel_fs_sel = ACCEL_FS_SEL::AFS;
  setting.accel_fchoice = AFC;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::ADLPF;
  setting.gyro_fs_sel = GYRO_FS_SEL:: GFS;
  setting.gyro_fchoice = GFC;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::GDLPF;
  setting.mag_output_bits = MAG_OUTPUT_BITS::MOB;

  if (!mpu.setup(MPU9250_ADDRESS)) {
    Serial.println(F("  \u00BB Starting MPU9250... failed!"));
    sf_Wave.fatalBlink(CRGB::Yellow);
  } else {
    Serial.println(F("  \u00BB MPU9250 initialized with success."));
  }

  // Checks if there is calibrated bias stored on the eeprom
  if (isnan(WaveBuoyFunctions::EEPROM_READ(500))) {
    Serial.println(F("\n################################"));
    Serial.println(F("##     MPU9250 Calibration    ##"));
    Serial.println(F("################################"));

    sf_Wave.displayLED(CRGB::Yellow, 15);

    Serial.println(F("\n\u21D2 Place the MPU9250 on a flat surface and wait."));
    delay(1000); Serial.print(F("  \u00BB Acc/Gyro calibration is going to start in: "));
    Serial.print(F("3...")); delay(1000); Serial.print(F("2...")); delay(1000); Serial.print(F("1...")); delay(1000); Serial.println(F(" NOW!")); delay(1000);

    mpu.calibrateAccelGyro();
    Serial.println(F("  \u00BB Acc/Gyro calibration is done."));

    sf_Wave.displayLED(CRGB::Purple, 15);

    Serial.println(F("\n\u21D2 Wave device in a figure eight until done!"));
    delay(1000); Serial.print(F("  \u00BB Mag calibration is going to start in: "));
    Serial.print(F("3...")); delay(1000); Serial.print(F("2...")); delay(1000); Serial.print(F("1...")); delay(1000); Serial.println(F(" NOW!")); delay(1000);

    mpu.calibrateMag();
    Serial.println(F("  \u00BB Mag calibration is done."));

    sf_Wave.displayLED(CRGB::Black, 0);


    for (int i = 0; i < 512; i++) { // Erase eeprom
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
    delay(500);

    WaveBuoyFunctions::EEPROM_WRITE(mpu.getAccBias(0), 0);
    WaveBuoyFunctions::EEPROM_WRITE(mpu.getAccBias(1), 8);
    WaveBuoyFunctions::EEPROM_WRITE(mpu.getAccBias(2), 16);

    WaveBuoyFunctions::EEPROM_WRITE(mpu.getGyroBias(0), 24);
    WaveBuoyFunctions::EEPROM_WRITE(mpu.getGyroBias(1), 32);
    WaveBuoyFunctions::EEPROM_WRITE(mpu.getGyroBias(2), 40);

    WaveBuoyFunctions::EEPROM_WRITE(mpu.getMagBias(0), 48);
    WaveBuoyFunctions::EEPROM_WRITE(mpu.getMagBias(1), 56);
    WaveBuoyFunctions::EEPROM_WRITE(mpu.getMagBias(2), 64);

    WaveBuoyFunctions::EEPROM_WRITE(mpu.getMagScale(0), 72);
    WaveBuoyFunctions::EEPROM_WRITE(mpu.getMagScale(1), 80);
    WaveBuoyFunctions::EEPROM_WRITE(mpu.getMagScale(2), 88);

    WaveBuoyFunctions::EEPROM_WRITE(1, 500);

  } else {
    mpu.setAccBias(0, WaveBuoyFunctions::EEPROM_READ(0));
    mpu.setAccBias(1, WaveBuoyFunctions::EEPROM_READ(8));
    mpu.setAccBias(2, WaveBuoyFunctions::EEPROM_READ(16));

    mpu.setGyroBias(0, WaveBuoyFunctions::EEPROM_READ(24));
    mpu.setGyroBias(1, WaveBuoyFunctions::EEPROM_READ(32));
    mpu.setGyroBias(2, WaveBuoyFunctions::EEPROM_READ(40));

    mpu.setMagBias(0, WaveBuoyFunctions::EEPROM_READ(48));
    mpu.setMagBias(1, WaveBuoyFunctions::EEPROM_READ(56));
    mpu.setMagBias(2, WaveBuoyFunctions::EEPROM_READ(64));

    mpu.setMagScale(0, WaveBuoyFunctions::EEPROM_READ(72));
    mpu.setMagScale(1, WaveBuoyFunctions::EEPROM_READ(80));
    mpu.setMagScale(2, WaveBuoyFunctions::EEPROM_READ(88));
  }
}

// Start BMP280
void WaveBuoyFunctions::start_BMP280() {

  Serial.println(F("\n\u21D2 Initializing BMP280..."));

  if (!bmp280.begin()) {
    Serial.println(F("  \u00BB Starting BMP280... failed!"));
    sf_Wave.fatalBlink(CRGB::Yellow);
  } else {
    Serial.println(F("  \u00BB BMP280 initialized with success."));

    // Default settings from datasheet.
    bmp280.setSampling(Adafruit_BMP280::MODE_FORCED,        // Operating Mode.
                       Adafruit_BMP280::SAMPLING_X16,       // Temp. oversampling
                       Adafruit_BMP280::SAMPLING_X16,       // Pressure oversampling
                       Adafruit_BMP280::FILTER_X16,         // Filtering.
                       Adafruit_BMP280::STANDBY_MS_4000);   // Standby time.

    bmp280.takeForcedMeasurement();                                   // Force the BMP280 to make a measure
    TX_Data_WaveBuoy.BMP280_Temperature = bmp280.readTemperature();   // Stores the BMP280 temperature in Celsius
    TX_Data_WaveBuoy.BMP280_Pressure = bmp280.readPressure() / 100;   // Stores the BMP280 pressure in hPa
  }
}

// Start MAX17048
void WaveBuoyFunctions::start_MAX17048() {

  Serial.println(F("\n\u21D2 Initializing MAX17048..."));

  // Set up the MAX17048 LiPo fuel gauge:
  if (lipoWave.begin() == false) // Connect to the MAX17048 using the default wire port
  {
    Serial.println(F("  \u00BB Starting MAX17048... failed!"));
    sf_Wave.fatalBlink(CRGB::Brown);
  } else {
    Serial.println(F("  \u00BB MAX17048 initialized with success."));


    bool RI = lipoWave.isReset(true); // Read the RI flag and clear it automatically if it is set

    if (RI)
    {
      RI = lipoWave.isReset(); // Read the RI flag
    }

    TX_Data_WaveBuoy.BatteryVoltage = lipoWave.getVoltage();    // Stores the battery voltage in Volts
  }
}

// Function used to check if the start year is a leap year
bool WaveBuoyFunctions::checkLeapYear(uint8_t the_year) {

  if (the_year % 4 != 0) {
    return false;
  }
  return true;
}

// Function used to update the timestamp
void WaveBuoyFunctions::updateTimeStamp() {

  if (timeStatus() != timeNotSet) {

    if (now() != prevTimeStampWave) {                     // Checks if the timestamp has changed
      _FirstSecondDetected = true;

      _HourNow = hour();
      _MinuteNow = minute();
      _SecondNow = second();

      char TimeBuffer[20];
      sprintf(TimeBuffer, "%02d:%02d:%02d", _HourNow, _MinuteNow, _SecondNow);
      TX_Data_WaveBuoy.RTC_Hour = String(TimeBuffer);     // Stores the Hour

      TX_Data_WaveBuoy.Epoch_Time = stampWave.timestamp(_YearNow, _MonthNow, _DayNow, _HourNow, _MinuteNow, _SecondNow);   // Converts timestamp into epoch time

      prevTimeStampWave = now();                          // updates the prevtime

      if (_HourNow != _HourNowPrev) {                                             // Checks if the hour has changed in order to create another CSV file
        xSemaphoreTake(SDWriteSemaphoreWave, portMAX_DELAY);                      // Takes the SDWriteSemaphoreWave before creating the new CSV file to avoid writing in a file that is closed
        WaveBuoyFunctions::CSV_File(true);
      }

      _HourNowPrev = _HourNow;
    }
  }
}

// Function used to get sensors data
void WaveBuoyFunctions::GetData(float WaveHeightThreshold, float Gravity_Acceleration, float Magnetic_Declination) {

  uint32_t Epoch_Time_Temp = TX_Data_WaveBuoy.Epoch_Time;

  if (mpu.update()) {
    float AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, Quaternion_Q0, Quaternion_Q1, Quaternion_Q2, Quaternion_Q3;

    // Gets accelerometer (g),  Gyro (deg/s) and Magnetometer (mG)
    AccX = mpu.getAccX();
    AccY = mpu.getAccY();
    AccZ = mpu.getAccZ();
    GyroX = mpu.getGyroX();
    GyroY = mpu.getGyroY();
    GyroZ = mpu.getGyroZ();
    MagX = mpu.getMagX() * 0.001; // Converts to G
    MagY = mpu.getMagY() * 0.001; // Converts to G
    MagZ = mpu.getMagZ() * 0.001; // Converts to G

    // Update the filter, which computes orientation with the right axis orientation for the MPU9250 (gyroX, -gyroY, -gyroZ, -accX, accY, accZ, magY, -magX, magZ)
    Madgwick_Filter.update(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ);

    // Get quaternions
    Madgwick_Filter.getQuaternion(&Quaternion_Q0, &Quaternion_Q1, &Quaternion_Q2, &Quaternion_Q3);

    // Multiply the quaternions by the rotation matrix
    R[0][0] = 2 * pow(Quaternion_Q0, 2) - 1 + 2 * pow(Quaternion_Q1, 2);
    R[0][1] = 2 * (Quaternion_Q1 * Quaternion_Q2 + Quaternion_Q0 * Quaternion_Q3);
    R[0][2] = 2 * (Quaternion_Q1 * Quaternion_Q3 - Quaternion_Q0 * Quaternion_Q2);
    R[1][0] = 2 * (Quaternion_Q1 * Quaternion_Q2 - Quaternion_Q0 * Quaternion_Q3);
    R[1][1] = 2 * pow(Quaternion_Q0, 2) - 1 + 2 * pow(Quaternion_Q2, 2);
    R[1][2] = 2 * (Quaternion_Q2 * Quaternion_Q3 + Quaternion_Q0 * Quaternion_Q1);
    R[2][0] = 2 * (Quaternion_Q1 * Quaternion_Q3 + Quaternion_Q0 * Quaternion_Q2);
    R[2][1] = 2 * (Quaternion_Q2 * Quaternion_Q3 - Quaternion_Q0 * Quaternion_Q1);
    R[2][2] = 2 * pow(Quaternion_Q0, 2) - 1 + 2 * pow(Quaternion_Q3, 2);

    // Get Tilt Acc
    float TiltAccXCompensated  = AccX * R[0][0] + AccY * R[1][0] + AccZ * R[2][0];
    float TiltAccYCompensated  = AccX * R[0][1] + AccY * R[1][1] + AccZ * R[2][1];
    float TiltAccZCompensated  = AccX * R[0][2] + AccY * R[1][2] + AccZ * R[2][2];

    // Subtracting Gravity
    float Lin_AccX = (TiltAccXCompensated - (2 * (Quaternion_Q1 * Quaternion_Q3 - Quaternion_Q0 * Quaternion_Q2))) * Gravity_Acceleration;
    float Lin_AccY = (TiltAccYCompensated - (2 * (Quaternion_Q0 * Quaternion_Q1 + Quaternion_Q2 * Quaternion_Q3))) * Gravity_Acceleration;
    float Lin_AccZ = (TiltAccZCompensated - (pow(Quaternion_Q0, 2) - pow(Quaternion_Q1, 2) - pow(Quaternion_Q2, 2) + pow(Quaternion_Q3, 2))) * Gravity_Acceleration;

    // Filtering Linear Acc with Low Pass Filter to remove/attenuate higher frequencies
    float Lin_filtered_AccZ = LowPassFilterAccZ->filterIn(Lin_AccZ);

    // 1st INTEGRAL (acc->vel)
    _velocity += (1 / (float)_SampleRate) * 0.5 * (_filtered_LinAccZ_prev + Lin_filtered_AccZ);
    _filtered_LinAccZ_prev = Lin_filtered_AccZ;

    // Filtering Velocity with High Pass Filter to remove lower frequencies
    float filtered_velocity = HighPassFilterVelocity->filterIn(_velocity);

    // 2nd INTEGRAL (vel->displacement)
    _displacement += (1 / (float)_SampleRate) * 0.5 * (_filtered_velocity_prev + filtered_velocity);
    _filtered_velocity_prev = filtered_velocity;

    // Filtering Displacement with High Pass Filter to remove lower frequencies
    float filtered_displacement = HighPassFilterDisplacement->filterIn(_displacement);

    // Detection of waves though and crest, height and period
    if ((filtered_displacement > _peak && _peak < _previous_peak)) {
      _previous_peak = _peak;
      _peak = filtered_displacement;
      _minimum = _previous_peak;

    } else if ((filtered_displacement < _peak && _peak > _previous_peak)) {
      _previous_peak = _peak;
      _peak = filtered_displacement;
      _maximum = _previous_peak;
      float height = _maximum - _minimum;

      // Avoid/eliminate small oscillations in displacement
      if (height > WaveHeightThreshold) {
        _Height_Average_Samples[_samplesPeak] = height;
        TX_Data_WaveBuoy.Height = height;

        TX_Data_WaveBuoy.WavePeriod = _index_peak * (1 / (float)_SampleRate); // Wave period in function of the sample rate
        _WavePeriod_AVG += TX_Data_WaveBuoy.WavePeriod;

        _samplesPeak++;                                                       //Number of wave peaks detected
      }
      _index_peak = 0;
    }

    _previous_peak = _peak;
    _peak = filtered_displacement;

    // Wave Direction (Yaw)
    _WaveDirection_AVG +=  Madgwick_Filter.getYaw() + Magnetic_Declination;

    _index_peak++;
    _indexIMU++;
  }

  if ((_AverageTimeArray[0] == 0) && (_AverageTimeArray[1] == 0) && (_AverageTimeArray[2] == 0)) {
    _AverageTimeArray[1] = 1;
  }

  _NumberOfCycles++;

  float SecondsAverage = (_AverageTimeArray[0] * 3600) + (_AverageTimeArray[1] * 60) + _AverageTimeArray[2];

  if ( _NumberOfCycles == (SecondsAverage * _SampleRate)) {

    if (_SerialDebug) {
      Serial.println("###################################");
      Serial.println("SamplesPeak: " + String(_samplesPeak));
      for (int i = 0; i < _samplesPeak; i++) {
        Serial.println("Height_Average_Samples: " + String(_Height_Average_Samples[i]));
      }
      Serial.println("###################################");
    }

    if (_samplesPeak > 0) {

      for (int i = 0; i < _samplesPeak; i++) {
        // Gets the sum of all the peaks detected
        _Height_Average_AVG += _Height_Average_Samples[i];
      }

      float holder;
      for (int i = 0; i < _samplesPeak; i++)
        for (int k = 0; k < _samplesPeak - 1; k++) {
          if (_Height_Average_Samples[k] < _Height_Average_Samples[k + 1]) {
            holder = _Height_Average_Samples[k + 1];
            _Height_Average_Samples[k + 1] = _Height_Average_Samples[k];
            _Height_Average_Samples[k] = holder;
          }
        }

      if (int(_samplesPeak / 3) > 0) {
        float Height_Sign_Average = 0;
        for (int i = 0; i < int(_samplesPeak / 3); i++) {
          Height_Sign_Average += _Height_Average_Samples[i];
        }
        TX_Data_WaveBuoy.Height_Sign = Height_Sign_Average / int(_samplesPeak / 3);
      } else {
        TX_Data_WaveBuoy.Height_Sign = 0;
      }

      // Calculates the average dividing the variables sum by the correct index
      TX_Data_WaveBuoy.Height_Average = _Height_Average_AVG / _samplesPeak;

      TX_Data_WaveBuoy.Height_Max = 2 * TX_Data_WaveBuoy.Height_Sign;
      _WavePeriod_AVG = _WavePeriod_AVG / _samplesPeak;

      _samplesPeak = 0;    // Clears the number of peaks detected before new loop

    } else {
      // If no wave peak was detected, clears the variables with the averages
      TX_Data_WaveBuoy.Height = 0;
      TX_Data_WaveBuoy.Height_Max = 0;
      TX_Data_WaveBuoy.Height_Average = 0;
      TX_Data_WaveBuoy.Height_Sign = 0;
      TX_Data_WaveBuoy.WavePeriod = 0;
    }

    sf_Wave.displayLED(CRGB::Green, 15);

    // Calculates the average dividing the variables sum by the correct index
    _WaveDirection_AVG = _WaveDirection_AVG / _indexIMU;
    TX_Data_WaveBuoy.WaveDirection = _WaveDirection_AVG;

    bmp280.takeForcedMeasurement();
    TX_Data_WaveBuoy.BMP280_Temperature = bmp280.readTemperature();
    TX_Data_WaveBuoy.BMP280_Pressure = bmp280.readPressure() / 100;

    TX_Data_WaveBuoy.BatteryVoltage = lipoWave.getVoltage();

    if (_SerialDebug) {
      Serial.println("###################################");
      Serial.println("Height_Average: " + String(TX_Data_WaveBuoy.Height_Average));
      Serial.println("Height_Max : " + String(TX_Data_WaveBuoy.Height_Max));
      Serial.println("Height_Sign : " + String(TX_Data_WaveBuoy.Height_Sign));
      Serial.println("BM280_Temperature : " + String(TX_Data_WaveBuoy.BMP280_Temperature));
      Serial.println("BM280_Pressure : " + String(TX_Data_WaveBuoy.BMP280_Pressure));
      Serial.println("Battery_Voltage : " + String(TX_Data_WaveBuoy.BatteryVoltage));
      if (_GPS_NO_DATA == 0) {
        Serial.print("Latitude: ");
        Serial.println(TX_Data_WaveBuoy.Latitude, 5);
        Serial.print("Longitude: ");
        Serial.println(TX_Data_WaveBuoy.Longitude, 5);
      }
      Serial.println("###################################");
    }

    // Creates the Cayenne LPP package (https://www.thethingsnetwork.org/docs/devices/arduino/api/cayennelpp/)
    lppWave.reset();
    lppWave.addUnixTime(Epoch_Time_Temp);                                // uint32_t
    lppWave.addBatteryVoltage(TX_Data_WaveBuoy.BatteryVoltage);          // in volts, positive values
    lppWave.addWaveDirection(_WaveDirection_AVG);                        // 0 to 360 degrees
    lppWave.addBMP280Temperature(TX_Data_WaveBuoy.BMP280_Temperature);   // -40 to +85 celsius
    lppWave.addBMP280Pressure(TX_Data_WaveBuoy.BMP280_Pressure);         // 300 to 1100 hPa
    lppWave.addHeightAverage(TX_Data_WaveBuoy.Height_Average);           // in meters, positive values
    lppWave.addHeightMax(TX_Data_WaveBuoy.Height_Max);                   // in meters, positive values
    lppWave.addHeightSign(TX_Data_WaveBuoy.Height_Sign);                 // in meters, positive values
    lppWave.addWavePeriod(_WavePeriod_AVG);                              // in seconds, positive values
    lppWave.addGPS(TX_Data_WaveBuoy.Latitude, TX_Data_WaveBuoy.Longitude, _GPS_NO_DATA); // deg

    uint8_t* payload = lppWave.getBuffer();                              // gets LPP package size
    String payloadString;

    // Converts into a string the LPP package before sending
    for (int i = 0; i < lppWave.getSize(); i++) {
      payloadString += (char)payload[i];
    }

    WaveBuoyFunctions::SendLoRaPacket(payloadString);

    sf_Wave.displayLED(CRGB::Black, 0);

    // Clears the variables before the new loop
    _height_prev = 0;
    memset(_Height_Average_Samples, 0, sizeof(_Height_Average_Samples));

    _NumberOfCycles = 0;
    _Height_Average_AVG = 0;
    _WavePeriod_AVG = 0;
    _WaveDirection_AVG = 0;
    _indexIMU = 0;
  }

  if (!_FirstSecondDetected) {
    xQueueReset( DataQueueWaveBuoyWriteSD );
  } else {
    if (xQueueSend( DataQueueWaveBuoyWriteSD, ( void * ) &TX_Data_WaveBuoy, portMAX_DELAY ) != pdPASS ) { //portMAX_DELAY
      Serial.println(F("xQueueSend is not working"));
    }
  }
}

//Function used to wake up the ESP32 using the ULP
void WaveBuoyFunctions::init_ulp_program(void) {
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);

  assert(rtc_gpio_desc[gpio_WakeUpWave].reg && "GPIO used for wake up must be an RTC IO");
  ulp_next_edge = 0;
  ulp_io_number = rtc_gpio_desc[gpio_WakeUpWave].rtc_num; // Map from GPIO# to RTC_IO

  // Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
  rtc_gpio_init(gpio_WakeUpWave);
  rtc_gpio_set_direction(gpio_WakeUpWave, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(gpio_WakeUpWave);
  rtc_gpio_pullup_en(gpio_WakeUpWave);
  rtc_gpio_hold_en(gpio_WakeUpWave);

  err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
  ESP_ERROR_CHECK(err);
}

// Function used to write on the eeprom
void WaveBuoyFunctions::EEPROM_WRITE(float dataToStore, unsigned int address) {
  for (byte i = 0; i < sizeof(dataToStore); i++) {
    EEPROM.write(address + i, reinterpret_cast<byte*>(&dataToStore)[i]);
  }
  EEPROM.commit();
}

// Function used to read on the eeprom
float WaveBuoyFunctions::EEPROM_READ(unsigned int address) {
  float dataToRead;
  for (byte i = 0; i < sizeof(dataToRead); i++) {
    reinterpret_cast<byte*>(&dataToRead)[i] = EEPROM.read(address + i);
  }
  return dataToRead;
}

// Function used to send the LoRa packet
void WaveBuoyFunctions::SendLoRaPacket (String payload) {

  // send in async / non-blocking mode
  LoRa.beginPacket();
  LoRa.write(_WaveBuoy_ID);               // WaveBuoy ID
  LoRa.write(payload.length());           // add payload length
  LoRa.print(payload);                    // add payload
  LoRa.endPacket(true);                   // finish packet and send it
}


void WaveBuoyFunctions::decode41(byte payload[], boolean SleepModeGPS) {

  gpsLat.b[3] = payload[23];
  gpsLat.b[2] = payload[24];
  gpsLat.b[1] = payload[25];
  gpsLat.b[0] = payload[26];

  gpsLon.b[3] = payload[27];
  gpsLon.b[2] = payload[28];
  gpsLon.b[1] = payload[29];
  gpsLon.b[0] = payload[30];

  if ( (payload[11] << 8) + payload[12] > 2020) {
    canSleepWave = true;
  }
  else {
    canSleepWave = false;
  }

  if ((gpsLat.i == 0) && (gpsLon.i == 0)) {
    _GPS_NO_DATA = 1;
  } else {
    TX_Data_WaveBuoy.Latitude = gpsLat.i / pow(10, 7);
    TX_Data_WaveBuoy.Longitude = gpsLon.i / pow(10, 7);
    _GPS_NO_DATA = 0;
  }

  if (SleepModeGPS) {
    if (((payload[4] & 7) >= 5) && canSleepWave) {
      digitalWrite(GPS_ONOFF, HIGH);
      vTaskDelay( 200 / portTICK_PERIOD_MS);
      digitalWrite(GPS_ONOFF, LOW);

      // Waits before waking up again the GPS
      vTaskDelay( 59800 / portTICK_PERIOD_MS);
    }
  }
}

// Function used to get the GPS Location
void WaveBuoyFunctions::GetGPSData(boolean SleepModeGPS) {

  while (digitalRead(GPS_WakeUP) == LOW) { // Waits until the GPS is wake
    digitalWrite(GPS_ONOFF, LOW);
    digitalWrite(GPS_ONOFF, HIGH);
    vTaskDelay( 200 / portTICK_PERIOD_MS);;
    digitalWrite(GPS_ONOFF, LOW);
    vTaskDelay( 1000 / portTICK_PERIOD_MS);
  }

  if (gpsSerialWave.available()) {
    bool validPackidge = false;
    byte head;
    byte payloadSize[2];

    byte checkSum[2];
    byte tail[2];
    //int sizeOfPayload = 0;
    gpsSerialWave.readBytes(&head, 1);
    if (head == 160) {
      gpsSerialWave.readBytes(&head, 1);
      if (head == 162) {
        gpsSerialWave.readBytes( payloadSize, 2);
        SizeWave = (payloadSize[0] << 8) +  payloadSize[1];
        gpsSerialWave.readBytes(DataWave, SizeWave);
        gpsSerialWave.readBytes(checkSum, 2);
        gpsSerialWave.readBytes(tail, 2);
        int valCheckSum = (checkSum[0] << 8) + checkSum[1];
        int payloadSum = 0;
        for (int i = 0; i < SizeWave; i++) {
          payloadSum += DataWave[i];
          payloadSum = payloadSum & 0x7FFF;
        }
        if (tail[0] == 176 && tail[1] == 179 && payloadSum == valCheckSum) {
          validPackidge = true;
        }
      }
    }

    if (validPackidge) {
      if (DataWave[0] == 41) {
        WaveBuoyFunctions::decode41(DataWave, SleepModeGPS);
      }
    }
  }
}

// Function used to create the CSV file
void WaveBuoyFunctions::CSV_File(boolean GetNewValue) {

  if (GetNewValue) {                                                        // If this function was called when the hour changed, the date is updated

    _DayNow  = day();
    _MonthNow = month();
    _YearNow = year() % 100;

    _HourNow = hour();
    _MinuteNow = minute();
    _SecondNow = second();
  }

  // Create filename scheme ====================================================================
  char DateBuffer[20];
  sprintf(DateBuffer, "%02d-%02d-%02d", _DayNow, _MonthNow, _YearNow);
  TX_Data_WaveBuoy.RTC_Date = DateBuffer;
  String DayFolder = "/" + String(DateBuffer);

  char TimeBuffer[20];
  sprintf(TimeBuffer, "%02d:%02d:%02d", _HourNow, _MinuteNow, _SecondNow);
  TX_Data_WaveBuoy.RTC_Hour = TimeBuffer;
  sprintf(TimeBuffer, "%02dH-%02dM-%02dS", _HourNow, _MinuteNow, _SecondNow);

  if (!SD_MMC.exists(DayFolder)) {
    if (!SD_MMC.mkdir(DayFolder)) {
      Serial.println(F("  \u00BB Create DayFolder failed!"));
      sf_Wave.fatalBlink(CRGB::Red);
    }
  }

  String FileDir = DayFolder + "/" + String(TimeBuffer) + ".csv";
  FileDir.toCharArray(filenameWave, 30);

  if (SD_MMC.exists(filenameWave)) {
    Serial.println(F("  \u00BB A file with that name already exists."));
    sf_Wave.fatalBlink(CRGB::Red);
  }

  // Create file and prepare it ============================================================
  logFileWriteWave = SD_MMC.open(filenameWave, FILE_WRITE);
  if ( ! logFileWriteWave ) {
    Serial.print(F("  \u00BB Couldnt create "));
    Serial.println(filenameWave);
    sf_Wave.fatalBlink(CRGB::Red);
  }

  Serial.print(F("\n\u21D2 Ready to write to: "));
  Serial.println(filenameWave);
  Serial.println();

  String CSV_Header[ARRAY_SIZE(CSV_Header_WaveBuoy)];

  for (int i = 0; i < ARRAY_SIZE(CSV_Header_WaveBuoy); i++) {
    CSV_Header[i] = CSV_Header_WaveBuoy[i];
  }

  //Column labels
  for (size_t i = 0; i < ARRAY_SIZE(CSV_Header); i++) {   // Function used to write the csv file headers
    logFileWriteWave.print(CSV_Header[i]);
    if (i != ARRAY_SIZE(CSV_Header) - 1) {
      logFileWriteWave.print(",");
    }
  }
  logFileWriteWave.print("\n");
  logFileWriteWave.close();

  _justOnce = true;

  xSemaphoreGive(SDWriteSemaphoreWave);                    // Semaphore give up allowing the SD Card and CSV writing
}

// Function used to write the sensors data on a CSV File
void WaveBuoyFunctions::WriteSD() {

  if ( xQueueReceive( DataQueueWaveBuoyWriteSD, &( RX_Data_WaveBuoy ), portMAX_DELAY ) != pdPASS ) { //portMAX_DELAY
    Serial.println(F("xQueueRecieve is not working"));
  }

  if (xSemaphoreTake(SDWriteSemaphoreWave, portMAX_DELAY ) == pdTRUE ) {

    if (_SamplesCount != _TotalCount) {       // This task will contine until the total number of samples is not reached (in sleep mode)

      // Variables to write in the CSV file
      String CSV_Values_First[] = {String(_WaveBuoy_ID), String(_SampleRate), RX_Data_WaveBuoy.RTC_Date};

      String CSV_Values_Second[] = {RX_Data_WaveBuoy.RTC_Hour};

      float GPSData [] = {RX_Data_WaveBuoy.Latitude, RX_Data_WaveBuoy.Longitude};

      float CSV_Values_Minute_WaveBuoy[] = {RX_Data_WaveBuoy.BatteryVoltage, RX_Data_WaveBuoy.BMP280_Temperature,
                                            RX_Data_WaveBuoy.BMP280_Pressure, RX_Data_WaveBuoy.WaveDirection
                                           };

      float CSV_Values_Always_WaveBuoy[] = {RX_Data_WaveBuoy.Height, RX_Data_WaveBuoy.Height_Average,
                                            RX_Data_WaveBuoy.Height_Max, RX_Data_WaveBuoy.Height_Sign, RX_Data_WaveBuoy.WavePeriod
                                           };

      WaveBuoyFunctions::ParseDataCSV (CSV_Values_First,  ARRAY_SIZE(CSV_Values_First), CSV_Values_Second, ARRAY_SIZE(CSV_Values_Second),
                                       CSV_Values_Minute_WaveBuoy, ARRAY_SIZE(CSV_Values_Minute_WaveBuoy), CSV_Values_Always_WaveBuoy,
                                       ARRAY_SIZE(CSV_Values_Always_WaveBuoy), GPSData);

      // If Sleep Mode is enabled, counts the number of samples until going to sleep
      if (_SleepMode) {
        _SamplesCount++;
      }

      _SamplesCountToWrite++;

    } else {

      Serial.println(F("\n################################"));
      Serial.println(F("##       Logging finished     ##"));
      Serial.println(F("################################"));

      Serial.println(F("\n\u21D2 Sleep Mode until RTC alarm is fired."));

      digitalWrite(GPS_WakeUP, HIGH);
      delay(200);
      digitalWrite(GPS_WakeUP, LOW);

      digitalWrite(TurnSensorsOn, LOW);

      sf_Wave.displayLED(CRGB::Black, 0);

      ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
      esp_sleep_enable_ulp_wakeup();
      delay(2000);
      esp_deep_sleep_start();
    }

    xSemaphoreGive(SDWriteSemaphoreWave);
  }
}

void WaveBuoyFunctions::ParseDataCSV (String* CSV_Values_First,  int CSV_Values_First_Size, String* CSV_Values_Second, int CSV_Values_Second_Size,  float* CSV_Values_Minute, int CSV_Values_Minute_Size, float* CSV_Values_Always, int CSV_Values_Always_Size, float GPSData [2]) {

  logFileWriteWave = SD_MMC.open(filenameWave, FILE_APPEND);  // Opens the CSV file in append mode

  // Variables that only is going to be writen once on the CSV file
  if (_justOnce) {
    for (int i = 0; i < CSV_Values_First_Size; i++) {
      logFileWriteWave.print(CSV_Values_First[i]);
      logFileWriteWave.print(',');
    }

    for (int i = 0; i < CSV_Values_Second_Size; i++) {
      logFileWriteWave.print(CSV_Values_Second[i]);
      logFileWriteWave.print(',');
    }

    for (int i = 0; i < CSV_Values_Minute_Size; i++) {
      logFileWriteWave.print(CSV_Values_Minute[i]);
      logFileWriteWave.print(',');
    }

    for (int i = 0; i < 2; i++) {
      if (_GPS_NO_DATA == 0) {
        logFileWriteWave.print(GPSData[i], 5);
      } else {
        logFileWriteWave.print("NO DATA");
      }
      logFileWriteWave.print(',');
    }

    for (int i = 0; i < CSV_Values_Always_Size; i++) {
      logFileWriteWave.print(CSV_Values_Always[i]);
      logFileWriteWave.print(',');
    }

    _justOnce = false;

  } else {

    for (int i = 0; i < CSV_Values_First_Size; i++) {
      logFileWriteWave.print("");
      logFileWriteWave.print(',');
    }

    for (int i = 0; i < CSV_Values_Second_Size; i++) {
      if (_SamplesCountToWrite % _SampleRate) {
        logFileWriteWave.print("");
      } else {
        logFileWriteWave.print(CSV_Values_Second[i]);
        _SamplesCountToWrite = 0;
      }
      logFileWriteWave.print(',');
    }

    for (int i = 0; i < CSV_Values_Minute_Size; i++) {
      if (_MinuteNow != _MinuteNowPrev) {
        logFileWriteWave.print(CSV_Values_Minute[i]);
      } else {
        logFileWriteWave.print("");
      }
      logFileWriteWave.print(',');
    }

    for (int i = 0; i < 2; i++) {
      if (_MinuteNow != _MinuteNowPrev) {
        if (_GPS_NO_DATA == 0) {
          logFileWriteWave.print(GPSData[i], 5);
        } else {
          logFileWriteWave.print("NO DATA");
        }
      } else {
        logFileWriteWave.print("");
      }
      logFileWriteWave.print(',');
    }

    for (int i = 0; i < CSV_Values_Always_Size; i++) {
      logFileWriteWave.print(CSV_Values_Always[i]);
      logFileWriteWave.print(',');
    }
  }

  _MinuteNowPrev = _MinuteNow;

  logFileWriteWave.print('\n');
  logFileWriteWave.close();
}
