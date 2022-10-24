#include "SharedFunctions.h"
#include "WaterQualityBuoyFunctions.h"

SharedFunctions sf_WaterQuality;

RtcDS3231<TwoWire> RtcWaterQuality(Wire);                 // Create RTC
File logFileWriteWaterQuality;                            // Create SD Card
SFE_MAX1704X lipoWaterQuality(MAX1704X_MAX17048);         // Create a MAX17048

// GPIO used for wake up.
gpio_num_t gpio_WakeUpWaterQuality = GPIO_NUM_36;

// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

// Starts Sensors Objects
Ezo_board* ORP;
Ezo_board* RTD;
Ezo_board* pH;
Ezo_board* EC;
Ezo_board* DO;

char filenameWaterQuality[30];                            // Buffer used to store the name of CSV file

// CSV file headers
String CSV_Header_WaterQualityBuoy[] = {"WaterQualityBuoy_ID", "SampleTime (sec)", "RTC_Date", "RTC_Hour", "BatteryVoltage (V)",
                                        "Latitude", "Longitude", "Oxidation-Reduction Potential (mV)", "RTD (C)", "pH",
                                        "Salinity (ppt)", "Dissolved Oxygen (mg/L)"
                                       };

// Data type for Queue item
struct Data_WaterQualityBuoy {
  String RTC_Date;
  String RTC_Hour;
  String RTC_Hour_Temp;
  uint32_t Epoch_Time;
  float BatteryVoltage;
  float Latitude = 0;
  float Longitude = 0;
  float pH_Value;
  float DO_Value;
  float EC_Value;
  float ORP_Value;
  float RTD_Value;
} TX_Data_WaterQualityBuoy, RX_Data_WaterQualityBuoy;


boolean wrongDateWaterQuality = false;                     // When the starting date is not defined or wrong

// Objects definition
timestamp32bits stampWaterQuality = timestamp32bits();
CayenneLPP lppWaterQuality(160);
HardwareSerial gpsSerialWaterQuality(2);

byte DataWaterQuality[1000];
int SizeWaterQuality;

bool canSleepWaterQuality = false;

union
{
  int i;
  byte b[4];
} gpsLon, gpsLat;

// Declare Queue data type for FreeRTOS
QueueHandle_t DataQueueWaterQualityBuoyWriteSD;
// Create Interrupt Semaphores
SemaphoreHandle_t SDWriteSemaphoreWaterQuality;

time_t prevTimeStampWaterQuality = 0;     // Stores the previous timestamp

// WaterQualityBuoy setup
void WaterQualityBuoyFunctions::setup(boolean SerialDebug, boolean SleepMode, uint8_t WaterQualityBuoy_ID, uint8_t SampleTime) {

  WiFi.mode(WIFI_OFF);                                           // Turn Off WiFi
  Wire.begin();

  SDWriteSemaphoreWaterQuality = xSemaphoreCreateBinary();       // Create semaphore

  //Queue Setup
  DataQueueWaterQualityBuoyWriteSD = xQueueCreate(100, sizeof(Data_WaterQualityBuoy));    // Queue with the number of struct variables

  _SerialDebug = SerialDebug;
  _SleepMode = SleepMode;

  _WaterQualityBuoy_ID = WaterQualityBuoy_ID;
  _SampleTime = SampleTime;
}


// Start GPS
void WaterQualityBuoyFunctions::start_GPS() {

  Serial.println(F("\n\u21D2 Initializing GPS..."));

  //Boot the GPS module
  Serial.println(F("  \u00BB Booting it up..."));
  sf_WaterQuality.displayLED(CRGB::Blue, 15);

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
  gpsSerialWaterQuality.begin(115200, SERIAL_8N1, GPS_Tx, GPS_Rx);
  while (!gpsSerialWaterQuality);
  Serial.println(F("  \u00BB GPS Serial is connected!"));

  sf_WaterQuality.displayLED(CRGB::Black, 0);
}

// Start RTC
void WaterQualityBuoyFunctions::start_RTC(uint8_t StartTimeArray[6], uint8_t AverageTimeArray[3], uint8_t SampleLengthMinutes, uint8_t* Alarm_WakeUp_Minutes, int Alarm_WakeUp_Minutes_Size) {

  Serial.println(F("\n\u21D2 Initializing RTC..."));
  RtcWaterQuality.Begin();

  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

  if (!RtcWaterQuality.IsDateTimeValid())
  {
    if (RtcWaterQuality.LastError() != 0)
    {
      // we have a communications error
      // see https://www.arduino.cc/en/Reference/WireEndTransmission for
      // what the number means
      Serial.print(F("  \u00BB RTC communications error = "));
      Serial.println(RtcWaterQuality.LastError());
      sf_WaterQuality.fatalBlink(CRGB::Purple);
    } else {
      Serial.println(F("  \u00BB RTC lost confidence in the DateTime!"));
      RtcWaterQuality.SetDateTime(compiled);
    }
  }

  if (!RtcWaterQuality.GetIsRunning())
  {
    Serial.println(F("  \u00BB RTC was not actively running, starting now"));
    RtcWaterQuality.SetIsRunning(true);
  }

  RtcDateTime nowRTC = RtcWaterQuality.GetDateTime();
  if (nowRTC < compiled)
  {
    Serial.println(F("  \u00BB RTC is older than compile time!  (Updating DateTime)"));
    RtcWaterQuality.SetDateTime(compiled);
  }

  // never assume the RtcWaterQuality was last configured by you, so
  // just clear them to your needed state
  RtcWaterQuality.Enable32kHzPin(false);
  RtcWaterQuality.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne);

  Serial.println(F("  \u00BB RTC initialized with success."));

  // Checks the Start Time
  for (int i = 0; i < 6; i++) {
    _StartTimeArray[i] = StartTimeArray[i];
    if ((_StartTimeArray[0] > 31) || (_StartTimeArray[1] > 12) || (_StartTimeArray[3] > 24) || (_StartTimeArray[4] > 60) || (_StartTimeArray[5] > 60)) {
      Serial.println(F("  \u00BB Wrong Start Time"));
      sf_WaterQuality.fatalBlink(CRGB::Purple);
    }
  }

  for (int i = 0; i < 3; i++) {
    _AverageTimeArray[i] = AverageTimeArray[i];
    if ((_AverageTimeArray[0] > 24) || (_AverageTimeArray[1] > 60) || (_AverageTimeArray[2] > 60)) {
      Serial.println(F("  \u00BB Wrong Average Time"));
      sf_WaterQuality.fatalBlink(CRGB::Purple);
    }
  }

  _SampleLengthMinutes = SampleLengthMinutes;
  _TotalCount = (_SampleLengthMinutes * 60) / _SampleTime;

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
    wrongDateWaterQuality = true;
  }

  if ((_StartTimeArray[1] == 4) || (_StartTimeArray[1] == 11) || (_StartTimeArray[1] == 9 ) || (_StartTimeArray[1] == 6)) {
    if (_StartTimeArray[1] > 30) {
      Serial.println(F("  \u00BB Starting Month with only 30 days"));
      wrongDateWaterQuality = true;
    }
  }

  if (_StartTimeArray[1] != 2) {
    if (_StartTimeArray[1] > 31) {
      Serial.println(F("  \u00BB Starting Month with only 31 days"));
      wrongDateWaterQuality = true;
    }
  }

  if (WaterQualityBuoyFunctions::checkLeapYear(_StartTimeArray[2])) {
    if (_StartTimeArray[1] == 2) {
      if (_StartTimeArray[0] > 29) {
        Serial.println(F("  \u00BB Starting Month with only 29 days (Leap Year)"));
        wrongDateWaterQuality = true;
      }
    }
  } else {
    if (_StartTimeArray[1] == 2) {
      if (_StartTimeArray[0] > 28) {
        Serial.println(F("  \u00BB Starting Month with only 28 days (Normal Year)"));
        wrongDateWaterQuality = true;
      }
    }
  }

  Serial.println(F("  \u00BB Waiting for the right time to start..."));

  while (true) {
    RtcDateTime nowRTC = RtcWaterQuality.GetDateTime();
    if (wrongDateWaterQuality) {
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
  prevTimeStampWaterQuality = now();

  _HourNowPrev = _HourNow;

  if (_SleepMode) {
    // Function where selects the minute from the Alarm_WakeUp_Minutes array that the LoPy4 is going to sleep based on the current RTC timestamp
    uint8_t Alarm_Minutes;

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

    RtcWaterQuality.SetAlarmOne(alarm1);
    RtcWaterQuality.LatchAlarmsTriggeredFlags();

    Serial.println(F("\n\u21D2 Initializing ULP..."));
    esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
    if (wakeupCause != ESP_SLEEP_WAKEUP_ULP)
    {
      Serial.println(F("  \u00BB Not ULP wakeup (First boot)"));
      WaterQualityBuoyFunctions::init_ulp_program();
    } else {
      Serial.println(F("  \u00BB ULP wakeup"));
      //Start the program
      esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
      ESP_ERROR_CHECK(err);
    }
  }
}

// Start MAX17048
void WaterQualityBuoyFunctions::start_MAX17048() {

  Serial.println(F("\n\u21D2 Initializing MAX17048..."));

  // Set up the MAX17048 LiPo fuel gauge:
  if (lipoWaterQuality.begin() == false) // Connect to the MAX17048 using the default wire port
  {
    Serial.println(F("  \u00BB Starting MAX17048... failed!"));
    sf_WaterQuality.fatalBlink(CRGB::Brown);
  } else {
    Serial.println(F("  \u00BB MAX17048 initialized with success."));
  }

  bool RI = lipoWaterQuality.isReset(true); // Read the RI flag and clear it automatically if it is set

  if (RI)
  {
    RI = lipoWaterQuality.isReset(); // Read the RI flag
  }
}

// Start WhiteBox Sensors with the right address
void WaterQualityBuoyFunctions::start_WhiteBoxT1() {

  ORP = new Ezo_board(98, "ORP");             // Range: -1019.9mV - 1019.9mV
  RTD = new Ezo_board(110, "RTD");            // Range: -200C - 850C
  pH = new Ezo_board(99, "pH");               // Range: 1 - 7
  EC = new Ezo_board(100, "EC");              // Range: Conductivity (10 μS/cm − 1 S/cm), Salinity (0 - 42 ppt)
  DO = new Ezo_board(97, "DO");               // Range: 0 − 100 mg/L
}

// Function used to check if the start year is a leap year
bool WaterQualityBuoyFunctions::checkLeapYear(uint8_t the_year) {

  if (the_year % 4 != 0) {
    return false;
  }
  return true;
}


// Function used to update the timestamp
void WaterQualityBuoyFunctions::updateTimeStamp() {

  if (timeStatus() != timeNotSet) {
    if (now() != prevTimeStampWaterQuality) {     // Checks if the timestamp has changed
      _FirstSecondDetected = true;

      _HourNow = hour();
      _MinuteNow = minute();
      _SecondNow = second();

      char TimeBuffer[20];
      sprintf(TimeBuffer, "%02d:%02d:%02d", _HourNow, _MinuteNow, _SecondNow);
      TX_Data_WaterQualityBuoy.RTC_Hour = String(TimeBuffer);   // Stores the Hour

      TX_Data_WaterQualityBuoy.Epoch_Time = stampWaterQuality.timestamp(_YearNow, _MonthNow, _DayNow, _HourNow, _MinuteNow, _SecondNow);   // Converts timestamp into epoch time

      prevTimeStampWaterQuality = now();                        // updates the prevtime

      if (_HourNow != _HourNowPrev) {                                        // Checks if the hour has changed in order to create another CSV file
        xSemaphoreTake(SDWriteSemaphoreWaterQuality, portMAX_DELAY);         // Takes the SDWriteSemaphoreWaterQuality before creating the new CSV file to avoid writing in a file that is closed
        WaterQualityBuoyFunctions::CSV_File(true);
      }

      _HourNowPrev = _HourNow;
    }
  }
}

// Gets Sensors Data
void WaterQualityBuoyFunctions::GetDataWhiteBoxT1() {

  ////////////////////////////1st Step////////////////////////////
  uint32_t Epoch_Time_Temp = TX_Data_WaterQualityBuoy.Epoch_Time;
  TX_Data_WaterQualityBuoy.RTC_Hour_Temp = TX_Data_WaterQualityBuoy.RTC_Hour;

  ORP->send_read_cmd();
  RTD->send_read_cmd();
  DO->send_read_with_salinity_comp(35.5);

  vTaskDelay( 815 / portTICK_PERIOD_MS );

  ////////////////////////////2nd Step////////////////////////////
  if (_SerialDebug) {
    Serial.println("###################################");
  }
  ORP->receive_read_cmd();
  _ORP_RESPONSE = ORP->get_error();
  if (_SerialDebug) {
    Serial.print("ORP: ");
  }
  if (_ORP_RESPONSE == Ezo_board::SUCCESS) {
    TX_Data_WaterQualityBuoy.ORP_Value = ORP->get_last_received_reading();
    if (_SerialDebug) {
      Serial.println(String(TX_Data_WaterQualityBuoy.ORP_Value));
    }
    _ORP_AVG += TX_Data_WaterQualityBuoy.ORP_Value;
    _index_ORP++;
  } else {
    if (_SerialDebug) {
      Serial.println(WaterQualityBuoyFunctions::CheckError(_ORP_RESPONSE, TX_Data_WaterQualityBuoy.ORP_Value));
    }
  }
  ORP->send_cmd("Sleep");

  RTD->receive_read_cmd();
  _RTD_RESPONSE = RTD->get_error();
  if (_SerialDebug) {
    Serial.print("RTD: ");
  }
  if (( _RTD_RESPONSE == Ezo_board::SUCCESS) && (RTD->get_last_received_reading() > -1000.0)) {  // If the temperature reading has been received and it is valid
    pH->send_read_with_temp_comp(RTD->get_last_received_reading());
    EC->send_read_with_temp_comp(RTD->get_last_received_reading());
    DO->send_read_with_temp_comp(RTD->get_last_received_reading());

    TX_Data_WaterQualityBuoy.RTD_Value = RTD->get_last_received_reading();
    if (_SerialDebug) {
      Serial.println(TX_Data_WaterQualityBuoy.RTD_Value);
    }
    _RTD_AVG += TX_Data_WaterQualityBuoy.RTD_Value;
    _index_RTD++;

    RTD->send_cmd("Sleep");                                                                     // If the temperature reading is invalid, send default temp = 25 deg C to EC sensor
  } else {
    pH->send_read_with_temp_comp(25.0);
    EC->send_read_with_temp_comp(25.0);
    DO->send_read_with_temp_comp(25.0);
    if (_SerialDebug) {
      Serial.println(WaterQualityBuoyFunctions::CheckError(_RTD_RESPONSE, TX_Data_WaterQualityBuoy.RTD_Value));
    }
  }

  vTaskDelay( 815 / portTICK_PERIOD_MS );

  /////////////////////////////3rd Step////////////////////////////
  pH->receive_read_cmd();
  _pH_RESPONSE = pH->get_error();
  if (_SerialDebug) {
    Serial.print("pH: ");
  }
  if (_pH_RESPONSE == Ezo_board::SUCCESS) {
    TX_Data_WaterQualityBuoy.pH_Value = pH->get_last_received_reading();
    if (_SerialDebug) {
      Serial.println(TX_Data_WaterQualityBuoy.pH_Value);
    }
    _pH_AVG += TX_Data_WaterQualityBuoy.pH_Value;
    _index_pH++;
  } else {
    if (_SerialDebug) {
      Serial.println(WaterQualityBuoyFunctions::CheckError(_pH_RESPONSE, TX_Data_WaterQualityBuoy.pH_Value));
    }
  }
  pH->send_cmd("Sleep");

  EC->receive_read_cmd();
  _EC_RESPONSE = EC->get_error();
  if (_SerialDebug) {
    Serial.print("EC: ");
  }
  if (_EC_RESPONSE == Ezo_board::SUCCESS) {
    TX_Data_WaterQualityBuoy.EC_Value = EC->get_last_received_reading();
    if (_SerialDebug) {
      Serial.println(TX_Data_WaterQualityBuoy.EC_Value);
    }
    _EC_AVG += TX_Data_WaterQualityBuoy.EC_Value;
    _index_EC++;

    DO->send_read_with_salinity_comp(TX_Data_WaterQualityBuoy.EC_Value);
    vTaskDelay( 300 / portTICK_PERIOD_MS );
  } else {
    if (_SerialDebug) {
      Serial.println(WaterQualityBuoyFunctions::CheckError(_EC_RESPONSE, TX_Data_WaterQualityBuoy.EC_Value));
    }
  }
  EC->send_cmd("Sleep");

  DO->send_read_cmd();
  vTaskDelay( 815 / portTICK_PERIOD_MS );

  DO->receive_read_cmd();
  _DO_RESPONSE = DO->get_error();
  if (_SerialDebug) {
    Serial.print("DO: ");
  }
  if (_DO_RESPONSE == Ezo_board::SUCCESS) {
    TX_Data_WaterQualityBuoy.DO_Value = DO->get_last_received_reading();
    if (_SerialDebug) {
      Serial.println(TX_Data_WaterQualityBuoy.DO_Value);
    }
    _DO_AVG += TX_Data_WaterQualityBuoy.DO_Value;
    _index_DO++;
  } else {
    if (_SerialDebug) {
      Serial.println(WaterQualityBuoyFunctions::CheckError(_DO_RESPONSE, TX_Data_WaterQualityBuoy.DO_Value));
    }
  }
  DO->send_cmd("Sleep");

  if (_SerialDebug) {
    Serial.println("###################################");
  }

  if ((_AverageTimeArray[0] == 0) && (_AverageTimeArray[1] == 0) && (_AverageTimeArray[2] == 0)) {
    _AverageTimeArray[1] = 1;
  }

  _NumberOfCycles++;

  float SecondsAverage = (_AverageTimeArray[0] * 3600) + (_AverageTimeArray[1] * 60) + _AverageTimeArray[2];

  if ((SecondsAverage / _SampleTime) != int(SecondsAverage / _SampleTime)) {
    Serial.println(F("  \u00BB SecondsAverage has to be a multiple of SampleTime!"));
    sf_WaterQuality.fatalBlink(CRGB::Red);
  }

  if ( _NumberOfCycles == (SecondsAverage / _SampleTime)) {

    sf_WaterQuality.displayLED(CRGB::Green, 15);

    TX_Data_WaterQualityBuoy.BatteryVoltage = lipoWaterQuality.getVoltage();

    // Calculates the average dividing the variables sum by the correct index
    if (_index_ORP != 0) {
      _ORP_AVG = _ORP_AVG / _index_ORP;
      _ORP_NO_DATA = 0;
    } else {
      _ORP_NO_DATA = 1;
    }
    if (_index_RTD != 0) {
      _RTD_AVG = _RTD_AVG / _index_RTD;
      _RTD_NO_DATA = 0;
    } else {
      _RTD_NO_DATA = 1;
    }
    if (_index_pH != 0) {
      _pH_AVG = _pH_AVG / _index_pH;
      _pH_NO_DATA = 0;
    } else {
      _pH_NO_DATA = 1;
    }
    if (_index_EC != 0) {
      _EC_AVG = _EC_AVG / _index_EC;
      _EC_NO_DATA = 0;
    } else {
      _EC_NO_DATA = 1;
    }
    if (_index_DO != 0) {
      _DO_AVG = _DO_AVG / _index_DO;
      _DO_NO_DATA = 0;
    } else {
      _DO_NO_DATA = 1;
    }

    if (_SerialDebug) {
      Serial.println();
      Serial.println("###################################");
      Serial.println("Battery_Voltage : " + String(TX_Data_WaterQualityBuoy.BatteryVoltage));
      Serial.println("_ORP_AVG: " + String(_ORP_AVG));
      Serial.println("_RTD_AVG: " + String(_RTD_AVG));
      Serial.println("_pH_AVG: " + String(_pH_AVG));
      Serial.println("_EC_AVG: " + String(_EC_AVG));
      Serial.println("_DO_AVG: " + String(_DO_AVG));
    }

    if (_GPS_NO_DATA == 0) {
      sf_WaterQuality.displayLED(CRGB::Blue, 15);
      vTaskDelay( 250 / portTICK_PERIOD_MS);
      sf_WaterQuality.displayLED(CRGB::Black, 0);

      if (_SerialDebug) {
        Serial.print("Latitude: ");
        Serial.println(TX_Data_WaterQualityBuoy.Latitude, 5);
        Serial.print("Longitude: ");
        Serial.println(TX_Data_WaterQualityBuoy.Longitude, 5);
      }
    }

    Serial.println("###################################");
    Serial.println();

    sf_WaterQuality.displayLED(CRGB::Green, 15);
    // Creates the Cayenne LPP package (https://www.thethingsnetwork.org/docs/devices/arduino/api/cayennelpp/)
    lppWaterQuality.reset();
    lppWaterQuality.addUnixTime(Epoch_Time_Temp);
    lppWaterQuality.addBatteryVoltage(TX_Data_WaterQualityBuoy.BatteryVoltage);     // in volts, positive values
    lppWaterQuality.addORP(_ORP_AVG, _ORP_NO_DATA);
    lppWaterQuality.addRTD(_RTD_AVG, _RTD_NO_DATA);
    lppWaterQuality.addPH(_pH_AVG, _pH_NO_DATA);
    lppWaterQuality.addEC(_EC_AVG, _EC_NO_DATA);
    lppWaterQuality.addDO(_DO_AVG, _DO_NO_DATA);
    lppWaterQuality.addGPS(TX_Data_WaterQualityBuoy.Latitude, TX_Data_WaterQualityBuoy.Longitude, _GPS_NO_DATA);

    uint8_t* payload = lppWaterQuality.getBuffer();       // gets LPP package size
    String payloadString;

    // Converts into a string the LPP package before sending
    for (int i = 0; i < lppWaterQuality.getSize(); i++) {
      payloadString += (char)payload[i];
    }

    WaterQualityBuoyFunctions::SendLoRaPacket(payloadString);

    vTaskDelay( 250 / portTICK_PERIOD_MS);
    sf_WaterQuality.displayLED(CRGB::Black, 0);

    // Clears the variables before the new loop
    _ORP_AVG = 0;
    _RTD_AVG = 0;
    _pH_AVG = 0;
    _EC_AVG = 0;
    _DO_AVG = 0;
    _index_ORP = 0;
    _index_RTD = 0;
    _index_pH = 0;
    _index_EC = 0;
    _index_DO = 0;
    _NumberOfCycles = 0;
  }
  if (_SerialDebug) {
    Serial.println();
  }

  if (!_FirstSecondDetected) {
    xQueueReset( DataQueueWaterQualityBuoyWriteSD );
  } else {
    if (xQueueSend( DataQueueWaterQualityBuoyWriteSD, ( void * ) &TX_Data_WaterQualityBuoy, portMAX_DELAY ) != pdPASS ) { //portMAX_DELAY
      Serial.println(F("xQueueSend is not working"));
    }
  }
}

//Function used to wake up the ESP32 using the ULP
void WaterQualityBuoyFunctions::init_ulp_program(void) {
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);

  assert(rtc_gpio_desc[gpio_WakeUpWaterQuality].reg && "GPIO used for wake up must be an RTC IO");
  ulp_next_edge = 0;
  ulp_io_number = rtc_gpio_desc[gpio_WakeUpWaterQuality].rtc_num; // Map from GPIO# to RTC_IO

  // Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
  rtc_gpio_init(gpio_WakeUpWaterQuality);
  rtc_gpio_set_direction(gpio_WakeUpWaterQuality, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(gpio_WakeUpWaterQuality);
  rtc_gpio_pullup_en(gpio_WakeUpWaterQuality);
  rtc_gpio_hold_en(gpio_WakeUpWaterQuality);

  err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
  ESP_ERROR_CHECK(err);
}

// Function used to send the LoRa packet
void WaterQualityBuoyFunctions::SendLoRaPacket (String payload) {

  // send in async / non-blocking mode
  LoRa.beginPacket();
  LoRa.write(_WaterQualityBuoy_ID);       // WaterQualityBuoy ID
  LoRa.write(payload.length());           // add payload length
  LoRa.print(payload);                    // add payload
  LoRa.endPacket(true);                   // finish packet and send it
}


void WaterQualityBuoyFunctions::decode41(byte payload[], boolean SleepModeGPS) {
  gpsLat.b[3] = payload[23];
  gpsLat.b[2] = payload[24];
  gpsLat.b[1] = payload[25];
  gpsLat.b[0] = payload[26];

  gpsLon.b[3] = payload[27];
  gpsLon.b[2] = payload[28];
  gpsLon.b[1] = payload[29];
  gpsLon.b[0] = payload[30];

  if ( (payload[11] << 8) + payload[12] > 2020) {
    canSleepWaterQuality = true;
  }
  else {
    canSleepWaterQuality = false;
  }

  if ((gpsLat.i == 0) && (gpsLon.i == 0)) {
    _GPS_NO_DATA = 1;
  } else {
    TX_Data_WaterQualityBuoy.Latitude = gpsLat.i / pow(10, 7);
    TX_Data_WaterQualityBuoy.Longitude = gpsLon.i / pow(10, 7);
    _GPS_NO_DATA = 0;
  }

  if (SleepModeGPS) {
    if (((payload[4] & 7) >= 5) && canSleepWaterQuality) {
      digitalWrite(GPS_ONOFF, HIGH);
      vTaskDelay( 200 / portTICK_PERIOD_MS);
      digitalWrite(GPS_ONOFF, LOW);

      // Waits before waking up again the GPS
      vTaskDelay( 59800 / portTICK_PERIOD_MS);
    }
  }
}

// Function used to get the GPS Location
void WaterQualityBuoyFunctions::GetGPSData(boolean SleepModeGPS) {

  while (digitalRead(GPS_WakeUP) == LOW) { // Waits until the GPS is wake
    digitalWrite(GPS_ONOFF, LOW);
    digitalWrite(GPS_ONOFF, HIGH);
    vTaskDelay( 200 / portTICK_PERIOD_MS);;
    digitalWrite(GPS_ONOFF, LOW);
    vTaskDelay( 1000 / portTICK_PERIOD_MS);
  }

  if (gpsSerialWaterQuality.available()) {
    bool validPackidge = false;
    byte head;
    byte payloadSize[2];

    byte checkSum[2];
    byte tail[2];
    //int sizeOfPayload = 0;
    gpsSerialWaterQuality.readBytes(&head, 1);
    if (head == 160) {
      gpsSerialWaterQuality.readBytes(&head, 1);
      if (head == 162) {
        gpsSerialWaterQuality.readBytes( payloadSize, 2);
        SizeWaterQuality = (payloadSize[0] << 8) +  payloadSize[1];
        gpsSerialWaterQuality.readBytes(DataWaterQuality, SizeWaterQuality);
        gpsSerialWaterQuality.readBytes(checkSum, 2);
        gpsSerialWaterQuality.readBytes(tail, 2);
        int valCheckSum = (checkSum[0] << 8) + checkSum[1];
        int payloadSum = 0;
        for (int i = 0; i < SizeWaterQuality; i++) {
          payloadSum += DataWaterQuality[i];
          payloadSum = payloadSum & 0x7FFF;
        }
        if (tail[0] == 176 && tail[1] == 179 && payloadSum == valCheckSum) {
          validPackidge = true;
        }
      }
    }

    if (validPackidge) {
      if (DataWaterQuality[0] == 41) {
        WaterQualityBuoyFunctions::decode41(DataWaterQuality, SleepModeGPS);
      }
    }
  }
}

// Function used to create the CSV file
void WaterQualityBuoyFunctions::CSV_File(boolean GetNewValue) {

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
  TX_Data_WaterQualityBuoy.RTC_Date = DateBuffer;
  String DayFolder = "/" + String(DateBuffer);

  char TimeBuffer[20];
  sprintf(TimeBuffer, "%02d:%02d:%02d", _HourNow, _MinuteNow, _SecondNow);
  TX_Data_WaterQualityBuoy.RTC_Hour = TimeBuffer;
  sprintf(TimeBuffer, "%02dH-%02dM-%02dS", _HourNow, _MinuteNow, _SecondNow);

  if (!SD_MMC.exists(DayFolder)) {
    if (!SD_MMC.mkdir(DayFolder)) {
      Serial.println(F("  \u00BB Create DayFolder failed!"));
      sf_WaterQuality.fatalBlink(CRGB::Red);
    }
  }

  String FileDir = DayFolder + "/" + String(TimeBuffer) + ".csv";
  FileDir.toCharArray(filenameWaterQuality, 30);

  if (SD_MMC.exists(filenameWaterQuality)) {
    Serial.println(F("  \u00BB A file with that name already exists."));
    sf_WaterQuality.fatalBlink(CRGB::Red);
  }

  // Create file and prepare it ============================================================
  logFileWriteWaterQuality = SD_MMC.open(filenameWaterQuality, FILE_WRITE);
  if ( ! logFileWriteWaterQuality ) {
    Serial.print(F("  \u00BB Couldnt create "));
    Serial.println(filenameWaterQuality);
    sf_WaterQuality.fatalBlink(CRGB::Red);
  }

  Serial.print(F("\n\u21D2 Ready to write to: "));
  Serial.println(filenameWaterQuality);
  Serial.println();

  String CSV_Header[ARRAY_SIZE(CSV_Header_WaterQualityBuoy)];

  for (int i = 0; i < ARRAY_SIZE(CSV_Header_WaterQualityBuoy); i++) {
    CSV_Header[i] = CSV_Header_WaterQualityBuoy[i];
  }

  //Column labels
  for (size_t i = 0; i < ARRAY_SIZE(CSV_Header); i++) {   // Function used to write the csv file headers
    logFileWriteWaterQuality.print(CSV_Header[i]);
    if (i != ARRAY_SIZE(CSV_Header) - 1) {
      logFileWriteWaterQuality.print(",");
    }
  }
  logFileWriteWaterQuality.print("\n");
  logFileWriteWaterQuality.close();

  _justOnce = true;

  xSemaphoreGive(SDWriteSemaphoreWaterQuality);                                     // Semaphore give up allowing the SD Card and CSV writing
}


// Checks if the value read by the Sensor if valid
String WaterQualityBuoyFunctions::CheckError(uint8_t SensorResponse, float value) {

  String Temporary_Value;

  switch (SensorResponse) {             //switch case based on what the response code is.
    case 0:
      Temporary_Value = String(value);
      break;

    case 1:
      Temporary_Value = "FAILED";        //means the command has failed.
      break;

    case 2:
      Temporary_Value = "PENDING";       //the command has not yet been finished calculating.
      break;

    case 3:
      Temporary_Value = "NO DATA";       //the sensor has no data to send.
      break;
  }

  return Temporary_Value;

}

// Function used to write the WaterQuality data on a CSV File
void WaterQualityBuoyFunctions::WriteSD() {

  if ( xQueueReceive( DataQueueWaterQualityBuoyWriteSD, &( RX_Data_WaterQualityBuoy ), portMAX_DELAY ) != pdPASS ) { //portMAX_DELAY
    Serial.println(F("xQueueRecieve is not working"));
  }

  if (xSemaphoreTake(SDWriteSemaphoreWaterQuality, portMAX_DELAY ) == pdTRUE ) {

    if (_SamplesCount != _TotalCount) {       // This task will contine until the total number of samples is not reached (in sleep mode)

      // Variables to write in the CSV file
      String CSV_Values_First[] = {String(_WaterQualityBuoy_ID), String(_SampleTime), RX_Data_WaterQualityBuoy.RTC_Date};

      // Variables to write in the CSV file
      String CSV_Values_Second[] = {RX_Data_WaterQualityBuoy.RTC_Hour_Temp};

      float GPSData [] = {RX_Data_WaterQualityBuoy.Latitude, RX_Data_WaterQualityBuoy.Longitude};

      float CSV_Values_Minute_WaterQualityBuoy[] = {RX_Data_WaterQualityBuoy.BatteryVoltage};

      String ORP_Temp = WaterQualityBuoyFunctions::CheckError(_ORP_RESPONSE, RX_Data_WaterQualityBuoy.ORP_Value);
      String RTD_Temp = WaterQualityBuoyFunctions::CheckError(_RTD_RESPONSE, RX_Data_WaterQualityBuoy.RTD_Value);
      String pH_Temp = WaterQualityBuoyFunctions::CheckError(_pH_RESPONSE, RX_Data_WaterQualityBuoy.pH_Value);
      String EC_Temp = WaterQualityBuoyFunctions::CheckError(_EC_RESPONSE, RX_Data_WaterQualityBuoy.EC_Value);
      String DO_Temp = WaterQualityBuoyFunctions::CheckError(_DO_RESPONSE, RX_Data_WaterQualityBuoy.DO_Value);

      String CSV_Values_Always_WaterQualityBuoy[] = {String(ORP_Temp), String(RTD_Temp),
                                                     String(pH_Temp), String(EC_Temp), DO_Temp
                                                    };

      WaterQualityBuoyFunctions::ParseDataCSV (CSV_Values_First,  ARRAY_SIZE(CSV_Values_First), CSV_Values_Second, ARRAY_SIZE(CSV_Values_Second),
          CSV_Values_Minute_WaterQualityBuoy, ARRAY_SIZE(CSV_Values_Minute_WaterQualityBuoy),
          CSV_Values_Always_WaterQualityBuoy, ARRAY_SIZE(CSV_Values_Always_WaterQualityBuoy), GPSData);

      // If Sleep Mode is enabled, counts the number of samples until going to sleep
      if (_SleepMode) {
        _SamplesCount++;
      }

    } else {

      Serial.println(F("\n################################"));
      Serial.println(F("##       Logging finished     ##"));
      Serial.println(F("################################"));

      Serial.println(F("\n\u21D2 Sleep Mode until RTC alarm is fired."));

      digitalWrite(GPS_WakeUP, HIGH);
      delay(200);
      digitalWrite(GPS_WakeUP, LOW);

      sf_WaterQuality.displayLED(CRGB::Black, 0);

      ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
      esp_sleep_enable_ulp_wakeup();
      delay(2000);
      esp_deep_sleep_start();
    }

    xSemaphoreGive(SDWriteSemaphoreWaterQuality);
  }
}

void WaterQualityBuoyFunctions::ParseDataCSV (String* CSV_Values_First,  int CSV_Values_First_Size, String* CSV_Values_Second, int CSV_Values_Second_Size,  float* CSV_Values_Minute, int CSV_Values_Minute_Size, String* CSV_Values_Always, int CSV_Values_Always_Size, float GPSData [2]) {

  logFileWriteWaterQuality = SD_MMC.open(filenameWaterQuality, FILE_APPEND);  // Opens the CSV file in append mode

  // Variables that only is going to be writen once on the CSV file
  if (_justOnce) {
    for (int i = 0; i < CSV_Values_First_Size; i++) {
      logFileWriteWaterQuality.print(CSV_Values_First[i]);
      logFileWriteWaterQuality.print(',');
    }

    for (int i = 0; i < CSV_Values_Second_Size; i++) {
      logFileWriteWaterQuality.print(CSV_Values_Second[i]);
      logFileWriteWaterQuality.print(',');
    }

    for (int i = 0; i < CSV_Values_Minute_Size; i++) {
      logFileWriteWaterQuality.print(CSV_Values_Minute[i]);
      logFileWriteWaterQuality.print(',');
    }

    for (int i = 0; i < 2; i++) {
      if (_GPS_NO_DATA == 0) {
        logFileWriteWaterQuality.print(GPSData[i], 5);
      } else {
        logFileWriteWaterQuality.print("NO DATA");
      }
      logFileWriteWaterQuality.print(',');
    }

    for (int i = 0; i < CSV_Values_Always_Size; i++) {
      logFileWriteWaterQuality.print(CSV_Values_Always[i]);
      logFileWriteWaterQuality.print(',');
    }

    _justOnce = false;

  } else {

    for (int i = 0; i < CSV_Values_First_Size; i++) {
      logFileWriteWaterQuality.print("");
      logFileWriteWaterQuality.print(',');
    }

    for (int i = 0; i < CSV_Values_Second_Size; i++) {
      if ((_SecondNow != _SecondNowPrev) || ((_SecondNow == _SecondNowPrev) && (_MinuteNow != _MinuteNowPrev))) {
        logFileWriteWaterQuality.print(CSV_Values_Second[i]);
      } else {
        logFileWriteWaterQuality.print("");
      }
      logFileWriteWaterQuality.print(',');
    }

    for (int i = 0; i < CSV_Values_Minute_Size; i++) {
      if (_MinuteNow != _MinuteNowPrev) {
        logFileWriteWaterQuality.print(CSV_Values_Minute[i]);
      } else {
        logFileWriteWaterQuality.print("");
      }
      logFileWriteWaterQuality.print(',');
    }

    for (int i = 0; i < 2; i++) {
      if (_MinuteNow != _MinuteNowPrev) {
        if (_GPS_NO_DATA == 0) {
          logFileWriteWaterQuality.print(GPSData[i], 5);
        } else {
          logFileWriteWaterQuality.print("NO DATA");
        }
      } else {
        logFileWriteWaterQuality.print("");
      }
      logFileWriteWaterQuality.print(',');
    }

    for (int i = 0; i < CSV_Values_Always_Size; i++) {
      logFileWriteWaterQuality.print(CSV_Values_Always[i]);
      logFileWriteWaterQuality.print(',');
    }
  }

  _MinuteNowPrev = _MinuteNow;
  _SecondNowPrev = _SecondNow;

  logFileWriteWaterQuality.print('\n');
  logFileWriteWaterQuality.close();
}
