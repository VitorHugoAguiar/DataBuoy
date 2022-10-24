// includes
#include <SharedFunctions.h>
#include <WaveBuoyFunctions.h>

SharedFunctions sf_db;
WaveBuoyFunctions Wave;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// WaveBuoy Settings
boolean SerialDebug = true;                               // Enables Serial Debug messages
boolean SleepMode = false;                                // Enables Sleep Mode
boolean SleepModeGPS = false;                             // Enables GPS Sleep Mode

uint8_t WaveBuoy_ID = 1;                                  // WaveBuoy ID (int until 99)
uint8_t SampleRate = 50 ;                                 // Sample Rate (Hz).

uint8_t StartTimeArray[6] = {9, 9, 21, 14, 55, 0};        // Defines when the WaveBuoy is going to start to record data
uint8_t AverageTimeArray[3] = {0, 0, 0};                  // Defines the average time (Hour, Minutes, Seconds). Default = 1min
uint8_t SampleLengthMinutes = 1;                          // Defines the interval between sent LoRa packets
uint8_t Alarm_WakeUp_Minutes[] = {0};                     // Alarm wake up minutes (RTC will wake up on this minutes every hour), ex: {0, 30} -> The WaveBuoy will wake up at 0min and 30min  

float LPCutOffFreqAccZ = 1;                               // AccZ Low Pass Filter cutoff frequency
float HPCutOffFreqVelocity = 0.1;                         // Velocity High Pass Filter cuttoff frequency  
float HPCutOffFreqDisplacement = 0.1;                     // Displacement High Pass Filter cuttoff frequency

float WaveHeightThreshold = 0.05;                         // Height threshold (m)
float Gravity_Acceleration = 9.79541;                     // Madeira gravity acceleration (m/s^2)
float Magnetic_Declination = -4.45;                       // Magnetic declination for Funchal, Madeira (Portugal) in degress
//Its possible to get the declination for each location from here:
//https://www.magnetic-declination.com/ and then convert to degrees:
//https://www.rapidtables.com/convert/number/degrees-minutes-seconds-to-degrees.html
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

// Tasks
void TaskTimer( void *pvParameters );
void TaskGetData( void *pvParameters );
void TaskWriteSD( void *pvParameters );
void TaskGPS( void *pvParameters );

TaskHandle_t xTimer;
TaskHandle_t xGetData;
TaskHandle_t xSDWrite;
TaskHandle_t xGPS;

// Hardware Timer
hw_timer_t * SamplingTimer = NULL;

// Create Interrupt Semaphores
SemaphoreHandle_t ST_WaveBuoy_Semaphore = NULL;
//##############################################################################

// Function used to give up on the sampling semaphore
void IRAM_ATTR vST_WaveBuoy_ISR() {                           // Timer ISR
  xSemaphoreGiveFromISR(ST_WaveBuoy_Semaphore, NULL);
}

// Function used to update the timestamp
void TaskTimer(void *pvParameters) {
  (void) pvParameters;

  TickType_t xTimerInterval;
  const TickType_t xFrequencyInterval = 1;                    // Time between readings (ms)

  xTimerInterval = xTaskGetTickCount ();

  for (;;) // A Task shall never return or exit.
  {

    Wave.updateTimeStamp();                                   // Updates the timestamp using the TimeLib

    vTaskDelayUntil( &xTimerInterval, xFrequencyInterval);    // Waits until the xFrequencyInterval has passed, without blocking
  }
  vTaskDelete( NULL );
}

// Function used to get IMU data
void TaskGetData(void *pvParameters) {
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    if (xSemaphoreTake(ST_WaveBuoy_Semaphore, portMAX_DELAY ) == pdTRUE ) {             // Semaphore responsible for keeping the sample rate

      Wave.GetData(WaveHeightThreshold, Gravity_Acceleration, Magnetic_Declination);    // Gets IMU data

    }
  }
  vTaskDelete( NULL );
}

//Function used to write data into SD Card
void TaskWriteSD(void *pvParameters) {
  (void) pvParameters;

  for (;;)                                  // A Task shall never return or exit.
  {

    Wave.WriteSD();                         // Writes the data on the CSV file

  }
  vTaskDelete( NULL );
}

//Function used to get GPS data
void TaskGPS(void *pvParameters) {
  (void) pvParameters;

  for (;;)                                  // A Task shall never return or exit.
  {

    Wave.GetGPSData(SleepModeGPS);          // Tries to get the GPS Location
    vTaskDelay( 1 / portTICK_PERIOD_MS);

  }
  vTaskDelete( NULL );
}

void setup() {

  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  sf_db.setup();                                                // Initializes the shared setup
  Wave.setup(SerialDebug, SleepMode, WaveBuoy_ID, SampleRate);  // Initializes the WaveBuoy setup

  Serial.println(F("\n################################"));
  Serial.println(F("##                            ##"));
  Serial.println(F("##     WaveBuoy LoPy4 Node    ##"));
  Serial.print(F("##             "));
  Serial.print(String(WaveBuoy_ID));
  Serial.println("              ##");
  Serial.println(F("##                            ##"));
  Serial.println(F("################################"));

  Serial.println(F("\n################################"));
  Serial.println(F("##        Main Settings       ##"));
  Serial.println(F("################################\n"));

  int cpuSpeed = getCpuFrequencyMhz();
  Serial.println("\u21D2 CPU Frequency: " + String(cpuSpeed) + "MHz");
  if (SerialDebug) {
    Serial.println(F("\u21D2 SerialDebug: TRUE"));
  }
  if (SleepMode) {
    Serial.println(F("\u21D2 SleepMode: TRUE"));
  }

  Serial.println(F("\n################################"));
  Serial.println(F("##  Components initialization ##"));
  Serial.println(F("################################"));

  Wave.start_MPU9250();                                                        // Starts MPU9250
  Wave.start_BMP280();                                                         // Starts BMP280
  Wave.start_Madgwick_Filter(SampleRate);                                      // Starts Madgwick filter
  Wave.initLowPassFilterAccZ(LPCutOffFreqAccZ, SampleRate);                    // Starts AccZ Low Pass Filter
  Wave.initHighPassFilterVelocity(HPCutOffFreqVelocity, SampleRate);           // Starts Velocity High Pass Filter
  Wave.initHighPassFilterDisplacement(HPCutOffFreqDisplacement, SampleRate);   // Starts Displacement High Pass Filter
  Wave.start_MAX17048();                                                       // Starts FuelGauge
  Wave.start_GPS();                                                            // Starts GPS module
  sf_db.start_LoRa();                                                          // Starts LoRa module with the choosen settings

  sf_db.displayLED(CRGB::Green, 15);
  delay(1000);

  Wave.start_RTC(StartTimeArray, AverageTimeArray, SampleLengthMinutes, 
                 Alarm_WakeUp_Minutes, ARRAY_SIZE(Alarm_WakeUp_Minutes));       // Starts RTC
  sf_db.start_SDCard();                                                         // Starts SD Card Reader

  Wave.CSV_File(false);                                                         // Creates a CSV file 

  ST_WaveBuoy_Semaphore = xSemaphoreCreateBinary();                             // Create Sampling Semaphore

  Serial.println(F("################################"));
  Serial.println(F("##         Logging...         ##"));
  Serial.println(F("################################"));
  Serial.println();

  sf_db.displayLED(CRGB::Black, 0);

  // Setup up Tasks and where to run ============================================================
  // Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskTimer
    ,  "TaskTimer"
    ,  4000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  &xTimer
    ,  TaskCore0);

  xTaskCreatePinnedToCore(
    TaskGPS
    ,  "TaskGPS"
    ,  3800 // Stack size
    ,  NULL
    ,  0 // Priority
    ,  &xGPS
    ,  TaskCore0);

  xTaskCreatePinnedToCore(
    TaskGetData
    ,  "TaskGetData"
    ,  5000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  &xGetData
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskWriteSD
    ,  "TaskWriteSD"
    ,  6000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  &xSDWrite
    ,  TaskCore1);


  // Create Timer ===============================================================================
  SamplingTimer = timerBegin(0, 80, true);                              // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
  timerAttachInterrupt(SamplingTimer, &vST_WaveBuoy_ISR, true);         // Attach &vTimerISR to our timer.
  timerAlarmWrite(SamplingTimer, (1000000 / SampleRate), true);         // Repeat the alarm (third parameter)
  timerAlarmEnable(SamplingTimer);                                      // Start an alarm
}

void loop() {
  // Empty. Things are done in Tasks.
  /*--------------------------------------------------*/
  /*---------------------- Tasks ---------------------*/
  /*--------------------------------------------------*/
}
