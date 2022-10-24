// includes
#include <SharedFunctions.h>
#include <WaterQualityBuoyFunctions.h>

SharedFunctions sf_db;
WaterQualityBuoyFunctions WaterQuality;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// WaterQualityBuoy Settings
boolean SerialDebug = true;                                  // Enables Serial Debug messages
boolean SleepMode = false;                                   // Enables Sleep Mode
boolean SleepModeGPS = false;                                // Enables GPS Sleep Mode

uint8_t WaterQualityBuoy_ID = 3;                             // WaterQualityBuoy ID (int until 99)
uint8_t SampleTime = 60;                                     // SampleTime (Hz).

uint8_t StartTimeArray[6] = {9, 9, 21, 14, 55, 0};           // Defines when the WaveBuoy is going to start to record data
uint8_t AverageTimeArray[3] = {0, 0, 0};                     // Defines the average time (Hour, Minutes, Seconds). Default = 1min
uint8_t SampleLengthMinutes = 1;                             // Defines the interval between sent LoRa packets
uint8_t Alarm_WakeUp_Minutes[] = {0};                        // Alarm wake up minutes (RTC will wake up on this minutes every hour)  
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

// Tasks
void TaskTimer( void *pvParameters );
void TaskGetDataWhiteBoxT1( void *pvParameters );
void TaskWriteSD( void *pvParameters );
void TaskGPS( void *pvParameters );

TaskHandle_t xTimer;
TaskHandle_t xGetDataWhiteBoxT1;
TaskHandle_t xSDWrite;
TaskHandle_t xGPS;

// Hardware Timer
hw_timer_t * SamplingTimer = NULL;

// Create Interrupt Semaphores
SemaphoreHandle_t ST_WaterQualityBuoy_Semaphore = NULL;

//##############################################################################

// Function used to give up on the sampling semaphore
void IRAM_ATTR vST_WaterQualityBuoy_ISR() {                       // Timer ISR
  xSemaphoreGiveFromISR(ST_WaterQualityBuoy_Semaphore, NULL);
}

// Function used to update the timestamp
void TaskTimer(void *pvParameters) {
  (void) pvParameters;

  TickType_t xTimerInterval;
  const TickType_t xFrequencyInterval = 1;                        // Time between readings (ms)

  xTimerInterval = xTaskGetTickCount ();

  for (;;) // A Task shall never return or exit.
  {

    WaterQuality.updateTimeStamp();                               // Updates the timestamp using the TimeLib

    vTaskDelayUntil( &xTimerInterval, xFrequencyInterval);        // Waits until the xFrequencyInterval has passed, without blocking
  }
  vTaskDelete( NULL );
}

// Function used to get all Sensors data
void TaskGetDataWhiteBoxT1(void *pvParameters) {
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {

    if (xSemaphoreTake(ST_WaterQualityBuoy_Semaphore, portMAX_DELAY ) == pdTRUE ) {      // Semaphore responsible for keeping the sample rate

      WaterQuality.GetDataWhiteBoxT1();                                                  // Gets Sensors data 

    }
  }
  vTaskDelete( NULL );
}


//Function used to write data into SD Card
void TaskWriteSD(void *pvParameters) {
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {

    WaterQuality.WriteSD();                       // Writes the data on the CSV file

  }
  vTaskDelete( NULL );
}

//Function used to get GPS data
void TaskGPS(void *pvParameters) {
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {

    WaterQuality.GetGPSData(SleepModeGPS);       // Tries to get the GPS Location
    vTaskDelay( 1 / portTICK_PERIOD_MS);

  }
  vTaskDelete( NULL );
}

void setup() {

  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  sf_db.setup();                                                                // Initializes the shared setup
  WaterQuality.setup(SerialDebug, SleepMode, WaterQualityBuoy_ID, SampleTime);  // Initializes the WaterQualityBuoy setup

  Serial.println(F("\n################################"));
  Serial.println(F("##                            ##"));
  Serial.println(F("##     DataBuoy LoPy4 Node    ##"));
  Serial.println(F("##        WaterQualityBuoy         ##"));
  Serial.print("##             ");
  Serial.print(String(WaterQualityBuoy_ID));
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
  

  Serial.println("\u21D2 SampleTime: "  + String(SampleTime) + "sec");


  Serial.println(F("\n################################"));
  Serial.println(F("##  Components initialization ##"));
  Serial.println(F("################################"));

  WaterQuality.start_MAX17048();                                                     // Starts FuelGauge
  WaterQuality.start_GPS();                                                          // Starts GPS module
  WaterQuality.start_WhiteBoxT1();                                                   // Starts Sensors readings
  sf_db.start_LoRa();                                                                // Starts LoRa module with the choosen settings

  sf_db.displayLED(CRGB::Green, 15);
  delay(1000);

  WaterQuality.start_RTC(StartTimeArray, AverageTimeArray, SampleLengthMinutes, 
                         Alarm_WakeUp_Minutes, ARRAY_SIZE(Alarm_WakeUp_Minutes));    // Starts RTC
  sf_db.start_SDCard();                                                              // Starts SD Card

  WaterQuality.CSV_File(false);                                                      // Creates a CSV file 
  ST_WaterQualityBuoy_Semaphore = xSemaphoreCreateBinary();                          // Create semaphore

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
    TaskWriteSD
    ,  "TaskWriteSD"
    ,  6000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  &xSDWrite
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskGPS
    ,  "TaskGPS"
    ,  3800 // Stack size
    ,  NULL
    ,  1 // Priority
    ,  &xGPS
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskGetDataWhiteBoxT1
    ,  "TaskGetDataWhiteBoxT1"
    ,  5000 // Stack size
    ,  NULL
    ,  2 // Priority
    ,  &xGetDataWhiteBoxT1
    ,  TaskCore0);


  // Create Timer ===============================================================================
  SamplingTimer = timerBegin(0, 80, true);                                 // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
  timerAttachInterrupt(SamplingTimer, &vST_WaterQualityBuoy_ISR, true);    // Attach &vTimerISR to our timer.
  timerAlarmWrite(SamplingTimer, (1000000 * SampleTime), true);            // Repeat the alarm (third parameter)
  timerAlarmEnable(SamplingTimer);                                         // Start an alarm
}

void loop() {
  // Empty. Things are done in Tasks.
  /*--------------------------------------------------*/
  /*---------------------- Tasks ---------------------*/
  /*--------------------------------------------------*/
}
