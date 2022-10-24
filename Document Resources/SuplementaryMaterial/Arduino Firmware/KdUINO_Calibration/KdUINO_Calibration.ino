 /*
  KdUINO calibration firmware code 
  by: Raul Bardají Benach.
  Date: 02/11/2015
  Number of sensors: 4
  Arduino MEGA with DATALOGGER SHIELD
  
  IMPORTANT NOTE: Please, connect with a wire, SLC and SDA pins 
  next to REF pin of the DATA LOGGER SHIELD with the A5 and A4 
  pins  respectively. They are not linked.
    
  This work is licensed under the Creative Commons Attribution 
  4.0 International License. To view a copy of this license, 
  visit http://creativecommons.org/licenses/by/4.0/.
  */


#include <SD.h>

#define SENSOR1_PIN 18 // Sensor 1 is connected to pin 18
#define SENSOR2_PIN 19 // Sensor 2 is connected to pin 19
#define SENSOR3_PIN 21 // Sensor 3 is connected to pin 21
#define SENSOR4_PIN 20 // Sensor 4 is connected to pin 20

const int chipSelect = 4;

//Creation datalogger file
File dataFile;

// Pulse counters of each sensor
unsigned long pulseCnt1 = 0; 
unsigned long pulseCnt2 = 0;      
unsigned long pulseCnt3 = 0;      
unsigned long pulseCnt4 = 0;

// Integration time of the measure in miliseconds 
int MeaurementTime = 1000; 

void setup() {
  // Pin configuration
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(SENSOR3_PIN, INPUT);
  pinMode(SENSOR4_PIN, INPUT);
  
  // Interrupt configuration
  attachInterrupt(5, addPulse1, RISING);  //Interrupt to pin 18
  attachInterrupt(4, addPulse2, RISING);  //Interrupt to pin 19
  attachInterrupt(2, addPulse3, RISING);  //Interrupt to pin 21
  attachInterrupt(3, addPulse4, RISING);  //Interrupt to pin 20

  // Open Serial communications and wait for port to open:
  Serial.begin(9600);

  // SD configuration
  pinMode(SS, OUTPUT);
  if (!SD.begin(10,11,12,13)) {
    // If you enter here, you are not going to 
    // save anything into the SD
    Serial.print("Error: SD");
  }
  else
  {
    // Open up the file we're going to log to
    dataFile = SD.open("calibration.txt", FILE_WRITE);
    if (! dataFile) {
      // If you enter here, you are not going to 
      // save anything into the datalog file
      Serial.print("Error: Cannot creat the datalog file");
    }
  }
  calibrationSensors();
}

void loop() 
{
}

void calibrationSensors()
{
  // Initialize pulse counters 
  pulseCount_init();
  // Whait the time of the meausre
  waitSeconds();

  //Get the pulse counters
  unsigned long pulses1 = getSensor1Pulse(); 
  unsigned long pulses2 = getSensor2Pulse();
  unsigned long pulses3 = getSensor3Pulse();
  unsigned long pulses4 = getSensor4Pulse();

  // Save data into the datalog file
  dataFile.print(" 1 "); dataFile.print(pulses1);
  dataFile.print(" 2 "); dataFile.print(pulses2);
  dataFile.print(" 3 "); dataFile.print(pulses3);
  dataFile.print(" 4 "); dataFile.println(pulses4);
  dataFile.flush();

  //Send data to Serial port
  Serial.print(" 1 "); Serial.print(pulses1);
  Serial.print(" 2 "); Serial.print(pulses2);
  Serial.print(" 3 "); Serial.print(pulses3);
  Serial.print(" 4 "); Serial.println(pulses4);
}

void pulseCount_init()
{
  // Inizialization of pulse counters
  pulseCnt1=0;
  pulseCnt2=0;
  pulseCnt3=0;
  pulseCnt4=0;
}

void waitSeconds()
{
  unsigned long curTm; // Time passed
  unsigned long preTm; // Time saved, time actual
  unsigned int diffTm; // Diferential time between actual and passed

  curTm   = millis();
  diffTm = 0;
  while ( diffTm <= MeaurementTime ) // Wait MeaurementTime seconds
  {
    // Calculation of passed time
    preTm   = curTm;
    curTm   = millis();
    if( curTm > preTm ) diffTm += curTm - preTm;
    else if( curTm < preTm ) diffTm += ( curTm + ( 34359737 - preTm ));  //Overflow   
  }
}

// Read the pulse counter of sensors and reset the pulse counter
unsigned long getSensor1Pulse() 
{   
  unsigned long freq = pulseCnt1;
  pulseCnt1 = 0; 
  return(freq);
}
unsigned long getSensor2Pulse() 
{  
  unsigned long freq = pulseCnt2;
  pulseCnt2 = 0;
  return(freq);
}
unsigned long getSensor3Pulse() 
{  
  unsigned long freq = pulseCnt3;
  pulseCnt3 = 0;
  return(freq);
}
unsigned long getSensor4Pulse() 
{  
  unsigned long freq = pulseCnt4;
  pulseCnt4 = 0;
  return(freq);
}

// Add 1 to the pulse counters
void addPulse1()
{
  pulseCnt1++;
  return;
}
void addPulse2()
{
  pulseCnt2++;
  return;
}
void addPulse3()
{
  pulseCnt3++;
  return;
}
void addPulse4()
{
  pulseCnt4++;
  return;
}
