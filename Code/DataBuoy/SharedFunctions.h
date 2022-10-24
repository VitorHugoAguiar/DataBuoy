#ifndef sf_h
#define sf_h

//Libraries
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <TimeLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <WiFiUdp.h>
#include "esp_system.h"
#include "FastLED.h"
#include "SD_MMC.h"

// LoRa module pins
const int LORA_MOSI_PIN = 27;
const int LORA_MISO_PIN = 19;
const int LORA_SCK_PIN  = 5;
const int LORA_SS_PIN   = 18;
const int LORA_RST_PIN  = 255;
const int LORA_DI00_PIN = 23;

// LoRa settings
#define LoRaFrequency       868E6  // Asia - 433E6, EU - 868E6, USA - 915E6
#define LoRaTxPower         14     // default 17db  (Europe Max - 14db)
#define LoRaSignalBandwidth 500E3  // default 125E3 (7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3)
#define LoRaCodingRate4     4      // default 5 (5 to 8)
#define LoRaPreambleLength  6      // default 8 (6 - 65535)
#define LoRaSpreadingFactor 12      // default 7 (6 - 12). 

// Use ESP32 duo core
const int TaskCore0  = 0;
const int TaskCore1  = 1;

class SharedFunctions {
  public:
    // Methods
    void setup();
    void start_LoRa();
    void start_SDCard();

    void displayLED(CRGB color, unsigned char brightness);
    void fatalBlink(CRGB color);

  private:

};

#endif
