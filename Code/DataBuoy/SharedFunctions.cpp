#include "SharedFunctions.h"

CRGB leds[1];
int fatalBlinkCounter;

void SharedFunctions::setup() {
  FastLED.addLeds<WS2812B, 0, GRB>(leds, 1);
  SharedFunctions::displayLED(CRGB::Black, 0);

  setCpuFrequencyMhz(240); //Set CPU clock to 240MHz
  btStop();
}

void SharedFunctions::start_LoRa() {
  Serial.println(F("\n\u21D2 Initializing LoRa Module..."));
  // Configuring LoRa
  pinMode(LORA_DI00_PIN, INPUT);
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_SS_PIN);
  LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DI00_PIN);

  if (!LoRa.begin(LoRaFrequency)) {
    Serial.println(F("  \u00BB Starting LoRa Module... failed!"));
    SharedFunctions::fatalBlink(CRGB::YellowGreen);
  } else {
    Serial.println(F("  \u00BB LoRa Module initialized with success."));
    LoRa.setTxPower(LoRaTxPower);
    LoRa.setSpreadingFactor(LoRaSpreadingFactor);
    LoRa.setSignalBandwidth(LoRaSignalBandwidth);
    LoRa.setCodingRate4(LoRaCodingRate4);
    LoRa.setPreambleLength(LoRaPreambleLength);
    LoRa.enableCrc();
    LoRa.idle();
  }
}

void SharedFunctions::start_SDCard() {
  // For SDMMC hookup need to pull certain channels high during startup
  pinMode(2, INPUT_PULLUP);

  // SD CARD SETUP ====================================================================
  // see if the card is present and can be initialized:
  Serial.println(F("\n\u21D2 Initializing SD Card..."));
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println(F("  \u00BB Card failed, or not present..."));
    SharedFunctions::fatalBlink(CRGB::Red);
  } else {
    Serial.println(F("  \u00BB Card initialized with success."));
  }
}

void SharedFunctions::displayLED(CRGB color, unsigned char brightness) {
  FastLED.setBrightness(brightness);
  leds[0] = color;
  FastLED.show();
}

// Function call when a error is detected, to blink the built in led with the right color
void SharedFunctions::fatalBlink(CRGB color) {
  //Red - Everything related with SDCard and folders (DataBuoy and Gateway)
  //YellowGreen - LoRa Module (DataBuoy and Gateway)
  //Yellow - MPU9250 and BMP280 (DataBuoy), Wrong device (Gateway)
  //Purple - RTC (DataBuoy)
  //Blue - GPS (DataBuoy)
  //Orange - StartTime (DataBuoy)
  //Brown  - MAX17048 (DataBuoy), Ping Failed (Gateway)

  while (true) {
    yield();
    SharedFunctions::displayLED(color, 255);
    delay(500);
    SharedFunctions::displayLED(CRGB::Black, 0);
    delay(200);
    fatalBlinkCounter++;
    if (fatalBlinkCounter == 10) {
      ESP.restart();
    }
  }
}
