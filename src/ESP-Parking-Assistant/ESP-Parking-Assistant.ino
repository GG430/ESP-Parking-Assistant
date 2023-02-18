#include <FastLED.h>						// https://github.com/FastLED/FastLED
#include <TFMPlus.h>						// https://github.com/budryerson/TFMini-Plus
#include <ESP8266WiFi.h>

// =========================================================================
//  User Defined values and options
// =========================================================================
//  Change default values here. Changing any of these requires a recompile and upload.

#define VERSION "V1.1.0 (ESP8266)"

#define wakeDistance 3500                   // Wake/sleep distance
#define startDistance 2000                  // Start countdown distance
#define parkDistance 550                    // Final parked distance
#define backupDistance 500                  // Flash backup distance
#define activeBrightness 150                // Active brightness 1 to 255
#define sleepBrightness 5                   // Sleep brightness 0 to 255 - If set to 0 no standby LEDs
#define maxOperationTimePark 60             // Time the LEDs are on after parking finished
#define maxOperationTimeExit 5              // Maximum exit time 
#define ledEffect_m1 "Out-In"               // Chose LED effect from "Out-In" "In-Out" "Full-Strip" "Full-Strip-Inv" or "Solid"

#define LED_DATA_PIN 12                     // Pin connected to LED strip DIN
#define numLEDs 30                          // Number of LEDs used
#define NUM_LEDS_MAX 30                     // For initialization - recommend actual max 50 LEDs
#define SERIAL_DEBUG 0                      // 0 = Disable (must be disabled if using RX/TX pins), 1 = enable

// LED Colors
CRGB ledColorOff = CRGB::Black;				// Off color - Do not change
CRGB ledColorStandby = CRGB::Blue;			// Standby color - Default: Blue
CRGB ledColorWake = CRGB::Green;			// Wake color when car enters wake zone passing wakeDistance - Default: Green
CRGB ledColorActive = CRGB::Yellow;			// Active color when car passes startDistance - Default: Yellow
CRGB ledColorParked = CRGB::Red;			// Color when car reaches parkDistance - Default: Red
CRGB ledColorBackup = CRGB::Red;			// Warning color when car should be backed up, starts crossing backupDistance - Default: Red (Will blink LEDs) 

// =========================================================================
// Global Variables - Do not change
// =========================================================================

bool showStandbyLEDs = true;
bool tfMiniEnabled = false;
bool blinkOn = false;
int intervalDistance = 0;
bool carDetected = false;
bool isAwake = false;
bool coldStart = true;

byte carDetectedCounter = 0;
byte carDetectedCounterMax = 3;
byte nocarDetectedCounter = 0;
byte nocarDetectedCounterMax = 3;
byte outOfRangeCounter = 0;
uint32_t startTime;
bool exitSleepTimerStarted = false;
bool parkSleepTimerStarted = false;

TFMPlus tfmini;
CRGB LEDs[NUM_LEDS_MAX];

// =========================================================================
//  Main Setup
// =========================================================================
void setup() {

  if (sleepBrightness == 0) {
    showStandbyLEDs = false;
  } else {
    showStandbyLEDs = true;
  }

  // Set interval distance based on current Effect
  intervalDistance = calculateInterval();  

  // -------------
  // SETUP FASTLED  
  // -------------
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(LEDs, NUM_LEDS_MAX);
  FastLED.setDither(false);
  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(activeBrightness);
  fill_solid(LEDs, numLEDs, ledColorOff);
  FastLED.show();

  // --------------
  // SETUP TFMINI
  // --------------
  Serial.begin(115200);
  delay(20);
  tfmini.begin(&Serial);
  tfMiniEnabled = true;

//  delay(1000);
  updateBOOT();
  
  WiFi.mode(WIFI_OFF);
  
  updateWIFIOFF();  

}

// =========================================================================
//   MAIN LOOP
// =========================================================================
void loop() {
 
  uint32_t currentMillis = millis();
  int16_t tf_dist = 0;
  int16_t distance = 0;

  //Attempt to get reading from TFMini
  if (tfMiniEnabled) {
    if (tfmini.getData(distance)) {
      tf_dist = (distance * 10) + 20; // Add correction factor
    } else {
      tf_dist = 8888;  //Default value if reading unsuccessful
    }
  } else {
    tf_dist = 9999;  //Default value if TFMini not enabled (serial connection failed)
  }

  //Determine if car (or other object) present in any zones
  if (tf_dist <= wakeDistance) {
    if (!carDetected) {
      carDetectedCounter ++;
    }
    if (carDetectedCounter > carDetectedCounterMax) {  //eliminate trigger on noise
      carDetectedCounter = 0;
      carDetected = true;
      exitSleepTimerStarted = false;
      parkSleepTimerStarted = true;
      startTime = currentMillis;
      FastLED.setBrightness(activeBrightness);
      isAwake = true;
    }
  } else {
    nocarDetectedCounter ++;
    if (nocarDetectedCounter > nocarDetectedCounterMax) {  //eliminate trigger on noise
      if (!exitSleepTimerStarted) {
        if ((carDetected) || (coldStart)) {
          exitSleepTimerStarted = true;
          coldStart = false;
          startTime = currentMillis;
        }
      }
      carDetected = false;
      carDetectedCounter = 0;
      nocarDetectedCounter = 0;
    }
  }

  //Update LEDs
  if ((carDetected) && (isAwake)) {
    if (tf_dist <= backupDistance) {
      //Beyond minimum distance - flash backup!
      blinkLEDs(ledColorBackup);
      
    } else if (tf_dist <= parkDistance) {  
      //In desired parked distance
      fill_solid(LEDs, numLEDs, ledColorParked);
  
    } else if ((tf_dist > startDistance) && (tf_dist <= wakeDistance)) {
      //Beyond start distance but within wake distance
      fill_solid(LEDs, numLEDs, ledColorWake);
      
    } else if ((tf_dist <= startDistance) && (tf_dist > parkDistance)) {
      //Update based on selected effect
      if (ledEffect_m1 == "Out-In") {
        updateOutIn(tf_dist);
      } else if (ledEffect_m1 == "In-Out") {
        updateInOut(tf_dist);
      } else if (ledEffect_m1 == "Full-Strip") {
        updateFullStrip(tf_dist);
      } else if (ledEffect_m1 == "Full-Strip-Inv") {
        updateFullStripInv(tf_dist);
      } else if (ledEffect_m1 == "Solid") {
        updateSolid(tf_dist);
      }
    }
  }
 
  //Put system to sleep if parking or exit time elapsed 
  uint32_t elapsedTime = currentMillis - startTime;
  if (((elapsedTime > (maxOperationTimePark * 1000)) && (parkSleepTimerStarted)) || ((elapsedTime > (maxOperationTimeExit * 1000)) && (exitSleepTimerStarted  ))) {
    updateSleepMode();
    isAwake = false;
    startTime = currentMillis;
    exitSleepTimerStarted = false;
    parkSleepTimerStarted = false;
  }

  //Show/Refresh LED Strip
  FastLED.show();
  delay(200);
}

// =========================================================================
// Calculations and Misc Functions
// =========================================================================
int calculateInterval() {
  int retVal = 0;
  if ((ledEffect_m1 == "Out-In") || (ledEffect_m1 == "In-Out")) {
    retVal = ((startDistance - parkDistance) / (numLEDs / 2));
  } else if ((ledEffect_m1 == "Full-Strip") || (ledEffect_m1 == "Full-Strip-Inv")) {
    retVal = ((startDistance - parkDistance) / (numLEDs));
  } 
  return retVal;
}

// =========================================================================
//  LED and Display Functions
// =========================================================================
void blinkLEDs(CRGB color) {
  if (blinkOn) {
    fill_solid(LEDs, numLEDs, color);
  } else {
    fill_solid(LEDs, numLEDs, CRGB::Black);
  }
  blinkOn = !blinkOn;
}


void updateOutIn(int curDistance) {
   byte numberToLight = 1;
  fill_solid(LEDs, numLEDs, CRGB::Black);

  //Get number of LEDs to light up on each end, based on interval
  numberToLight = (startDistance - curDistance) / intervalDistance;
  if (numberToLight == 0 ) numberToLight = 1;  //Assure at least 1 light if integer truncation results in 0
  for (int i=0; i < numberToLight; i++) {
    LEDs[i] = ledColorActive;
    LEDs[(numLEDs-1) - i] = ledColorActive;
  }
}

void updateInOut(int curDistance) {
  byte numberToLight = 1;
  byte startLEDLeft = 0;
  byte startLEDRight = 0;
  fill_solid(LEDs, numLEDs, CRGB::Black);
  //Get number of LEDs to light up on each end, based on interval
  numberToLight = ((startDistance - curDistance) / intervalDistance);
  if (numberToLight == 0 ) numberToLight = 1;  //Assure at least 1 light if integer truncation results in 0
  //Find center LED(s) - single of odd number, two if even number of LEDS
  startLEDLeft = (numLEDs / 2);
  startLEDRight = startLEDLeft;
  if ((startLEDLeft % 2) == 0) {
    startLEDLeft --;
  }
  for (int i=0; i < numberToLight; i++) {
    LEDs[(startLEDLeft - i)] = ledColorActive;
    LEDs[(startLEDRight + i)] = ledColorActive;
  }
}

// =========================================================================
//Get number of LEDs to light up from start of LED strip, based on interval
// =========================================================================
void updateFullStrip(int curDistance) {
  byte numberToLight = 1;
  fill_solid(LEDs, numLEDs, CRGB::Black);
  numberToLight = (startDistance - curDistance) / intervalDistance;
  if (numberToLight ==0 ) numberToLight = 1;  //Assure at least 1 light if integer truncation results in 0
  for (int i=0; i < numberToLight; i++) {
    LEDs[i] = ledColorActive;
  }
}
// =========================================================================
// Get number of LEDs to light up from end of LED strip, based on interval
// =========================================================================
void updateFullStripInv(int curDistance) {
  byte numberToLight = 1;
  fill_solid(LEDs, numLEDs, ledColorOff);
  numberToLight = (startDistance - curDistance) / intervalDistance;
  if (numberToLight ==0 ) numberToLight = 1;  //Assure at least 1 light if integer truncation results in 0
  for (int i=0; i < numberToLight; i++) {
    LEDs[((numLEDs - i)- 1)] = ledColorActive;
  }
}

void updateSolid(int curDistance) {
  fill_solid(LEDs, numLEDs, ledColorOff);
  if ((curDistance > startDistance) && (curDistance <= wakeDistance)) {
    fill_solid(LEDs, numLEDs, ledColorWake); 
  } else if ((curDistance > parkDistance) && (curDistance <= startDistance)) {
    fill_solid(LEDs, numLEDs, ledColorActive);
  } else if ((curDistance > backupDistance) && (curDistance <= parkDistance)) {
    fill_solid(LEDs, numLEDs, ledColorParked);
  }
}

void updateSleepMode() {
  fill_solid(LEDs, numLEDs, ledColorOff);
  FastLED.setBrightness(sleepBrightness);
  if (showStandbyLEDs) {
    LEDs[0] = ledColorStandby;
    LEDs[numLEDs - 1] = ledColorStandby;
  }
}

// =========================================================================
//  Flash LEDs blue for 2 seconds to indicate successful boot 
// =========================================================================
void updateBOOT() {
  fill_solid(LEDs, numLEDs, ledColorStandby);
  FastLED.show();
  delay(2000);
  fill_solid(LEDs, numLEDs, ledColorOff);
  FastLED.show();
}

// =========================================================================
//  Alternate LED colors using red and green
// =========================================================================
void updateWIFIOFF() {
  fill_solid(LEDs, numLEDs, ledColorOff);
  FastLED.setBrightness(activeBrightness);
  for (int i=0; i < (numLEDs-1); i = i + 2) {
    LEDs[i] = CRGB::Red;
    LEDs[i+1] = CRGB::Green;
  }
  FastLED.show();
  delay(2000);
}