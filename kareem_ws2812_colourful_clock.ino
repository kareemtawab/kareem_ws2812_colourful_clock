// Retro 7 Segment Clock - Software update v4
// https://www.thingiverse.com/thing:3014572

#define FASTLED_ALLOW_INTERRUPTS 0

#include <DS3232RTC.h>
#include <Wire.h>
#include <FastLED.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <BH1750.h>
#include <SoftwareSerial.h>

#define MODULES 3                                           // 2 or 3 modules for HH:MM or HH:MM:SS
#define LED_PIN 6                                           // led data in connected to d6
#define LED_COUNT (MODULES * 47 + ( (MODULES - 1) * 5) )    // will return 99 leds for 2 moduls, 151 for 3 modules. Watch serial console when starting...

CRGB leds[LED_COUNT];
CRGBPalette16 currentPalette;
BH1750 lightMeter;
SoftwareSerial BTSerial(5, 4); // RX, TX

int buttonA = 2;                          // 6x6 switch, 1 pin to gnd, 1 pin to 3
int buttonB = 3;                          // 6x6 switch, 1 pin to gnd, 1 pin to 4

byte brightnessMin = 0;       //160          // brightness levels for min/medium/max
byte brightnessMed = 150;       //190          // don't change all three values and keep the one that's stored to EEPROM! (value gets stored to EEPROM, not index to brightness level)
byte brightnessMax = 254;       //240          // do not set brightnessMax above 254!
byte brightness = brightnessMin;          // brightness always at medium value when powering on/resetting (or not yet stored to EEPROM)
byte brightnessAuto = 1;                  // 1 = enable brightness corrections using a photo resistor/readBH1750();
byte intervalBH1750 = 50;                    // read value from BH1750 every 50ms
unsigned long valueBH1750LastRead = 0;       // time when we did the last readout
byte avgBH1750 = 0;                          // we will average this value somehow somewhere in readBH1750();
byte lastAvgBH1750 = 0;

byte colorMode = 1;                       // colorMode 0 uses HSV, 1 uses palettes
byte selectedPalette = 0;                 // palette to use when starting up in colorMode 1
byte saturation = 255;                    // saturation in hsv mode, 0 = no saturation, white
byte startHue = 0;                        // hsv color 0 = red (also used as "index" for the palette color when colorMode = 1)
byte displayMode = 0;                     // 0 = 12h, 1 = 24h (will be saved to EEPROM once set using buttons)
byte overlayMode = 0;                     // switch on/off (1/0) to use void colorOverlay(); (will be saved to EEPROM once set using buttons)
int overlayInterval = 10;                // interval (ms) to change colors in overlayMode

byte colorOffset = 32;                    // default distance between colors on the color wheel/palette used between segments/leds (gets changed when selecting palettes)
int colorChangeInterval = 10;           // interval (ms) to change colors when not in overlayMode (per pixel/led coloring uses overlayInterval)

int paletteChangeInterval =  0;           // interval (minutes, 0 = disable) to select a random palette using randomPalette();
unsigned long paletteLastChange = 0;      // this var will store the time when the last change has been done

byte btnRepeatCounter = 1;
byte lastKeyPressed = 0;
int btnRepeatDelay = 150;
unsigned long btnRepeatStart = 0;
int VirtualBTbutton;

byte lastSecond = 0;
unsigned long lastLoop = 0;
unsigned long lastColorChange = 0;
unsigned long lastButtonPress = 0;

/* these values will be stored to the EEPROM when set using the buttons:
  0 = selectedPalette
  1 = selectedBrightness
  2 = displayMode
  3 = overlayMode
*/

byte segGroups[14][3] = {                 // 14 segments per module, each segment has 3 leds. So lets assign them in a way we get something similar for both digits
  // right (seen from front) digit. This is which leds can be seen in which of the 7 segments
  {   6,   7,   8 },                      // top, a
  {   9,  10,  11 },                      // top right, b
  {  13,  14,  15 },                      // bottom right, c
  {  16,  17,  18 },                      // bottom, d
  {  19,  20,  21 },                      // bottom left, e
  {   3,   4,   5 },                      // top left, f
  {   0,   1,   2 },                      // center, g
  // left (seen from front) digit
  {  38,  39,  40 },                      // top, a
  {  41,  42,  43 },                      // top right, b
  {  25,  26,  27 },                      // bottom right, c
  {  28,  29,  30 },                      // bottom, d
  {  31,  32,  33 },                      // bottom left, e
  {  35,  36,  37 },                      // top left, f
  {  44,  45,  46 }                       // center, g};
};

// Using above arrays it's very easy to "talk" to the segments. Simply use 0-6 for the first 7 segments, 7-13 for the following ones per module.


byte digits[10][7] = {                    // Lets define 10 numbers (0-9) with 7 segments each, 1 = segment is on, 0 = segment is off
  {   1,   1,   1,   1,   1,   1,   0 },  // 0 -> Show segments a - f, don't show g (center one)
  {   0,   1,   1,   0,   0,   0,   0 },  // 1 -> Show segments b + c (top and bottom right), nothing else
  {   1,   1,   0,   1,   1,   0,   1 },  // 2 -> and so on...
  {   1,   1,   1,   1,   0,   0,   1 },  // 3
  {   0,   1,   1,   0,   0,   1,   1 },  // 4
  {   1,   0,   1,   1,   0,   1,   1 },  // 5
  {   1,   0,   1,   1,   1,   1,   1 },  // 6
  {   1,   1,   1,   0,   0,   0,   0 },  // 7
  {   1,   1,   1,   1,   1,   1,   1 },  // 8
  {   1,   1,   1,   1,   0,   1,   1 }   // 9
};

void setup() {
  pinMode(buttonA, INPUT_PULLUP);
  pinMode(buttonB, INPUT_PULLUP);
  BTSerial.begin(9600);
  Serial.begin(115200);
  Serial.println("7 Segment-Clock v4 starting up...");
  Serial.print("Configured for: ");  Serial.print(MODULES);  Serial.print(" modules / ");  Serial.print(LED_COUNT); Serial.println(" leds");
  Wire.begin();
  lightMeter.begin();
  setSyncProvider(RTC.get);
  setSyncInterval(15);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
  FastLED.setDither(0);
  FastLED.setBrightness(brightness);
  FastLED.clear();
  FastLED.show();
  loadValuesFromEEPROM();
}

void loop() {
  if (BTSerial.available()) {
    byte BTread = BTSerial.read();
    if (BTread == '<') {
      if (overlayMode == 0) overlayMode = 1; else overlayMode = 0;
      updateDisplay(startHue, colorOffset);
      EEPROM.write(3, overlayMode);
    }
    else if (BTread == '>') {
      lastKeyPressed = 2;
    }
    Serial.write(BTread);
  }

  if (  (lastLoop - lastColorChange >= colorChangeInterval) && (overlayMode == 0)
        || (lastLoop - lastColorChange >= overlayInterval) && (overlayMode == 1)    ) {        // update display if colorChangeInterval or overlayInterval has been reached
    startHue++;
    updateDisplay(startHue, colorOffset);
    lastColorChange = millis();
  }
  if (lastSecond != second()) {                                                             // update display if current second is different from last second drawn
    updateDisplay(startHue, colorOffset);
    lastSecond = second();
  }
  if (lastKeyPressed == 1) {
    if (brightness == brightnessMin) brightness = brightnessMed;                            // switch between brightness levels (color spacing effect when colorMode = 0 / HSV)
    else if (brightness == brightnessMed) brightness = brightnessMax;
    else if (brightness == brightnessMax) brightness = brightnessMin;
    updateDisplay(startHue, colorOffset);
    if (btnRepeatCounter >= 20) {                                                           // if button is held for a few seconds change overlayMode 0/1 (per segment, per led using void colorOverlay();
      if (overlayMode == 0) overlayMode = 1; else overlayMode = 0;
      updateDisplay(startHue, colorOffset);
      EEPROM.write(3, overlayMode);
      btnRepeatStart = millis();
    }
    EEPROM.write(1, brightness);                                                            // write brightness to EEPROM only when set using the buttons and this loop
  }
  if (lastKeyPressed == 2) {                                                                // switch between color palettes
    if (selectedPalette <= 3) selectedPalette += 1; else selectedPalette = 0;
    selectPalette(selectedPalette);
    updateDisplay(startHue, colorOffset);
    if (btnRepeatCounter >= 20) {                                                           // if button is held for a few seconds change displayMode 0/1 (12h/24h)
      if (displayMode == 0) displayMode = 1; else displayMode = 0;
      updateDisplay(startHue, colorOffset);
      EEPROM.write(2, displayMode);
      btnRepeatStart = millis();
    }
    EEPROM.write(0, selectedPalette);                                                       // write selectedPalette to EEPROM only when set using the buttons and this loop
  }
  if ( ( (lastLoop - paletteLastChange) / 1000 / 60 >= paletteChangeInterval) && (paletteChangeInterval > 0) ) {
    randomPalette(3);                                                                       // only interested in random 0-3 because 4 is single colored and boring ;)
    paletteLastChange = millis();
  }
  if ( (lastLoop - valueBH1750LastRead >= intervalBH1750) && (brightnessAuto == 1) ) {            // if BH1750 enabled and sample interval has been reached...
    readBH1750();                                                                              // ...call readBH1750();
    if (avgBH1750 != lastAvgBH1750) {                                                             // only adjust current brightness if avgBH1750 (sample over 5 values of BH1750 readout) has changed.
      updateDisplay(startHue, colorOffset);                                                 // When using this for the first time/connnecting a new BH1750 I recommend commenting this out and do a
      // Serial.println(avgBH1750); to adjust values in readBH1750();
      lastAvgBH1750 = avgBH1750;
    }
    valueBH1750LastRead = millis();
  }
  if (lastKeyPressed == 12) setupClock();                                                   // enter setup
  lastKeyPressed = readButtons();
  lastLoop = millis();
}

void readBH1750() {                                                                                                // read BH1750 value and add to tmp 5 times. The average is then assigned to avgBH1750 for use in updateDisplay();
  uint16_t readOut = 0;
  static byte runCounter = 1;
  static int tmp = 0;
  //readOut = map(lightMeter.readLightLevel(), 0, 5, 220, 0);
  Serial.print("Average BH1750 Value: "); Serial.print(lightMeter.readLightLevel()); Serial.print(", "); Serial.println(avgBH1750);   // uncomment this line and watch your avgBH1750. We want a value between ~20 at room lighting and ~200 in the dark
  if (lightMeter.readLightLevel() > 4) {
    readOut = 0;
  }
  else{
        readOut = 220;
  }

  tmp += readOut;
  if (runCounter == 5) {
    avgBH1750 = tmp / 5;
    tmp = 0; runCounter = 0;
  }
  runCounter++;
}

void randomPalette(byte palette) {
  byte newPalette = selectedPalette;                                                                              // set newPalette to the currently selected one...
  while (newPalette == selectedPalette) newPalette = random(palette);                                             // ...and randomize until we have a new one, so we don't get the same palettes twice in a row
  selectPalette(newPalette);
}

void colorOverlay() {                                                                                             // example of how to "project" colors on already drawn time/segments before showing leds in updateDisplay();
  for (byte i = 0; i < LED_COUNT; i++) {                                                                          // check each led...
    if ((colorMode == 0) && leds[i])                                                                              // ...and if it is lit and colorMode 0/HSV...
      leds[i].setHSV(startHue + (colorOffset * i), saturation, brightness);                                       // ...assign an increasing hue
    else if ((colorMode == 1) && leds[i])                                                                         // ...or if it is lit and colorMode 1/palettes...
      leds[i] = ColorFromPalette(currentPalette, startHue + (colorOffset * i), brightness, LINEARBLEND);          // ...assign increasing color from a palette
  }
}

void updateDisplay(byte color, byte colorSpacing) {                                                               // this is what redraws the "screen"
  FastLED.clear();                                                                                                // clear whatever the leds might have assigned currently...
  displayTime(color, colorSpacing);                                                                               // ...set leds to display the time...
  if (overlayMode == 1) colorOverlay();                                                                           // ...and if using overlayMode = 1 draw custom colors over single leds
  if (brightnessAuto == 1) {                                                                                      // If brightness is adjusted automatically by using readBH1750()...
    if (brightness - avgBH1750 >= 50) {                                                                            // check if resulting brightness stays above 20...
      FastLED.setBrightness(255 - avgBH1750);                                                               // ...and set brightness to current value - avgBH1750
    } else {
      FastLED.setBrightness(50);                                                                                // or set to 20 (make sure to not go < 0)
    }
  } else {                                                                                                        // If not using BH1750/auto just...
    FastLED.setBrightness(brightness);                                                                            // ...assign currently selected brightness...
  }
  FastLED.show();                                                                                                 // ...and finally make the leds light up/change visibly.
}

byte readButtons() {
  byte activeButton = 0;
  byte retVal = 0;
  if ( digitalRead(buttonA) == 0 || digitalRead(buttonB) == 0) {
    if (digitalRead(buttonA) == 0) activeButton = 1;
    else if (digitalRead(buttonB) == 0) activeButton = 2;
    if ( digitalRead(buttonA) == 0 && digitalRead(buttonB) == 0 ) activeButton = 12;
    if (millis() - lastButtonPress >= btnRepeatDelay) {
      btnRepeatStart = millis();
      btnRepeatCounter = 0;
      retVal = activeButton;
    } else if (millis() - btnRepeatStart >= btnRepeatDelay * (btnRepeatCounter + 1) ) {
      btnRepeatCounter++;
      if (btnRepeatCounter > 5) retVal = activeButton;
    }
    lastButtonPress = millis();
  }
  return retVal;
}

void setupClock() {
  byte prevColorMode = colorMode;               // store current colorMode and switch back after setup
  colorMode = 0;                                // switch to hsv mode for setup
  brightness = brightnessMax;                   // switch to maximum brightness
  tmElements_t setupTime;
  setupTime.Hour = 12;
  setupTime.Minute = 0;
  setupTime.Second = 0;
  setupTime.Day = 26;
  setupTime.Month = 7;
  setupTime.Year = 2018 - 1970;                 // yes... .Year is years since 1970.
  byte setting = 1;
  byte blinkStep = 0;
  int blinkInterval = 500;
  unsigned long lastBlink = millis();
  Serial.println("Entering setup mode...");
  FastLED.clear();
  FastLED.show();
  while (digitalRead(buttonA) == 0 || digitalRead(buttonB) == 0) delay(20);     // this will keep the display blank while any of the keys is still pressed
  while (setting <= 2) {
    while (setting == 1) {                                                      // hour setup loop
      if ( lastKeyPressed == 1 ) setting += 1;                                  // set and display hour in green and continue to minutes
      if ( lastKeyPressed == 2 ) {
        if (setupTime.Hour < 23) setupTime.Hour += 1; else setupTime.Hour = 0;  // increase hour when buttonB is pressed
      }
      if (millis() - lastBlink >= blinkInterval) {                              // pretty sure there is a much easier and nicer way
        if (blinkStep == 0) {                                                   // to get the switch between min and max brightness (boolean?)...
          brightness = brightnessMax;
          blinkStep = 1;
        } else {
          brightness = brightnessMin;
          blinkStep = 0;
        }
        lastBlink = millis();
      }
      drawSetupTime(setupTime.Hour, setupTime.Minute, 48, 160, 0, 16);
      lastKeyPressed = readButtons();
    }
    while (digitalRead(buttonA) == 0 || digitalRead(buttonB) == 0) delay(20);
    while (setting == 2) {                                                      // minute setup loop
      if ( lastKeyPressed == 1 ) {                                              // set and display hour and minutes in green and end setup
        RTC.write(setupTime);                                                   // store new time to rtc
        setting += 1;                                                           // end setup
      }
      if ( lastKeyPressed == 2 ) {  // increase minute when buttonB is pressed
        if (setupTime.Minute < 59) setupTime.Minute += 1; else setupTime.Minute = 0;
      }
      if (millis() - lastBlink >= blinkInterval) {
        if (blinkStep == 0) {
          brightness = brightnessMax;
          blinkStep = 1;
        } else {
          brightness = brightnessMin;
          blinkStep = 0;
        }
        lastBlink = millis();
      }
      lastKeyPressed = readButtons();
      drawSetupTime(setupTime.Hour, setupTime.Minute, 96, 160, 48, 0);
    }
  }
  Serial.println("Setup done...");
  drawSetupTime(setupTime.Hour, setupTime.Minute, 96, 96, 96, 0);
  time_t sync = now();                                                         // create variable sync to synchronize arduino clock to rtc
  while (digitalRead(buttonA) == 0 || digitalRead(buttonB) == 0) delay(20);
  colorMode = prevColorMode;
}

void drawSetupTime(byte setupHour, byte setupMinute, byte hourColor, byte dotColor, byte minuteColor, byte amColor) {
  // Ugly routine and basically a copy of updateDisplay only used while in setup. Should be remade using some kind of time_t structure
  // and merged to updateDisplay somehow.
  FastLED.clear();
  if (displayMode == 0) {
    if (amColor > 0) {
      if (setupHour < 12) showDots(1, dotColor); else showDots(2, dotColor); // display upper dot when time is am, both dots when pm
    }
    if (setupHour == 0) setupHour = 12;
    if (setupHour >= 13) setupHour -= 12;
    if (setupHour >= 10) showDigit(1, hourColor, 3);
    showDigit((setupHour % 10), hourColor, 2);
  } else if (displayMode == 1) {
    showDigit(setupHour / 10, hourColor, 3);
    showDigit(setupHour % 10, hourColor, 2);
  }
  showDigit((setupMinute / 10), minuteColor, 1);
  showDigit((setupMinute % 10), minuteColor, 0);
  FastLED.show();
}

void displayTime(byte color, byte colorSpacing) {
  time_t t = now();                                                   // store current time in variable t to prevent changes while updating
  byte startPos = 3;                                                  // default 2 modules -> digits 0-3, starting from the right when seen from front
  if (MODULES >= 3) startPos += 2;                                    // if more than 2 modules, set digits of to positions 0-5 and include 2 digits for seconds
  if (displayMode == 0) {
    if (hourFormat12(t) >= 10) {
      showDigit(1, color + colorSpacing * 2, startPos); startPos--;
    } else {
      startPos--;
    }
    showDigit((hourFormat12(t) % 10), color + colorSpacing * 3, startPos); startPos--;
  } else if (displayMode == 1) {
    showDigit(hour(t) / 10, color + colorSpacing * 2, startPos); startPos--;
    showDigit(hour(t) % 10, color + colorSpacing * 3, startPos); startPos--;
  }
  showDigit((minute(t) / 10), color + colorSpacing * 4, startPos); startPos--;
  showDigit((minute(t) % 10), color + colorSpacing * 5, startPos);
  if (startPos >= 2) {
    startPos--;
    showDigit((second(t) / 10), color + colorSpacing * 6, startPos); startPos--;
    showDigit((second(t) % 10), color + colorSpacing * 7, startPos);
  }
  if (second(t) % 2 == 0) {                                           // show : between hours and minutes on even seconds
    showDots(2, 96 + second(t) * 2.7);                                // : always green at 00 seconds, turning to red at 59 seconds (in colorMode = 1 start/end of palette, overlayMode overwrites this completely)
  }
}

void selectPalette(byte colorPalette) {                     // for simple custom colors have a look at case 4
  switch (colorPalette) {
    case 0:
      currentPalette = CRGBPalette16(                       // 0 - 255, intensity for (R)ed, (G)reen, (B)lue
                         CRGB(224,   0,   0),
                         CRGB(  0,   0, 244),
                         CRGB(128,   0, 128),
                         CRGB(224,   0,  64) );
      colorOffset = 48;
      break;
    case 1:
      currentPalette = CRGBPalette16(
                         CRGB(224,   0,   0),
                         CRGB(192,  64,   0),
                         CRGB(128, 128,   0),
                         CRGB(224,  32,   0) );
      colorOffset = 64;
      break;
    case 2:
      fill_rainbow(currentPalette, 16, 0, 255 / 16);        // create a simple rainbow palette, could also be done using rainbow_p, see FastLED documentation for instructions
      colorOffset = 16;
      break;
    case 3:
      currentPalette = CRGBPalette16(
                         CRGB(  0, 224,   0),
                         CRGB(  0, 192,  64),
                         CRGB(  0, 160, 160),
                         CRGB(  0, 192, 192) );
      colorOffset = 32;
      break;
    case 4:
      fill_solid(currentPalette, 16, CRGB::Blue);            // This palette will only have a single color, Blue (HTMLColorCode) in this case
      colorOffset = 1;                                       // in overlayMode this will be the mode with the lowest color spread between leds
      break;
  }
  selectedPalette = colorPalette;
}

void showSegment(byte segment, byte color, byte segDisplay) {
  /* pretty sure this could be done nicer but it'll explain somehow how to extend this to more modules
     display numbering starts with #0 for 1st digit on the first module (the one where DataIn is connected to the Arduino). */
  byte segOffset = 0;                                           //  segOffset is digit0/1 per module - remembers the arrays defined on top of this sketch?
  byte targetOffset = 0;                                        //  targetOffset is the number of leds between modules (47 per module + 5 per center part = 52), both 0 = 1st digit, module #1
  if (segDisplay == 1) {
    segOffset = 7;  //  = 2nd digit, module #1
    targetOffset =   0;
  }
  if (segDisplay == 2) {
    segOffset = 0;  //  = 1st digit, module #2
    targetOffset =  52;
  }
  if (segDisplay == 3) {
    segOffset = 7;  //  = 2nd digit, module #2
    targetOffset =  52;
  }
  if (segDisplay == 4) {
    segOffset = 0;  //  = 1st digit, module #3
    targetOffset = 104;
  }
  if (segDisplay == 5) {
    segOffset = 7;  //  = 2nd digit, module #3
    targetOffset = 104;
  }
  for (byte i = 0; i < 3; i++) {
    if (colorMode == 0) {
      leds[(segGroups[segment + segOffset][i] + targetOffset)].setHSV(color, saturation, brightness);
    } else if (colorMode == 1) {
      leds[(segGroups[segment + segOffset][i] + targetOffset)] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND);
    }
  }
}

void showDigit(byte digit, byte color, byte segDisplay) {
  for (byte i = 0; i < 7; i++) {
    if (digits[digit][i] != 0) {
      showSegment(i, color, segDisplay);
    }
  }
}

void showDots(byte dots, byte color) {
  byte dot_leds[] = {  47,  48,  50,  51,  99, 100, 102, 103 };                 // default for 2 modules, only first 4 values will be used
  byte show_leds = 0;                                                           // way too lazy to do the math now and build the array dynamically
  if (dots == 1) show_leds = 2; else if (dots == 2) show_leds = 4;
  if (MODULES == 3) {
    byte dot_leds[] = { 47,  48,  99, 100,  50,  51, 102, 103 };
    show_leds = show_leds * (MODULES - 1);
  }
  for (byte i = 0; i < show_leds; i++) {
    if (colorMode == 0) leds[dot_leds[i]].setHSV(color, saturation, brightness);
    else if (colorMode == 1) leds[dot_leds[i]] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND);
  }
}

void loadValuesFromEEPROM() {
  byte tmp = 0;
  tmp = EEPROM.read(0);
  if (tmp == 255) selectPalette(0); else selectPalette(tmp);  // Unwritten values return 255. So we know it's never been written and saved to EEPROM, so let's choose palette 0. Also...
  tmp = EEPROM.read(1);
  if (tmp != 255) brightness = tmp;                           // ...this is the reason why you should know exactly what you're doing when setting brightnessMax = 255!
  tmp = EEPROM.read(2);
  if (tmp != 255) displayMode = tmp;
  tmp = EEPROM.read(3);
  if (tmp != 255) overlayMode = tmp;
}
