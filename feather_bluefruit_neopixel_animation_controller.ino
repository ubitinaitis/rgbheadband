/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

/* IMPORTS =========================================================*/

#include <string.h>
#include <Arduino.h>
#include "FastLED.h"
#include <SPI.h>

#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_NeoPixel.h>

/* DEFINITIONS ======================================================*/

#define FACTORYRESET_ENABLE     1

#define PIN                     A0
#define DATA_PIN                A0
#define NUMPIXELS               16
#define NUM_LEDS              16

#define LED_TYPE   WS2811
#define COLOR_ORDER   GRB
#define VOLTS          12
#define MAX_MA       4000

Adafruit_NeoPixel strip(NUMPIXELS, DATA_PIN, NEO_GRB + NEO_KHZ800);

CRGBArray<NUM_LEDS> leds;

// Overall twinkle speed.
// 0 (VERY slow) to 8 (VERY fast).
// 4, 5, and 6 are recommended, default is 4.
#define TWINKLE_SPEED 6

// Overall twinkle density.
// 0 (NONE lit) to 8 (ALL lit at once).
// Default is 5.
#define TWINKLE_DENSITY 8

// How often to change color palettes.
#define SECONDS_PER_PALETTE  10

// Background color for 'unlit' pixels
//// Can be set to CRGB::Black if desired.
CRGB gBackgroundColor = CRGB::Black;
// Example of dim incandescent fairy light background color
// CRGB gBackgroundColor = CRGB(CRGB::Purple).nscale8_video(16);

// If AUTO_SELECT_BACKGROUND_COLOR is set to 1,
// then for any palette where the first two entries
// are the same, a dimmed version of that color will
// automatically be used as the background color.
#define AUTO_SELECT_BACKGROUND_COLOR 0

// If COOL_LIKE_INCANDESCENT is set to 1, colors will
// fade out slighted 'reddened', similar to how
// incandescent bulbs change color as they get dim down.
#define COOL_LIKE_INCANDESCENT 0

CRGBPalette16 gCurrentPalette;
CRGBPalette16 gTargetPalette;

/*=========================================================================*/

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN);

// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
//additional variables

//Color
uint8_t red = 100;
uint8_t green = 100;
uint8_t blue = 100;
uint8_t animationState = 1;

// for twinkle!
float redStates[NUMPIXELS];
float blueStates[NUMPIXELS];
float greenStates[NUMPIXELS];
float fadeRate = 0.96;

int pos = 0, dir = 1; // Position, direction of "eye" for larson scanner animation

void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
  .setCorrection(TypicalLEDStrip);

  chooseNextColorPalette(gTargetPalette);

  // turn off neopixel
  pixel.begin(); // This initializes the NeoPixel library.
  for (uint8_t i = 0; i < NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0, 0, 0)); // off
  }
  colorWipe(pixel.Color(100, 100, 100), 20); // do a quick colorWipe to show that the pixels are all working, even before Bluefruit connection established
  colorWipe(pixel.Color(0, 0, 0), 20);
  pixel.show();

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

// BUTTON 1: FLASHES RANDOM (DEPENDING ON SYSTEM COLOR)
// BUTTON 2: COLOR WIPES (DEPENDING ON SYSTEM COLOR)
// BUTTON 3: LARSON SCANNER
// BUTTON 4: TWINKLES

// BUTTON UP: RAINBOW CYCLE
// BUTTON DOWN: THEATRE CHASE RAINBOW
// BUTTON LEFT: THEATRE CHASE (DEPENDING ON SYSTEM COLOR)


void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  //if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  if (mySerial.available() > 0) { 
    message = mySerial.read();
  }

  // Color
  if (packetbuffer[1] == 'C') {
    red = packetbuffer[2];
    green = packetbuffer[3];
    blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);

    // this part colors ALL the pixels according to the app's color picker:
    //    for(uint8_t i=0; i<NUMPIXELS; i++) {
    //      pixel.setPixelColor(i, pixel.Color(red,green,blue));
    //   }
    //    pixel.show(); // This sends the updated pixel color to the hardware.
  }

  // Buttons
  if (packetbuffer[1] == 'B') {

    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    animationState = buttnum;
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }

  /*=========================================================================*/
  // BUTTON 1
  if (animationState == 1) { // button labeled "1" in control pad
    for (uint16_t i = 0; i < pixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    }
    flashRandom(5, random(10, 30)); // FLASHES RANDOM PRETTY LIGHTS
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
  /*=========================================================================*/

  /*=========================================================================*/
  // BUTTON 2
  if (animationState == 2) { // button labeled "2" in control pad
    for (uint16_t i = 0; i < pixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    }
    colorWipe(pixel.Color(red, green, blue), 20); // COLOR WIPES WITH THE SYSTEM'S CURRENT SELECTED COLOR
    pixel.show(); // This sends the updated pixel color to the hardware.
    colorWipe(pixel.Color(0, 0, 0), 20); // IDK WHAT THIS DOES BUT I NEED TO KEEP IT OR IT BREAKS
    pixel.show();
  }
  /*=========================================================================*/

  /*=========================================================================*/
  // BUTTON 3
  if (animationState == 3) { // button labeled "3" in control pad
    for (uint16_t i = 0; i < pixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    }
    larsonScanner(30); // larsonScanner is set to red and does not take color input!
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
  /*=========================================================================*/

  /*=========================================================================*/
  // BUTTON 4
  if (animationState == 4) { // button labeled "4" in control pad
    for (uint16_t i = 0; i < pixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    }
    EVERY_N_SECONDS( SECONDS_PER_PALETTE ) {
      chooseNextColorPalette( gTargetPalette );
    }

    EVERY_N_MILLISECONDS(10) {
      nblendPaletteTowardPalette( gCurrentPalette, gTargetPalette, 12);
    }

    drawTwinkles(leds); // TWINKLES YAYA

    FastLED.show();
  }
  /*=========================================================================*/

  /*=========================================================================*/
  // BUTTON UP
  if (animationState == 5) { // button labeled "UP" in control pad
    for (uint16_t i = 0; i < pixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    }
    rainbowCycle(5);
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
  /*=========================================================================*/

  /*=========================================================================*/
  // BUTTON DOWN
  if (animationState == 6) { // button labeled "DOWN" in control pad
    for (uint16_t i = 0; i < pixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    }
    theaterChaseRainbow(50); // larsonScanner is set to red and does not take color input.
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
  /*=========================================================================*/

  /*=========================================================================*/
  // BUTTON LEFT
  if (animationState == 7) { // button labeled "LEFT" in control pad
    for (uint16_t i = 0; i < pixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    }
    theaterChase(pixel.Color(red, green, blue), 50); // larsonScanner is set to red and does not take color input.
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
  /*=========================================================================*/

  /*=========================================================================*/
  // BUTTON RIGHT
  if (animationState == 8) { // button labeled "RIGHT" in control pad
    for (uint16_t i = 0; i < pixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    } // TURNS OFF ALL COLORS
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
}
/*=========================================================================*/


// METHODS

/*=========================================================================*/
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < pixel.numPixels(); i++) {
    pixel.setPixelColor(i, c);
    pixel.show();
    delay(wait);
  }
}
/*=========================================================================*/

/*=========================================================================*/
void larsonScanner(uint8_t wait) {
  int j;

  for (uint16_t i = 0; i < pixel.numPixels() + 5; i++) {
    // Draw 5 pixels centered on pos.  setPixelColor() will clip any
    // pixels off the ends of the strip, we don't need to watch for that.
    pixel.setPixelColor(pos - 2, 0x100000); // Dark red
    pixel.setPixelColor(pos - 1, 0x800000); // Medium red
    pixel.setPixelColor(pos    , 0xFF3000); // Center pixel is brightest
    pixel.setPixelColor(pos + 1, 0x800000); // Medium red
    pixel.setPixelColor(pos + 2, 0x100000); // Dark red

    pixel.show();
    delay(wait);

    // Rather than being sneaky and erasing just the tail pixel,
    // it's easier to erase it all and draw a new one next time.
    for (j = -2; j <= 2; j++) pixel.setPixelColor(pos + j, 0);

    // Bounce off ends of strip
    pos += dir;
    if (pos < 0) {
      pos = 1;
      dir = -dir;
    } else if (pos >= pixel.numPixels()) {
      pos = pixel.numPixels() - 2;
      dir = -dir;
    }
  }
}
/*=========================================================================*/

/*=========================================================================*/
void flashRandom(int wait, uint8_t howmany) {
  randomSeed(analogRead(0));
  for (uint16_t i = 0; i < howmany; i++) {
    // get a random pixel from the list
    int j = random(pixel.numPixels());

    // now we will 'fade' it in 5 steps
    for (int x = 0; x < 5; x++) {
      int r = red * (x + 1); r /= 5;
      int g = green * (x + 1); g /= 5;
      int b = blue * (x + 1); b /= 5;

      pixel.setPixelColor(j, pixel.Color(r, g, b));
      pixel.show();
      delay(wait);
    }
    // & fade out in 5 steps
    for (int x = 5; x >= 0; x--) {
      int r = red * x; r /= 5;
      int g = green * x; g /= 5;
      int b = blue * x; b /= 5;

      pixel.setPixelColor(j, pixel.Color(r, g, b));
      pixel.show();
      delay(wait);
    }
  }
  // LEDs will be off when done (they are faded to 0)
}
/*=========================================================================*/

/*=========================================================================*/
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i + j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}
/*=========================================================================*/

/*=========================================================================*/
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}
/*=========================================================================*/

/*=========================================================================*/
//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < pixel.numPixels(); i = i + 3) {
        pixel.setPixelColor(i + q, c);  //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (int i = 0; i < pixel.numPixels(); i = i + 3) {
        pixel.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}
/*=========================================================================*/

/*=========================================================================*/
//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {   // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < pixel.numPixels(); i = i + 3) {
        pixel.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (int i = 0; i < pixel.numPixels(); i = i + 3) {
        pixel.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}
/*=========================================================================*/

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for (int a = 0; a < 30; a++) { // Repeat 30 times...
    for (int b = 0; b < 3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for (int a = 0; a < 10; a++) { // Repeat 10 times...
    for (int b = 0; b < 3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

//====================================================================================
//====================================================================================
// START OF TWINKLE
//====================================================================================
//====================================================================================

//  This function loops over each pixel, calculates the
//  adjusted 'clock' that this pixel should use, and calls
//  "CalculateOneTwinkle" on each pixel.  It then displays
//  either the twinkle color of the background color,
//  whichever is brighter.
void drawTwinkles( CRGBSet& L)
{
  // "PRNG16" is the pseudorandom number generator
  // It MUST be reset to the same starting value each time
  // this function is called, so that the sequence of 'random'
  // numbers that it generates is (paradoxically) stable.
  uint16_t PRNG16 = 11337;

  uint32_t clock32 = millis();

  // Set up the background color, "bg".
  // if AUTO_SELECT_BACKGROUND_COLOR == 1, and the first two colors of
  // the current palette are identical, then a deeply faded version of
  // that color is used for the background color
  CRGB bg;
  if ( (AUTO_SELECT_BACKGROUND_COLOR == 1) &&
       (gCurrentPalette[0] == gCurrentPalette[1] )) {
    bg = gCurrentPalette[0];
    uint8_t bglight = bg.getAverageLight();
    if ( bglight > 64) {
      bg.nscale8_video( 16); // very bright, so scale to 1/16th
    } else if ( bglight > 16) {
      bg.nscale8_video( 64); // not that bright, so scale to 1/4th
    } else {
      bg.nscale8_video( 86); // dim, scale to 1/3rd.
    }
  } else {
    bg = gBackgroundColor; // just use the explicitly defined background color
  }

  uint8_t backgroundBrightness = bg.getAverageLight();

  for ( CRGB& pixel : L) {
    PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; // next 'random' number
    uint16_t myclockoffset16 = PRNG16; // use that number as clock offset
    PRNG16 = (uint16_t)(PRNG16 * 2053) + 1384; // next 'random' number
    // use that number as clock speed adjustment factor (in 8ths, from 8/8ths to 23/8ths)
    uint8_t myspeedmultiplierQ5_3 =  ((((PRNG16 & 0xFF) >> 4) + (PRNG16 & 0x0F)) & 0x0F) + 0x08;
    uint32_t myclock30 = (uint32_t)((clock32 * myspeedmultiplierQ5_3) >> 3) + myclockoffset16;
    uint8_t  myunique8 = PRNG16 >> 8; // get 'salt' value for this pixel

    // We now have the adjusted 'clock' for this pixel, now we call
    // the function that computes what color the pixel should be based
    // on the "brightness = f( time )" idea.
    CRGB c = computeOneTwinkle( myclock30, myunique8);

    uint8_t cbright = c.getAverageLight();
    int16_t deltabright = cbright - backgroundBrightness;
    if ( deltabright >= 32 || (!bg)) {
      // If the new pixel is significantly brighter than the background color,
      // use the new color.
      pixel = c;
    } else if ( deltabright > 0 ) {
      // If the new pixel is just slightly brighter than the background color,
      // mix a blend of the new color and the background color
      pixel = blend( bg, c, deltabright * 8);
    } else {
      // if the new pixel is not at all brighter than the background color,
      // just use the background color.
      pixel = bg;
    }
  }
}


//  This function takes a time in pseudo-milliseconds,
//  figures out brightness = f( time ), and also hue = f( time )
//  The 'low digits' of the millisecond time are used as
//  input to the brightness wave function.
//  The 'high digits' are used to select a color, so that the color
//  does not change over the course of the fade-in, fade-out
//  of one cycle of the brightness wave function.
//  The 'high digits' are also used to determine whether this pixel
//  should light at all during this cycle, based on the TWINKLE_DENSITY.
CRGB computeOneTwinkle( uint32_t ms, uint8_t salt)
{
  uint16_t ticks = ms >> (8 - TWINKLE_SPEED);
  uint8_t fastcycle8 = ticks;
  uint16_t slowcycle16 = (ticks >> 8) + salt;
  slowcycle16 += sin8( slowcycle16);
  slowcycle16 =  (slowcycle16 * 2053) + 1384;
  uint8_t slowcycle8 = (slowcycle16 & 0xFF) + (slowcycle16 >> 8);

  uint8_t bright = 0;
  if ( ((slowcycle8 & 0x0E) / 2) < TWINKLE_DENSITY) {
    bright = attackDecayWave8( fastcycle8);
  }

  uint8_t hue = slowcycle8 - salt;
  CRGB c;
  if ( bright > 0) {
    c = ColorFromPalette( gCurrentPalette, hue, bright, NOBLEND);
    if ( COOL_LIKE_INCANDESCENT == 1 ) {
      coolLikeIncandescent( c, fastcycle8);
    }
  } else {
    c = CRGB::Black;
  }
  return c;
}


// This function is like 'triwave8', which produces a
// symmetrical up-and-down triangle sawtooth waveform, except that this
// function produces a triangle wave with a faster attack and a slower decay:
//
//     / \ 
//    /     \ 
//   /         \ 
//  /             \ 
//

uint8_t attackDecayWave8( uint8_t i)
{
  if ( i < 86) {
    return i * 3;
  } else {
    i -= 86;
    return 255 - (i + (i / 2));
  }
}

// This function takes a pixel, and if its in the 'fading down'
// part of the cycle, it adjusts the color a little bit like the
// way that incandescent bulbs fade toward 'red' as they dim.
void coolLikeIncandescent( CRGB& c, uint8_t phase)
{
  if ( phase < 128) return;

  uint8_t cooling = (phase - 128) >> 4;
  c.g = qsub8( c.g, cooling);
  c.b = qsub8( c.b, cooling * 2);
}


// A pure "fairy light" palette with some brightness variations
#define HALFFAIRY ((CRGB::FairyLight & 0xFEFEFE) / 2)
#define QUARTERFAIRY ((CRGB::FairyLight & 0xFCFCFC) / 4)
const TProgmemRGBPalette16 FairyLight_p FL_PROGMEM =
{ CRGB::FairyLight, CRGB::FairyLight, CRGB::FairyLight, CRGB::FairyLight,
  HALFFAIRY,        HALFFAIRY,        CRGB::FairyLight, CRGB::FairyLight,
  QUARTERFAIRY,     QUARTERFAIRY,     CRGB::FairyLight, CRGB::FairyLight,
  CRGB::FairyLight, CRGB::FairyLight, CRGB::FairyLight, CRGB::FairyLight
};

// Add or remove palette names from this list to control which color
// palettes are used, and in what order.
const TProgmemRGBPalette16* ActivePaletteList[] = {
  //  &RetroC9_p,
  &RainbowColors_p,
  //  &FairyLight_p,
};


// Advance to the next color palette in the list (above).
void chooseNextColorPalette( CRGBPalette16& pal)
{
  const uint8_t numberOfPalettes = sizeof(ActivePaletteList) / sizeof(ActivePaletteList[0]);
  static uint8_t whichPalette = -1;
  whichPalette = addmod8( whichPalette, 1, numberOfPalettes);

  pal = *(ActivePaletteList[whichPalette]);
}

//====================================================================================
//====================================================================================
// END OF TWINKLE
//====================================================================================
//====================================================================================
