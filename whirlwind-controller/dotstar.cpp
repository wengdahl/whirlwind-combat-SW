#include "dotstar.h"

#include <Adafruit_DotStar.h>

#include "controller_pins.h"

#define NUMPIXELS 20 // Number of LEDs in strip

//Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
// Hardware SPI is a little faster, but must be wired to specific pins
// TODO debug why this isn't working out of the box
// Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BRG);

void dotstar_init() {
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
}

// Runs 10 LEDs at a time along strip, cycling through red, green and blue.
// This requires about 200 mA for all the 'on' pixels + 1 mA per 'off' pixel.

int      head  = 0, tail = -10; // Index of first 'on' and 'off' pixels
uint32_t color = 0x0F0000;      // 'On' color (starts red)

void dotstar_cycle() {
  strip.setPixelColor(head, color); // 'On' pixel at head
  strip.setPixelColor(tail, 0);     // 'Off' pixel at tail
  strip.show();                     // Refresh strip

  if(++head >= NUMPIXELS) {         // Increment head index.  Off end of strip?
    head = 0;                       //  Yes, reset head index to start
    if((color >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
      color = 0x0F0000;             //   Yes, reset to red
  }
  if(++tail >= NUMPIXELS) tail = 0; // Increment, reset tail index
}

void dotstar_set_red(){
  strip.fill(0x0F0000, head, NUMPIXELS);
  strip.show();                     // Refresh strip
}