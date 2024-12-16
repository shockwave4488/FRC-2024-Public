#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel frameLeftStrip = Adafruit_NeoPixel(35, 2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel frameRightStrip = Adafruit_NeoPixel(36, 3, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel dragonStrip = Adafruit_NeoPixel(26, 5, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel armLeftStrip = Adafruit_NeoPixel(23, 6, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel armRightStrip = Adafruit_NeoPixel(23, 4, NEO_GRB + NEO_KHZ800);

// The Raspberry Pi thinks all of the strips are on a single strip, in this order:
#define NUM_STRIPS 5
Adafruit_NeoPixel* strips[] = {&frameLeftStrip, &frameRightStrip, &dragonStrip, &armLeftStrip, &armRightStrip};

uint8_t currentStripIndex = 0;
uint8_t* currentStrip = strips[0]->getPixels();
uint16_t currentStripLen = strips[0]->numPixels() * 3;
int dataIndex = 0;

unsigned long lastFrame = 0;
bool autonomous = false;

void setup() {
  for (int i = 0; i < NUM_STRIPS; i++) {
    Adafruit_NeoPixel* strip = strips[i];
    strip->begin();
    strip->fill();
    strip->show();
  }

  Serial.begin(1000000);
}

void resetFrame() {
  currentStripIndex = 0;
  currentStrip = strips[0]->getPixels();
  currentStripLen = strips[0]->numPixels() * 3;
  dataIndex = 0;
}

void loop() {
  // Each byte of color data consists of 2 serial bytes (refer to LEDStrip#show for the bit layout)
  bool dataAvailable = (Serial.available() >= 2);

  if (autonomous) {
    if (dataAvailable) {
      autonomous = false;
      lastFrame = millis();
    } else {
      loopAutonomous();
    }
  }
  if (dataAvailable) {
    uint8_t first = Serial.read();
    uint8_t second = Serial.read();

    // Check if the first bit of first and second are equal
    // If they aren't, the first read serial byte is the second of the previous color byte
    // So a third serial byte must be read to get both serial bytes of the current color byte
    // (Caused by desync from lost data)
    if ((first & 0b10000000) != (second & 0b10000000)) {
      first = second;
      while (Serial.available() == 0);
      second = Serial.read();
      resetFrame(); // Prefer dropping the frame over a partial frame
    }

    // Check if the second bit of first is on; indicates the first color byte of the frame
    if ((first & 0b01000000) == 0) {
      resetFrame();
    }

    // Get the second half of first and second and concat them to find the value of the color byte
    currentStrip[dataIndex++] = ((first & 0xF) << 4) + (second & 0xF);

    if (dataIndex == currentStripLen) {
      currentStripIndex++;
      if (currentStripIndex == NUM_STRIPS) {
        currentStripIndex = 0;
        
        for (int i = 0; i < NUM_STRIPS; i++) {
          strips[i]->show();
        }
        lastFrame = millis();
      }
      currentStrip = strips[currentStripIndex]->getPixels();
      currentStripLen = strips[currentStripIndex]->numPixels() * 3;
      dataIndex = 0;
    }
  } else if (millis() - lastFrame > 10000) {
    autonomous = true;
  }
}

// Generic animation to show when disconnected from the raspberry pi
void loopAutonomous() {
  for (int i = 0; i < NUM_STRIPS; i++) {
    Adafruit_NeoPixel* strip = strips[i];
    if (millis() / 1000 % 2 == 0) {
      strip->fill(0xFF0000);
    } else {
      strip->fill(0);
    }
    strip->show();
  }
}
