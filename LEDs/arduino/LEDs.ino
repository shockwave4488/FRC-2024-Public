/*
----- PROTOCOL SPECIFICATION -----
Pins:
  0. Mode change & bit indicator
  1. Data: Sends bits [uint32_t modeId, uint32_t modeArg]
  2. Ready: Signals to the robot that the Arduino is ready for the next bit

To send data:
  1. Data is set to the first bit and then the bit indicator is set to HIGH
  2. The Arduino reads the bit and then sets the ready pin to HIGH
  3. Data is set to the next bit and then the bit indicator is set to LOW
  4. The Arduino reads the bit and then sets the ready pin to LOW
  5. Repeat 1-4 until all bits are sent
  6. The bit indicator and ready pins should end in LOW (so the number of bits sent must be even)
*/

#include <Adafruit_NeoPixel.h>

#define RIGHT_LED_PIN 2
#define LEFT_LED_PIN 3
#define INDICATOR_PIN 6
#define DATA_PIN 7
#define READY_PIN 8
#define LED_COUNT 48

Adafruit_NeoPixel rightStrip(LED_COUNT, RIGHT_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leftStrip(LED_COUNT, LEFT_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rightAndLeft[] = {rightStrip, leftStrip};

class LEDStrips {
private:
  Adafruit_NeoPixel* strips;
  int numStrips;
  int ledCount;

public:
  LEDStrips(Adafruit_NeoPixel* strips, int numStrips, int ledCount) : strips(strips), numStrips(numStrips), ledCount(ledCount) {}
  
  // Normal methods
  show() {
    for (int i = 0; i < numStrips; i++) {
      strips[i].show();
    }
  }

  clear() {
    for (int i = 0; i < numStrips; i++) {
      strips[i].clear();
    }
  }

  fill(uint32_t color = 0, uint16_t first = 0, uint16_t count = 0) {
    if (first < 0) {
      first = 0;
    } else if (first >= ledCount) {
      return;
    }
    if (first + count >= ledCount) {
      count = ledCount - first;
    }
    for (int i = 0; i < numStrips; i++) {
      strips[i].fill(color, first, count == 0 ? ledCount : count);
    }
  }

  setPixelColor(uint16_t index, uint32_t color) {
    if (index < 0 || index >= ledCount) {
      return;
    }
    for (int i = 0; i < numStrips; i++) {
      strips[i].setPixelColor(index, color);
    }
  }

  // Custom methods
  fillWrapping(uint32_t color, uint16_t first, uint16_t count) {
    first = first % ledCount;
    if (count >= ledCount) {
      fill(color);
    } else if (first + count <= ledCount) {
      fill(color, first, count);
    } else {
      fill(color, first, ledCount - first);
      fill(color, 0, count - (ledCount - first));
    }
  }

  fillWrapping(uint32_t color, float progress, float deltaProgress) {
    fillWrapping(color, (uint16_t) (progress * ledCount), (uint16_t) (deltaProgress * ledCount));
  }
} strips(rightAndLeft, 2, LED_COUNT);

uint32_t hsv(uint8_t hue, uint8_t sat, uint8_t value) {
  return Adafruit_NeoPixel::ColorHSV(((uint16_t) hue) << 8, sat, value);
}

float easeInOut(float progress, int level) {
  if (progress <= 0.5) {
    return pow(2 * progress, 2 * level) / 2;
  }
  return -pow(2 * (progress - 1), 2 * level) / 2 + 1;
}

class LEDMode {
public:
  virtual void loop() = 0;
};

class BlankLEDMode : public LEDMode {
public:
  BlankLEDMode(uint32_t arg) {
    strips.clear();
    strips.show();
  }

  void loop() {}
};

class SolidLEDMode : public LEDMode {
public:
  SolidLEDMode(uint32_t color) {
    strips.fill(color);
    strips.show();
  }

  void loop() {}
};

class SeismicLEDMode : public LEDMode {
private:
  const uint32_t yellow = 0xFFC800;
  const uint32_t red = 0xFF0000;
  const uint32_t dimRed = 0x320000;
  const uint32_t orange = 0xFF1E00;
  const float iterationAmount = 0.0625f;
  uint8_t phase;
  float progress;
  double phaseStart;

public:
  SeismicLEDMode(uint32_t arg) : phase(0), progress(0), phaseStart(millis()) {}

  void loop() {
    if (phase == 0) {
      loopFadeIn();
    } else if (phase == 1) {
      loopFadeOut();
    } else if (phase == 2) {
      if (millis() - phaseStart >= 500.0f) {
        nextPhase();
      }
    }
  }

  void loopFadeIn() {
    // Exec every <iterationAmount>ms
    if (phaseStart >= millis()) {
      return;
    }
    phaseStart += iterationAmount;
    float i = progress;

    // Fade in
    // 1.1, 1.2, & 1.3 are from the old Arduino code and make some colors move faster than others
    strips.setPixelColor(i, yellow);
    strips.setPixelColor(pow(i, 1.1) - (pow((LED_COUNT / 2), 1.1) - LED_COUNT / 2), orange);
    strips.setPixelColor(pow(i, 1.2) - (pow((LED_COUNT / 2), 1.2) - LED_COUNT / 2), red);
    strips.setPixelColor(pow(i, 1.3) - (pow((LED_COUNT / 2), 1.3) - LED_COUNT / 2), dimRed);
    strips.setPixelColor(LED_COUNT + 1 - i, yellow);
    strips.setPixelColor(LED_COUNT + 1 - (pow(i, 1.1) - (pow((LED_COUNT / 2), 1.1) - LED_COUNT / 2)), orange);
    strips.setPixelColor(LED_COUNT + 1 - (pow(i, 1.2) - (pow((LED_COUNT / 2), 1.2) - LED_COUNT / 2)), red);
    strips.setPixelColor(LED_COUNT + 1 - (pow(i, 1.3) - (pow((LED_COUNT / 2), 1.3) - LED_COUNT / 2)), dimRed);
    strips.show();

    progress += iterationAmount;
    if (progress > LED_COUNT / 2) {
      nextPhase();
    }
  }

  void loopFadeOut() {
    // Exec every 20ms
    if (phaseStart >= millis()) {
      return;
    }
    phaseStart += 20;
    int i = (int) progress;
    
    // Fade out
    chameleon2(i);
    strips.setPixelColor(i + 8, dimRed);
    chameleon2(LED_COUNT + 1 - i);
    strips.setPixelColor(LED_COUNT + 1 - i - 8, dimRed);
    strips.show();

    progress--;
    if (progress < -8) {
      nextPhase();
    }
  }

  void nextPhase() {
    phase++;
    if (phase >= 3) {
      phase = 0;
    }
    if (phase == 0) {
      progress = 0;
      strips.fill(dimRed);
      strips.show();
    } else if (phase == 1) {
      progress = LED_COUNT / 2;
    } else if (phase == 2) {
      progress = 0;
    }
    phaseStart = millis();
  }

  // For each 1/6: 0xFA0000, 0x966400, 0xFF6400, 0xFF6400, 0x966400, 0xFA0000
  void chameleon2(int index) {
    if (index < (LED_COUNT / 6)) {
      strips.setPixelColor(index, 0xFA0000);
    } else if (index < (LED_COUNT / 3)) {
      strips.setPixelColor(index, 0x966400);
    } else if (index < (LED_COUNT * 2 / 3)) {
      strips.setPixelColor(index, 0xFF6400);
    } else if (index < (LED_COUNT * 5 / 6)) {
      strips.setPixelColor(index, 0x966400);
    } else if (index < LED_COUNT) {
      strips.setPixelColor(index, 0xFA0000);
    }
  }
};

class StripeLEDMode : public LEDMode {
private:
  uint8_t speed;
  float progress;

public:
  StripeLEDMode(uint32_t speed) : speed(speed), progress(0) {}

  void loop() {
    strips.fillWrapping(0xFF6600, progress, 0.33f);
    strips.fillWrapping(0xFF3300, progress + 0.33f, 0.33f);
    strips.fillWrapping(0xFF0000, progress + 0.66f, 0.33f);
    strips.show();
    progress += speed / 255.0f;
    if (progress >= 1) {
      progress--;
    }
  }
};

class RainbowLEDMode : public LEDMode {
private:
  const int slideSize = 25; // This is the number of LEDs currently on, which "slide" upward
  uint8_t speed;
  unsigned long lastExec;
  int i;

public:
  RainbowLEDMode(uint32_t speed) : speed(speed), lastExec(0), i(-slideSize) {}

  void loop() {
    unsigned long time = millis();
    if (time - lastExec < speed) {
      return;
    }
    lastExec = time;
    
    if (i >= 0) {
      strips.setPixelColor(i, 0);
    }
    if (i + slideSize < LED_COUNT) {
      chameleon1(i + slideSize);
    }
    strips.show();

    i++;
    if (i >= LED_COUNT) {
      i = -slideSize;
    }
  }

  void chameleon1(int x) {
    if (x < (LED_COUNT / 6)) {
      strips.setPixelColor(x, 0x640000);
    } else if (x < (LED_COUNT / 3)) {
      strips.setPixelColor(x, 0x501000);
    } else if (x < (LED_COUNT / 2)) {
      strips.setPixelColor(x, 0x323200);
    } else if (x < (LED_COUNT * 2 / 3)) {
      strips.setPixelColor(x, 0x006400);
    }  else if (x < (LED_COUNT * 5 / 6)) {
      strips.setPixelColor(x, 0x000064);
    } else if (x < LED_COUNT) {
      strips.setPixelColor(x, 0x320046);
    }
  }
};

class FlowLEDMode : public LEDMode {
private:
  uint8_t hues[LED_COUNT];
  uint8_t velocity[LED_COUNT];
  uint8_t dir[LED_COUNT];

public:
  FlowLEDMode(uint32_t arg) {
    for (int i = 0; i < LED_COUNT; i++) {
      hues[i] = random(256);
      velocity[i] = random(-10, 11);
      dir[i] = random(0, 2) * 2 - 1;
    }
  }

  void loop() {
    uint8_t buf[LED_COUNT];
    for (int i = 0; i < LED_COUNT; i++) {
      buf[i] = ((uint16_t) hues[(i - 1 + LED_COUNT) % LED_COUNT] + (uint16_t) hues[i] + (uint16_t) hues[(i + 1) % LED_COUNT]) / 3 + velocity[i];
      velocity[i] = constrain(velocity[i] + random(0, 2) * dir[i], -10, 10);
      if (abs(velocity[i]) == 10) {
        dir[i] *= -1;
      }
    }
    for (int i = 0; i < LED_COUNT; i++) {
      hues[i] = buf[i];
      strips.setPixelColor(i, hsv(hues[i] / 16, 255, 255));
    }
    strips.show();
    delay(100);
  }
};

class BunnyLEDMode : public LEDMode {
private:
  float y;
  float ySpeed;
  uint32_t color;
  static uint32_t randColor() {
    switch (random(0, 3)) {
      case 0: return 0xFF0000;
      case 1: return 0xFF3300;
      case 2: return 0xFF6600;
    }
  }

public:
  BunnyLEDMode(uint32_t arg) : y(0), ySpeed(0.5f), color(randColor()) {}

  void loop() {
    y += ySpeed;
    ySpeed -= 0.003f;
    if (y < 0) {
      ySpeed = 0.5f;
      y = 0;
      color = randColor();
    }
    strips.clear();
    strips.fill(color, y, 5);
    strips.show();
  }
};

LEDMode* mode = nullptr;

void setup() {
  randomSeed(analogRead(3));

  rightStrip.begin();
  leftStrip.begin();
  pinMode(INDICATOR_PIN, INPUT_PULLUP);
  pinMode(DATA_PIN, INPUT_PULLUP);
  pinMode(READY_PIN, OUTPUT);
  
  digitalWrite(READY_PIN, LOW);

  mode = new SeismicLEDMode(0);
}

void loop() {
  if (digitalRead(INDICATOR_PIN)) {
    updateLEDMode();
  }
  if (mode != nullptr) {
    mode->loop();
  }
}

void updateLEDMode() {
  if (mode != nullptr) {
    delete mode;
  }
  unsigned long start = millis();
  uint32_t modeId = 0;
  uint32_t modeArg = 0;
  bool toggle = true;
  for (int i = 0; i < 64; i++) {
    while (digitalRead(INDICATOR_PIN) != toggle) {
      if (millis() - start > 1000) {
        digitalWrite(READY_PIN, LOW);
        return;
      }
    }
    int value = digitalRead(DATA_PIN);
    digitalWrite(READY_PIN, toggle);
    toggle = !toggle;
    if (i < 32) {
      modeId = (modeId << 1) + value;
    } else {
      modeArg = (modeArg << 1) + value;
    }
  }
  digitalWrite(READY_PIN, LOW);

  switch (modeId) {
    case 0:
      mode = new BlankLEDMode(modeArg);
      break;
    case 1:
      mode = new SolidLEDMode(modeArg);
      break;
    case 2:
      mode = new SeismicLEDMode(modeArg);
      break;
    case 3:
      mode = new StripeLEDMode(modeArg);
      break;
    case 4:
      mode = new RainbowLEDMode(modeArg);
      break;
  }
}