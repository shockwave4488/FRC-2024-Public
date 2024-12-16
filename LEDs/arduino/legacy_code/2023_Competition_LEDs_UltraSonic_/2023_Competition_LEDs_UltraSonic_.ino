#include <Adafruit_NeoPixel.h>
#include<math.h>
const int Sonic = A1;
const int PinR = 2;
const int PinL = 3;
const int Pin0 = 6;
const int Pin1 = 7;
const int Pin2 = 8;
const int NumLed = 48;
const int InchWorm = 20;
const int StripeLength = 6;
const int SlowBurst = 1;
const int Burst = 1;
const int Dash = 25;
const int tail = 8;
const int slomo = 20;
const int brightness = 255;
Adafruit_NeoPixel stripR(NumLed, PinR, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripL(NumLed, PinL, NEO_GRB + NEO_KHZ800);

long unsigned count = 0;
bool stickyDash = false;

// Control Functions

void setup() {
  stripR.begin();
  stripL.begin();
  Serial.begin(115200);
  pinMode(Sonic, INPUT);
  pinMode(Pin0, INPUT_PULLUP);
  pinMode(Pin1, INPUT_PULLUP);
  pinMode(Pin2, INPUT_PULLUP);
}

void loop() {
  if (analogRead(Sonic) >= 16) {
    rainbowDash();
  } else {
    switch (readInputs()) {
      case 0:
        seismic();
        stripR.show();
        stripL.show();
        delay(75);
        break;
      case 1:
        wiper();
        rainbowDash();
        stickyDash = true;
        break;
      case 2:
        yWiper();
        break;
      case 3:
        pWiper();
        break;
      case 4:
        rLightning();
        break;
      case 5:
        bLightning();
        break;
      case 6:
        break;
      case 7:
        seismic();
        stripR.show();
        stripL.show();
        delay(75);
        break;
      default:
        DNADoubleOffset(count);
        stripR.show();
        stripL.show();
        delay(75);
        break;
    }
    count++;
  }
}

int readInputs() {
  int output = 0;
  if (digitalRead(Pin0)) {
    output += 1;
  }
  if (digitalRead(Pin1)) {
    output += 2;
  }
  if (digitalRead(Pin2)) {
    output += 4;
  }
  return output;
}

void dramaticPause(int cycles) {
  for (int i = 0; i < cycles; i++) {
    seismic();
    stripR.show();
    stripL.show();
    delay(75);
    count++;
  }
}

// Pattern Functions
void pinkiePie() {
  for (int i = 0; i <= NumLed / 4; i++) {
    yellow(i);
    off(i - tail);
    yellow(NumLed + 1 - i);
    off(NumLed + 1 - i + tail);
    stripR.show();
    stripL.show();
    delay(Burst);
  }
  for (int i = (NumLed / 4) + 1; i <= NumLed / 2 + ( tail / 2); i++) {
    yellow(i);
    off(i - tail);
    yellow(NumLed + 1 - i);
    off(NumLed + 1 - i + tail);
    stripR.show();
    stripL.show();
    delay(SlowBurst);
  }
  for (int i = 0; i <= NumLed / 4; i++) {
    orange(i);
    off(i - tail);
    orange(NumLed + 1 - i);
    off(NumLed + 1 - i + tail);
    stripR.show();
    stripL.show();
    delay(Burst);
  }
  for (int i = (NumLed / 4) + 1; i <= NumLed / 2 + ( tail / 2); i++) {
    orange(i);
    off(i - tail);
    orange(NumLed + 1 - i);
    off(NumLed + 1 - i + tail);
    stripR.show();
    stripL.show();
    delay(SlowBurst);
  }
  for (int i = 0; i <= NumLed / 4; i++) {
    red(i);
    off(i - tail);
    red(NumLed + 1 - i);
    off(NumLed + 1 - i + tail);
    stripR.show();
    stripL.show();
    delay(Burst);
  }
  for (int i = (NumLed / 4) + 1; i <= NumLed / 2 + ( tail / 2); i++) {
    red(i);
    off(i - tail);
    red(NumLed + 1 - i);
    off(NumLed + 1 - i + tail);
    stripR.show();
    stripL.show();
    delay(SlowBurst);
  }
  for (int i = NumLed / 2; i >= NumLed / 4; i--) {
    chameleon1(i);
    off(i + tail);
    chameleon1(NumLed + 1 - i);
    off(NumLed + 1 - i - tail);
    stripR.show();
    stripL.show();
    delay(SlowBurst);
  }
  for (int i = NumLed / 4; i >= 0 - tail; i--) {
    chameleon1(i);
    off(i + tail);
    chameleon1(NumLed + 1 - i);
    off(NumLed + 1 - i - tail);
    stripR.show();
    stripL.show();
    delay(Burst);
  }
  delay(500);
}
void wiper() {
  for (int i = 0; i < NumLed; i++) {
    off(i);
    delay(0);
  }
}
//seismic
void seismic() {
  for (float i = 0; i <= NumLed / 2; i = i + 0.0625) {
    yellow(i);
    orange(pow(i, 1.1) - (pow((NumLed / 2), 1.1) - NumLed / 2));
    red(pow(i, 1.2) - (pow((NumLed / 2), 1.2) - NumLed / 2));
    dimRed(pow(i, 1.3) - (pow((NumLed / 2), 1.3) - NumLed / 2));
    yellow(NumLed + 1 - i);
    orange(NumLed + 1 - (pow(i, 1.1) - (pow((NumLed / 2), 1.1) - NumLed / 2)));
    red(NumLed + 1 - (pow(i, 1.2) - (pow((NumLed / 2), 1.2) - NumLed / 2)));
    dimRed(NumLed + 1 - (pow(i, 1.3) - (pow((NumLed / 2), 1.3) - NumLed / 2)));
    stripR.show();
    stripL.show();
    if (int (i) % 12 == 0) {
      if (analogRead(Sonic) >= 16) {
        break;
      }
    }
    delay(0.0625);
  }
  for (int i = NumLed / 2; i >= -8; i--) {
    chameleon2(i);
    dimRed(i + tail);
    chameleon2(NumLed + 1 - i);
    dimRed(NumLed + 1 - i - tail);
    stripR.show();
    stripL.show();
    if (int (i) % 12 == 0) {
      if (analogRead(Sonic) >= 16) {
        break;
      }
    }
    delay(slomo);
  }
  for (int i = 0; i < NumLed; i++) {
    dimRed(i);
    if (int (i) % 12 == 0) {
      if (analogRead(Sonic) >= 16) {
        break;
      }
    }
  }
  delay(500);
}

void rainbowDash() {
  for (int i = - Dash; i < NumLed / 3 - Dash; i++) {
    off(i);
    chameleon1(i + Dash);
    stripR.show();
    stripL.show();
    delay(Burst);
  }
  for (int i = NumLed / 3 - Dash; i < NumLed * 2 / 3 - Dash; i++) {
    off(i);
    chameleon1(i + Dash);
    stripR.show();
    stripL.show();
    delay(SlowBurst);
  }
  for (int i = NumLed * 2 / 3 - Dash; i < NumLed; i++) {
    off(i);
    chameleon1(i + Dash);
    stripR.show();
    stripL.show();
    delay(Burst);
  }
}
// DNADoubleOffset
void DNADoubleOffset(int t) {
  redR(t % NumLed);
  orangeR((t + StripeLength) % NumLed);
  yellowR((t + (2 * StripeLength)) % NumLed);
  dimRedR((t - StripeLength) % NumLed);
  redR((t + (NumLed / 2)) % NumLed);
  orangeR((t + StripeLength + (NumLed / 2)) % NumLed);
  yellowR((t + (2 * StripeLength) + (NumLed / 2)) % NumLed);
  dimRedR((t - StripeLength + (NumLed / 2)) % NumLed);
  redL((t + (NumLed / 4)) % NumLed);
  orangeL((t + StripeLength + (NumLed / 4)) % NumLed);
  yellowL((t + (2 * StripeLength) + (NumLed / 4)) % NumLed);
  dimRedL((t - StripeLength + (NumLed / 4)) % NumLed);
  redL((t + (3 * NumLed / 4)) % NumLed);
  orangeL((t + StripeLength + (3 * NumLed / 4)) % NumLed);
  yellowL((t + (2 * StripeLength) + (3 * NumLed / 4)) % NumLed);
  dimRedL((t - StripeLength + (3 * NumLed / 4)) % NumLed);
}

void DNADouble(int t) {
  red(t % NumLed);
  orange((count + StripeLength) % NumLed);
  yellow((count + (2 * StripeLength)) % NumLed);
  off((count - StripeLength) % NumLed);
  red((count + (NumLed / 2)) % NumLed);
  orange((count + StripeLength + (NumLed / 2)) % NumLed);
  yellow((count + (2 * StripeLength) + (NumLed / 2)) % NumLed);
  off((count - StripeLength + (NumLed / 2)) % NumLed);
}

void inchworm(int t) {
  int state = t % 4;
  int x = t / 2;
  if (state == 0) {
    chameleon1(x + 1);
  } else if (state == 1) {
    chameleon1(x);
  } else if (state == 2) {
    off(x - InchWorm + 1);
  } else {
    off(x - InchWorm);
  }
}

// Color Functions

void chameleon1(int x) {
  if (x < (NumLed / 6)) {
    stripR.setPixelColor(x, 100, 0, 0);
    stripL.setPixelColor(x, 100, 0, 0);
  } else if (x < (NumLed / 3)) {
    stripR.setPixelColor(x, 80, 16, 0);
    stripL.setPixelColor(x, 80, 16, 0);
  } else if (x < (NumLed / 2)) {
    stripR.setPixelColor(x, 50, 50, 0);
    stripL.setPixelColor(x, 50, 50, 0);
  } else if (x < (NumLed * 2 / 3)) {
    stripR.setPixelColor(x, 0, 100, 0);
    stripL.setPixelColor(x, 0, 100, 0);
  }  else if (x < (NumLed * 5 / 6)) {
    stripR.setPixelColor(x, 0, 0, 100);
    stripL.setPixelColor(x, 0, 0, 100);
  } else if (x < NumLed) {
    stripR.setPixelColor(x, 50, 0, 70);
    stripL.setPixelColor(x, 50, 0, 70);
  }
}
void chameleon2(int x) {
  if (x < (NumLed / 6)) {
    stripR.setPixelColor(x, 250, 0, 0);
    stripL.setPixelColor(x, 250, 0, 0);
  } else if (x < (NumLed / 3)) {
    stripR.setPixelColor(x, 150, 100, 0);
    stripL.setPixelColor(x, 150, 100, 0);
  } else if (x < (NumLed * 2 / 3)) {
    stripR.setPixelColor(x, 255, 100, 0);
    stripL.setPixelColor(x, 255, 100, 0);
  } else if (x < (NumLed *   5 / 6)) {
    stripR.setPixelColor(x, 150, 100, 0);
    stripL.setPixelColor(x, 150, 100, 0);
  } else if (x < NumLed) {
    stripR.setPixelColor(x, 250, 0, 0);
    stripL.setPixelColor(x, 250, 0, 0);
  }
}

void red(int x) {
  redR(x);
  redL(x);
}

void redR(int x) {
  stripR.setPixelColor(x, 255, 0, 0);
}

void redL(int x) {
  stripL.setPixelColor(x, 255, 0, 0);
}

void orange(int x) {
  orangeR(x);
  orangeL(x);
}

void orangeR(int x) {
  stripR.setPixelColor(x, 255, 30, 0);
}

void orangeL(int x) {
  stripL.setPixelColor(x, 255, 30, 0);
}

void yellow(int x) {
  yellowR(x);
  yellowL(x);
}

void purple(int x) {
  purpleR(x);
  purpleL(x);
}

void yellowR(int x) {
  stripR.setPixelColor(x, 255, 200, 0);
}

void yellowL(int x) {
  stripL.setPixelColor(x, 255, 200, 0);
}

void purpleR(int x) {
  stripR.setPixelColor(x, 150, 0, 225);
}

void purpleL(int x) {
  stripL.setPixelColor(x, 150, 0, 225);
}
void off(int x) {
  offR(x);
  offL(x);
}

void offR(int x) {
  stripR.setPixelColor(x, 0, 0, 0);
}

void offL(int x) {
  stripL.setPixelColor(x, 0, 0, 0);
}
void dimRed(int x) {
  dimRedR(x);
  dimRedL(x);
}

void dimRedR(int x) {
  stripR.setPixelColor(x, 50, 0, 0);
}

void dimRedL(int x) {
  stripL.setPixelColor(x, 50, 0, 0);
}

//Lightning
void rLightning() {
  // volume defines both the led brightness and delay after flash
  int volMin = 5;
  int volMax = 10;
  int randomVol = random(volMin, volMax);

  // upper value should be one more than total tracks
  int randomTrack = random(1, 9);

  // lightning variables
  // use rgbw neopixel adjust the following values to tweak lightning base color
  int r = random (250, 255);
  int g = random (0, 5);
  int b = random (0, 0);
  // return 32 bit color
  uint32_t color = stripR.Color(r, g, b, 250);
  // number of flashes
  int flashCount = random (5, 15);
  // flash white brightness range - 0-255
  int flashBrightnessMin =  10;
  int flashBrightnessMax =  255;
  // flash duration range - ms
  int flashDurationMin = 5;
  int flashDurationMax = 75;
  // flash off range - ms
  int flashOffsetMin = 0;
  int flashOffsetMax = 75;
  // time to next flash range - ms
  int nextFlashDelayMin = 1;
  int nextFlashDelayMax = 30;
  // map white value to volume - louder is brighter
  int flashBrightness = map(randomVol, volMin, volMax, flashBrightnessMin, flashBrightnessMax);

  // map flash to thunder delay - invert mapping
  int thunderDelay = map(randomVol,  volMin, volMax, 1000, 250);

  // randomize pause between strikes
  // longests track length - ms
  int longestTrack = 1000;
  // intensity - closer to longestTrack is more intense
  int stormIntensity = 5000;
  long strikeDelay = random(longestTrack, stormIntensity);

  // debug serial print
  Serial.println("FLASH");
  Serial.print("Track: ");
  Serial.println(randomTrack);
  Serial.print("Volume: ");
  Serial.println(randomVol);
  Serial.print("Brightness: ");
  Serial.println(flashBrightness);
  Serial.print("Thunder delay: ");
  Serial.println(thunderDelay);
  Serial.print("Strike delay: ");
  Serial.println(strikeDelay);
  Serial.print("-");

  for (int flash = flashCount ; flash >= 0; flash -= 1) {
    // add variety to color
    int colorV = random(0, 50);
    if (colorV < 0) colorV = 0;
    // flash segments of neopixel strip
    color = stripR.Color(r, g , b, flashBrightness);
    stripR.fill(color, 20, 28);
    stripR.show();
    stripL.fill(color, 20, 28);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(color, 20, 12);
    stripR.show();
    stripL.fill(color, 20, 12);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(color, 10, 12);
    stripR.show();
    stripL.fill(color, 10, 12);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(color, 0, 12);
    stripR.show();
    stripL.fill(color, 0, 12);
    stripL.show();
    delay (random(flashDurationMin, flashDurationMax));
    stripR.clear();
    stripR.show();
    //delay(50);
    stripL.clear();
    stripL.show();
    delay (random(nextFlashDelayMin, nextFlashDelayMax));
  }
  // pause between flash and thunder
  delay (thunderDelay);

  /* trigger audio - randomize volume and track
    myPlayer.volume(randomVol);
    myPlayer.play(randomTrack);
  */
  delay(strikeDelay);
}

void bLightning() {
  // volume defines both the led brightness and delay after flash
  int volMin = 5;
  int volMax = 10;
  int randomVol = random(volMin, volMax);

  // upper value should be one more than total tracks
  int randomTrack = random(1, 9);

  // lightning variables
  // use rgbw neopixel adjust the following values to tweak lightning base color
  int r = random (0, 0);
  int g = random (0, 5);
  int b = random (250, 255);
  // return 32 bit color
  uint32_t color = stripR.Color(r, g, b, 250);
  // number of flashes
  int flashCount = random (5, 15);
  // flash white brightness range - 0-255
  int flashBrightnessMin =  10;
  int flashBrightnessMax =  255;
  // flash duration range - ms
  int flashDurationMin = 5;
  int flashDurationMax = 75;
  // flash off range - ms
  int flashOffsetMin = 0;
  int flashOffsetMax = 75;
  // time to next flash range - ms
  int nextFlashDelayMin = 1;
  int nextFlashDelayMax = 30;
  // map white value to volume - louder is brighter
  int flashBrightness = map(randomVol, volMin, volMax, flashBrightnessMin, flashBrightnessMax);

  // map flash to thunder delay - invert mapping
  int thunderDelay = map(randomVol,  volMin, volMax, 1000, 250);

  // randomize pause between strikes
  // longests track length - ms
  int longestTrack = 1000;
  // intensity - closer to longestTrack is more intense
  int stormIntensity = 5000;
  long strikeDelay = random(longestTrack, stormIntensity);

  // debug serial print
  Serial.println("FLASH");
  Serial.print("Track: ");
  Serial.println(randomTrack);
  Serial.print("Volume: ");
  Serial.println(randomVol);
  Serial.print("Brightness: ");
  Serial.println(flashBrightness);
  Serial.print("Thunder delay: ");
  Serial.println(thunderDelay);
  Serial.print("Strike delay: ");
  Serial.println(strikeDelay);
  Serial.print("-");

  for (int flash = flashCount ; flash >= 0; flash -= 1) {
    // add variety to color
    int colorV = random(0, 50);
    if (colorV < 0) colorV = 0;
    // flash segments of neopixel strip
    color = stripR.Color(r, g , b, flashBrightness);
    stripR.fill(color, 20, 28);
    stripR.show();
    stripL.fill(color, 20, 28);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(color, 20, 12);
    stripR.show();
    stripL.fill(color, 20, 12);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(color, 10, 12);
    stripR.show();
    stripL.fill(color, 10, 12);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(color, 0, 12);
    stripR.show();
    stripL.fill(color, 0, 12);
    stripL.show();
    delay (random(flashDurationMin, flashDurationMax));
    stripR.clear();
    stripR.show();
    //delay(50);
    stripL.clear();
    stripL.show();
    delay (random(nextFlashDelayMin, nextFlashDelayMax));
  }
  // pause between flash and thunder
  delay (thunderDelay);

  /* trigger audio - randomize volume and track
    myPlayer.volume(randomVol);
    myPlayer.play(randomTrack);
  */
  delay(strikeDelay);
}
//RainbowLightning
void rainbowLightning() {
  // volume defines both the led brightness and delay after flash
  int volMin = 5;
  int volMax = 10;
  int randomVol = random(volMin, volMax);

  // upper value should be one more than total tracks
  int randomTrack = random(1, 9);

  // lightning variables
  // use rgbw neopixel adjust the following values to tweak lightning base color
  int r = random (250, 255);
  int g = random (0, 5);
  int b = random (0, 0);
  // return 32 bit color
  uint32_t color = stripR.Color(r, g, b, 250);
  // number of flashes
  int flashCount = random (5, 15);
  // flash white brightness range - 0-255
  int flashBrightnessMin =  10;
  int flashBrightnessMax =  255;
  // flash duration range - ms
  int flashDurationMin = 5;
  int flashDurationMax = 75;
  // flash off range - ms
  int flashOffsetMin = 0;
  int flashOffsetMax = 75;
  // time to next flash range - ms
  int nextFlashDelayMin = 1;
  int nextFlashDelayMax = 30;
  // map white value to volume - louder is brighter
  int flashBrightness = map(randomVol, volMin, volMax, flashBrightnessMin, flashBrightnessMax);

  // map flash to thunder delay - invert mapping
  int thunderDelay = map(randomVol,  volMin, volMax, 1000, 250);

  // randomize pause between strikes
  // longests track length - ms
  int longestTrack = 1000;
  // intensity - closer to longestTrack is more intense
  int stormIntensity = 5000;
  long strikeDelay = random(longestTrack, stormIntensity);

  // debug serial print
  Serial.println("FLASH");
  Serial.print("Track: ");
  Serial.println(randomTrack);
  Serial.print("Volume: ");
  Serial.println(randomVol);
  Serial.print("Brightness: ");
  Serial.println(flashBrightness);
  Serial.print("Thunder delay: ");
  Serial.println(thunderDelay);
  Serial.print("Strike delay: ");
  Serial.println(strikeDelay);
  Serial.print("-");

  for (int flash = flashCount ; flash >= 0; flash -= 1) {
    // add variety to color
    int colorV = random(0, 50);
    if (colorV < 0) colorV = 0;
    // flash segments of neopixel strip
    color = stripR.Color(r, g , b, flashBrightness);
    stripR.fill(chameleon1, 20, 28);
    stripR.show();
    stripL.fill(color, 20, 28);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(chameleon1, 20, 12);
    stripR.show();
    stripL.fill(color, 20, 12);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(chameleon1, 10, 12);
    stripR.show();
    stripL.fill(chameleon1, 10, 12);
    stripL.show();
    delay(random(flashOffsetMin, flashOffsetMax));
    stripR.fill(chameleon1, 0, 12);
    stripR.show();
    stripL.fill(chameleon1, 0, 12);
    stripL.show();
    delay (random(flashDurationMin, flashDurationMax));
    stripR.clear();
    stripR.show();
    //delay(50);
    stripL.clear();
    stripL.show();
    delay (random(nextFlashDelayMin, nextFlashDelayMax));
  }
  // pause between flash and thunder
  delay (thunderDelay);

  /* trigger audio - randomize volume and track
    myPlayer.volume(randomVol);
    myPlayer.play(randomTrack);
  */
  delay(strikeDelay);
}

//Wiper

void yWiper() {
  for (int i = 0; i < NumLed; i++) {
    yellow(i);
    delay(0);
  }
}

void pWiper() {
  for (int i = 0; i < NumLed; i++) {
    purple(i);
    delay(0);
  }
}
