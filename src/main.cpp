#include <Arduino.h>
#include <EEPROM.h>
#include <FastLED.h>
#include "DFRobotDFPlayerMini.h"
#include <vector>

// Configuration
#define NUM_LEDS 60
#define DATA_PIN 3
#define CLOCK_PIN 12
#define BUTTON_PIN 3
#define BUSY_PIN 7
#define SOUND_ENABLED false
#define BUTTON_ENABLED false
#define DEBUG true

// Timing constants
#define CLICK_DELAY 10
#define DOUBLE_CLICK_TIME 250

// Globals
CRGB leds[NUM_LEDS];
HardwareSerial myDFSerial(1); // Use UART1 for DFPlayer
#define DFPLAYER_RX_PIN 4
#define DFPLAYER_TX_PIN 5

DFRobotDFPlayerMini myDFPlayer;
bool sound_available = false;

uint8_t hue = 0;

struct CloudConfig
{
  uint8_t magic = 133;
  uint8_t volume = 25;
  uint8_t cur_animation = 0;
  uint8_t setup_count = 0;
} config;

// Forward declarations
class Animation;
class Lightning;
class ColorRoll;
class UnicornCloud;

void buttonUpISR();
void printDFPlayerDetail(uint8_t type, int value);

// Base Animation class
class Animation
{
protected:
  unsigned long lastButtonEvent = 0;
  unsigned long doublePressWaitTime = DOUBLE_CLICK_TIME;
  bool waitingDoublePress = false;
  bool buttonHandled = true;

public:
  virtual void loop()
  {
    handleButton();
    animate();
  }

  virtual void animate()
  {
    Serial.println(F("Base animation - no action"));
  }

  virtual void begin() {}
  virtual void end() {}
  virtual void setup() {}

  virtual void handleButton()
  {
    if (waitingDoublePress && (millis() - lastButtonEvent >= doublePressWaitTime))
    {
      waitingDoublePress = false;
      buttonHandled = true;
      onSinglePress();
    }
  }

  virtual void onSinglePress();
  virtual void onDoublePress()
  {
    Serial.println(F("Double press detected - no action"));
  }

  virtual bool buttonUp(unsigned long lastPressTime)
  {
    if (millis() - lastPressTime > 100)
    {
      if (waitingDoublePress && (millis() - lastButtonEvent < doublePressWaitTime))
      {
        onDoublePress();
        lastButtonEvent = millis();
        waitingDoublePress = false;
        buttonHandled = true;
        return true;
      }
      if (millis() - lastButtonEvent > doublePressWaitTime)
      {
        lastButtonEvent = millis();
        waitingDoublePress = true;
        buttonHandled = false;
        return true;
      }
    }
    return false;
  }
};

// --- Lightning Animation ---
class Lightning : public Animation
{
private:
  CRGB *leds;
  uint16_t numLeds;
  uint8_t folder;
  int16_t filesAvailable;
  unsigned long sleepTime;

public:
  Lightning(CRGB *leds, uint16_t numLeds, unsigned long sleep = 10, uint8_t folder = 1)
      : leds(leds), numLeds(numLeds), sleepTime(sleep), folder(folder), filesAvailable(-1) {}

  void setup() override
  {
    Serial.println(F("Setup Lightning"));
    int tries = 0;
    if (sound_available)
    {
      while (++tries < 10 && filesAvailable <= 0)
      {
        Serial.print(F("Try reading thunder files: "));
        filesAvailable = myDFPlayer.readFileCountsInFolder(folder);
        Serial.println(filesAvailable);
        printDFPlayerDetail(myDFPlayer.readType(), myDFPlayer.read());
        delay(200);
      }
      Serial.print(F("Files available: "));
      Serial.println(filesAvailable);
    }
  }

  void begin() override
  {
    fill_solid(leds, numLeds, CRGB::Black);
    FastLED.show();
  }

  void end() override
  {
    if (sound_available)
      myDFPlayer.pause();
  }

  void thunder()
  {
    static unsigned long lastThunder = 0;
    if (sound_available && millis() - lastThunder > 2000)
    {
      lastThunder = millis();
      if (digitalRead(BUSY_PIN) != HIGH)
      {
        int vol = config.volume;
        while (--vol > 0)
        {
          myDFPlayer.volume(vol);
          delay(10);
        }
      }
      Serial.println(F("Thunder!"));
      int playIndex = random8() % filesAvailable;
      myDFPlayer.play(playIndex);
      myDFPlayer.volume(config.volume);
    }
  }

  void strike()
  {
    Serial.println(F("Lightning strike"));
    int pos = random16() % numLeds;
    int offset = random8() % 32;
    int direction = random8() % 2 == 0 ? -1 : 1;
    int target = pos + offset * direction;

    if (offset > 15)
      thunder();

    for (int i = pos; i != target; i += direction)
    {
      int idx = i % numLeds;
      if (idx < 0)
        idx += numLeds;

      leds[idx] = CRGB::White;
      FastLED.show();
      leds[idx] = CRGB::Black;
      delay(sleepTime);
    }
    FastLED.show();
  }

  void animate() override
  {
    Serial.println(F("Animating Lightning"));
    strike();
    delay(random16() % 1000);
  }
};

// --- Unicorn Cloud Animation ---
class UnicornCloud : public Animation
{
private:
  CRGB *leds;
  uint16_t numLeds;
  std::vector<uint8_t> ledHues;

public:
  UnicornCloud(CRGB *leds, uint16_t numLeds)
      : leds(leds), numLeds(numLeds), ledHues(numLeds)
  {
    for (uint16_t i = 0; i < numLeds; ++i)
    {
      ledHues[i] = (360 * i) / numLeds;
    }
  }

  void animate() override
  {
    Serial.println(F("Animating Unicorn Cloud"));
    for (uint16_t i = 0; i < numLeds; ++i)
    {
      leds[i] = CHSV(ledHues[i], 255, 100);
      ledHues[i] = (ledHues[i] + 1) % 256;
    }
    FastLED.show();
    delay(100);
  }
};

// --- Color Roll Animation ---
class ColorRoll : public Animation
{
private:
  uint8_t folder;
  int filesAvailable;

public:
  ColorRoll(uint8_t folder = 2) : folder(folder), filesAvailable(-1) {}

  void setup() override
  {
    Serial.println(F("Setup ColorRoll"));
    if (sound_available)
    {
      int tries = 0;
      while (++tries < 10 && filesAvailable <= 0)
      {
        filesAvailable = myDFPlayer.readFileCountsInFolder(folder);
        Serial.print(F("Files in folder (try "));
        Serial.print(tries);
        Serial.print(F("): "));
        Serial.println(filesAvailable);
        delay(200);
      }
    }
  }

  void begin() override
  {
    if (sound_available)
    {
      myDFPlayer.enableLoop();
      myDFPlayer.loopFolder(folder);
      myDFPlayer.start();
      Serial.println(F("Started lullabies"));
    }
  }

  void end() override
  {
    if (sound_available)
    {
      myDFPlayer.pause();
      myDFPlayer.disableLoop();
      Serial.println(F("Stopped lullabies"));
    }
  }

  void animate() override
  {
    Serial.println(F("Animating ColorRoll"));
    fill_solid(leds, NUM_LEDS, CHSV(hue, 255, 100));
    FastLED.show();
    delay(200);
    hue++;
  }

  void onDoublePress() override
  {
    Serial.println(F("Next lullaby"));
    myDFPlayer.next();
  }
};

// --- Create animation instances now that classes are defined ---
Lightning lightning(leds, NUM_LEDS);
ColorRoll colorRoll;
UnicornCloud unicornCloud(leds, NUM_LEDS);

#define NUM_ANIMATIONS 3
Animation *animations[NUM_ANIMATIONS] = {&lightning, &colorRoll, &unicornCloud};

// --- onSinglePress implementation ---
void Animation::onSinglePress()
{
  Serial.println(F("Switching animation"));
  animations[config.cur_animation]->end();
  config.cur_animation = (config.cur_animation + 1) % NUM_ANIMATIONS;
  animations[config.cur_animation]->begin();
  EEPROM.put(0, config);
}

// --- Utility functions ---
void printDFPlayerDetail(uint8_t type, int value)
{
  switch (type)
  {
  case TimeOut:
    Serial.println(F("Timeout"));
    break;
  case WrongStack:
    Serial.println(F("Wrong Stack"));
    break;
  case DFPlayerCardInserted:
    Serial.println(F("Card Inserted"));
    break;
  case DFPlayerCardRemoved:
    Serial.println(F("Card Removed"));
    break;
  case DFPlayerCardOnline:
    Serial.println(F("Card Online"));
    break;
  case DFPlayerPlayFinished:
    Serial.print(F("Play Finished Number: "));
    Serial.println(value);
    break;
  case DFPlayerError:
    Serial.print(F("DFPlayer Error: "));
    switch (value)
    {
    case Busy:
      Serial.println(F("Card not found"));
      break;
    case Sleeping:
      Serial.println(F("Sleeping"));
      break;
    case SerialWrongStack:
      Serial.println(F("Wrong Stack"));
      break;
    case CheckSumNotMatch:
      Serial.println(F("Checksum not match"));
      break;
    case FileIndexOut:
      Serial.println(F("File Index Out of Bound"));
      break;
    case FileMismatch:
      Serial.println(F("Cannot Find File"));
      break;
    case Advertise:
      Serial.println(F("In Advertise"));
      break;
    default:
      Serial.println(F("Unknown error"));
      break;
    }
    break;
  default:
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Starting setup"));
  EEPROM.begin(512);

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);

  if (BUTTON_ENABLED)
  {
    pinMode(BUTTON_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonUpISR, FALLING);
  }
  pinMode(BUSY_PIN, INPUT);

  EEPROM.get(0, config);
  CloudConfig defaultConfig;
  if (config.magic != defaultConfig.magic)
  {
    Serial.println(F("Loading default config"));
    config = defaultConfig;
  }

  Serial.print(F("Current animation: "));
  Serial.println(config.cur_animation);

  config.cur_animation = (config.cur_animation + 1) % NUM_ANIMATIONS;
  config.setup_count++;
  EEPROM.put(0, config);
  EEPROM.commit();

  if (SOUND_ENABLED)
  {
    myDFSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
    if (!myDFPlayer.begin(myDFSerial, false))
    {
      Serial.println(F("Failed to initialize DFPlayer"));
      sound_available = false;
    }
    else
    {
      sound_available = true;
      myDFPlayer.volume(config.volume);
      myDFPlayer.play(1);
    }
  }

  for (int i = 0; i < NUM_ANIMATIONS; ++i)
  {
    animations[i]->setup();
  }

  animations[config.cur_animation]->begin();
}

void loop()
{
  if (SOUND_ENABLED)
  {
    int vol_read = analogRead(A2);
    int vol_set = (vol_read + 15) * 30 / 1024;
    if (vol_set != config.volume)
    {
      config.volume = vol_set;
      Serial.print(F("Setting volume: "));
      Serial.println(config.volume);
      myDFPlayer.volume(config.volume);
    }
  }

  animations[config.cur_animation]->loop();
}

void buttonUpISR()
{
  static unsigned long lastPress = 0;
  Serial.println(F("Button pressed"));
  if (animations[config.cur_animation]->buttonUp(lastPress))
  {
    lastPress = millis();
  }
}
