#include <bitswap.h>

#include <chipsets.h>
#include <color.h>
#include <colorpalettes.h>
#include <colorutils.h>
#include <controller.h>
#include <cpp_compat.h>
#include <dmx.h>
#include <FastLED.h>
#include <fastled_config.h>
#include <fastled_delay.h>
#include <fastled_progmem.h>
#include <fastpin.h>
#include <fastspi.h>
#include <fastspi_bitbang.h>
#include <fastspi_dma.h>
#include <fastspi_nop.h>
#include <fastspi_ref.h>
#include <fastspi_types.h>
#include <hsv2rgb.h>
#include <led_sysdefs.h>
#include <lib8tion.h>
#include <noise.h>
#include <pixelset.h>
#include <pixeltypes.h>
#include <platforms.h>
#include <power_mgt.h>

// For the sound
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// I found out my china dfplayer mini need isAck = false
#define ISACK false

#define DEBUG false

// Storage
#include <EEPROM.h>

#include <ArduinoSTL.h>
#include <iterator>
#ifndef ARDUINO_ESP8266_NODEMCU
#include <vector>
#include <iostream>
#endif
#include <memory>

// Just to remember how to check for boards
//__AVR__, ARDUINO_AVR_PRO, ESP8266, ARDUINO_ESP8266_NODEMCU
#if defined(__AVR__)
// AVR specific code here
#elif defined(ESP8266)
// ESP8266 specific code here
#endif

#ifdef ARDUINO_ESP8266_NODEMCU
#undef F
#define F(s) s
#endif

#include "FastLED.h"

// How many leds in your strip?
#define NUM_LEDS 60
//#define NUM_LEDS 60

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
//#define DATA_PIN 5
//#define DATA_PIN 6
#define DATA_PIN 9
//#define DATA_PIN 2
#define CLOCK_PIN 12

#define BUTTON_PIN 3
//#define busyPin 10  // to DFPlayer
#define busyPin 7  // to DFPlayer

#ifdef ARDUINO_ESP8266_NODEMCU
// Using pin 16 for nodemcu => flash button onboard
#undef BUTTON_PIN
#define BUTTON_PIN 0
#endif

#define SOUND true
#define BUTTON true


#define click_delay 10
#define double_click_time 250

// MP3-Player serial connection on pin 10 and 11
//SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
//SoftwareSerial mySoftwareSerial(12, 13); // RX, TX
SoftwareSerial mySoftwareSerial(4, 5); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
bool sound_available;


unsigned char led=0;
unsigned char hue=0;
// Define the array of leds
CRGB leds[NUM_LEDS];

struct Config {
  uint8_t magic = 1337;
  uint8_t volume = 25;
  uint8_t cur_animation = 0;
};

Config config;


void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
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
          break;
      }
      break;
    default:
      break;
  }
}


class animation{
  protected:
  long button_event_last = 0;
  long double_press_time = 300;
  bool double_press_wait = false;
  bool button_event_handled = true;

  public:

    virtual void loop() {
      key_handling();
      animate();
    }

    virtual void key_handling();
    virtual void key_press();
    virtual void key_double();

    virtual void animate() {
      Serial.println(F("called basic animation - nothing will happen"));
    }
    /// Called when mode is switching to this animation
    virtual void begin() {
    }
    /// Called when mode is leaving this animation
    virtual void end() {
    }
    /// Called at the end of setup/boot procedure
    virtual void setup() {

    }

    virtual bool button_up(long last_pressed);
};

class lightning : public animation{
  private:
    CRGB *leds;
    uint16_t num_leds;
    uint8_t folder;
    short files_available; // no more than 127 thunders!
  public:
    long sleep;
    lightning(CRGB* leds, int num_leds, long sleep=10, unsigned short folder=1);
    void strike();
    void thunder();
    virtual void animate();
    virtual void begin();
    virtual void end();
    virtual void setup();
};

lightning::lightning(CRGB *leds, int num_leds, long sleep, unsigned short folder){
  this->leds = leds;
  this->num_leds = num_leds;
  this->sleep = sleep;
  this->folder = folder;
  this->files_available = -1;
}

void lightning::setup() {
  Serial.println(F("setup lightning"));
  int tries = 0;
  if (sound_available) {
    //myDFPlayer.waitAvailable();
    while(++tries < 10 && files_available <= 0) {
      Serial.print(tries);
      Serial.print(F(" reading thunder files: "));
      files_available = myDFPlayer.readFileCountsInFolder(folder);
      Serial.println(files_available);
      printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
      delay(200);
    }
    Serial.print(F("Files_available"));
    Serial.println(files_available);
  }
}

void lightning::begin() {
  for (int i = 0; i < this->num_leds; ++i) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void lightning::end() {
  myDFPlayer.pause();
}

void lightning::thunder() {
  static long last_thunder = 0;
  if (sound_available && millis() -  last_thunder > 2000) {
    last_thunder = millis();
    // Fade out last one
    if (digitalRead(busyPin) != HIGH) {
      //Serial.println("Player busy...");
      //delay(50);
      int vol = config.volume;
      while (--vol > 0) {
        myDFPlayer.volume(vol);
        delay(10);
      }
    }
    // Play sound
    Serial.println(F("Thunder"));
    int play = random8()%files_available;
    myDFPlayer.play(play);  //Play the first mp3
    myDFPlayer.volume(config.volume);
  }
}

void lightning::strike(){
  Serial.println(F("Lighting strike"));
  uint16_t pos = random16()%this->num_leds;
  uint8_t off = random8()%32;
  uint8_t direction = random8()%2;
  if (direction == 0) direction = -1;
  uint16_t target = pos + off*direction;
  if (off > 15) thunder();
  int idx;
  for (int led = pos; led != target; led+=direction){
    idx = led % this->num_leds;
    if (idx < 0) {
      idx = this->num_leds  + idx;
    }
    leds[idx] = CRGB::White;
    FastLED.show();
    leds[idx] = CRGB::Black;
    delay(sleep);
  }
  FastLED.show();
}

void lightning::animate(){
  Serial.println(F("Lightning!"));
  uint16_t wait = random16() % 1000;
  strike();
  delay(wait);
}

class unicorn_cloud : public animation{

  private:
    CRGB *leds;
    uint16_t num_leds;
    std::vector<uint8_t> led_hues;

  public:
    unicorn_cloud(CRGB* leds, uint16_t num_leds)
    : led_hues(num_leds){
      this->leds = leds;
      this->num_leds = num_leds;
      for (int i=0; i < num_leds; ++i) {
        led_hues[i] = 360*i/num_leds;
      }
    }

    virtual void init() {

    }

    virtual void animate() {
      Serial.println(F("Unicorns!"));
      for (int i=0; i < num_leds; ++i) {
        leds[i] = CHSV(led_hues[i], 255, 100);
        led_hues[i] = (led_hues[i]+1)%256;
      }
      FastLED.show();
      delay(100);
    }
};



void changeAllLeds(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
}

void colorroll() {
  changeAllLeds(CHSV(hue, 255, 100));
  FastLED.show();
  delay(200);

  hue+=1;
  if (hue>255) {
      hue-=256;
  }

}

class color_roll : public animation{
    uint8_t folder;
    uint8_t files_available;

  public:
    virtual void animate() {
      Serial.println(F("Color!"));
      colorroll();
    }

    color_roll(unsigned short folder=2){
      this->folder = folder;
      this->files_available = -1;
    }

    virtual void setup() {
      #if DEBUG
      Serial.println(F("setup color_roll"));
      #endif
      uint8_t tries = 0;
      if (sound_available) {
        //myDFPlayer.waitAvailable();
        while(++tries < 10 && files_available <= 0) {
          files_available = myDFPlayer.readFileCountsInFolder(folder);
          #if DEBUG
          Serial.print(F(" reading thunder files "));
          Serial.print(F(" (try ");Serial.print(tries));Serial.print(F("):"));
          Serial.println(files_available);
          #endif
          delay(200);
        }
        Serial.print(F("Files_available"));
        Serial.println(files_available);
      }
    }

    /// Called when mode is switching to this animation
    virtual void begin() {
      myDFPlayer.enableLoop();
      myDFPlayer.loopFolder(folder);
      myDFPlayer.start();
      Serial.println(F("Singing lulabies"));
    }
    /// Called when mode is leaving this animation
    virtual void end() {
      myDFPlayer.pause();
      myDFPlayer.disableLoop();
      Serial.println(F("Stopped singing"));
    }

    virtual void key_double() {
      Serial.println(F("Next lullaby"));
      myDFPlayer.next();
    }
};

// Animation objects
lightning lightning(leds, NUM_LEDS);
color_roll roll;
unicorn_cloud unicorn_cloud {leds, NUM_LEDS};

#define NUM_ANIMATIONS 3
animation* animations[] {
//std::vector<animation*> animations {
    &lightning,
    &roll,
    &unicorn_cloud
  };


void button_up();

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  if (BUTTON) {

  #ifdef ARDUINO_ESP8266_NODEMCU
  //pinMode(BUTTON_PIN, INPUT_PULLDOWN_16);
  pinMode(BUTTON_PIN, INPUT);

  #else
  pinMode(BUTTON_PIN, INPUT);
  #endif
  //attachInterrupt(BUTTON_PIN, button_up, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_up, FALLING);
  }


  EEPROM.get(0, config);
  Config cfg_default;
  if (config.magic != cfg_default.magic) {
    config = cfg_default;
  }


  // initialize sound output
  mySoftwareSerial.begin(9600);

  Serial.println(F("Begin clouding"));
  Serial.println(F("Initializing sound..."));

  if (SOUND && !myDFPlayer.begin(mySoftwareSerial, false)) {  //Use softwareSerial to communicate with mp3.
    #if DEBUG
    Serial.println(F("Unable to initialize sound output:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    #endif
    sound_available = false;
  } else {
    #if DEBUG
    Serial.println("Sound enabled");
    #endif
    uint8_t type = myDFPlayer.readType();
    //myDFPlayer.setTimeOut(1500);
    myDFPlayer.volume(config.volume);  //Set volume value. From 0 to 30
    sound_available = true;

    myDFPlayer.play(1);

    int volume = myDFPlayer.readVolume();
    int filecount = myDFPlayer.readFileCounts();
    #if DEBUG
    Serial.print("Volume: "); Serial.println(volume); //read current volume
    Serial.print("Files: "); Serial.println(filecount); //read all file counts in SD card
    #endif
    uint8_t type2 = myDFPlayer.readType();
    printDetail(myDFPlayer.readType(), myDFPlayer.read());
    if (type == TimeOut && type2 == TimeOut && volume == -1 && filecount == -1) {
      #if DEGBUG
      Serial.println("Sorry, guess I was wrong about sound =(");
      Serial.println("Sound disabled");
      #endif
      sound_available = false;
    }
  }

  for (int i = 0; i < NUM_ANIMATIONS; ++i) {
    animations[i]->setup();
  }

  animations[config.cur_animation]->begin();
}

void loop() {
  if( SOUND ){
  int vol_read = analogRead(2);
  int vol_set = (vol_read+15)*30/1024;
  if (vol_set != config.volume) {
    config.volume = vol_set;
    Serial.print("Setting volume to: ");
    Serial.println(config.volume);
    myDFPlayer.volume(config.volume);
  }
  }

  animations[config.cur_animation]->loop();
}

// Idea to make the button handling better
// - use a stack
// @button_up
// - put events into stack when button pressed
// - note millis on each event as age
// @ key_handling
// - if length > 1: handle double (tripe?...)
// - lock at top
// --- if age > millis() + 250 handle
// --- else just leave it
// - pop any event remaining with age > millis() + 250.... (there should be none)


bool animation::button_up(long last_pressed) {
  //TODO: I should here just create button events and interpret them as double in key handling
  // TODO: this outer if is only here as I had no capacitor at hand and an analog button....
  if (millis() -  last_pressed > 100) {
    if(double_press_wait && (millis() - button_event_last < double_press_time)) {
      key_double();
      button_event_last = millis();
      double_press_wait = false;
      button_event_handled = true;
      return true;
    }
    if(millis() - button_event_last > double_press_time) {
      button_event_last = millis();
      double_press_wait = true;
      button_event_handled = false;
      return true;
    }
  }

  return false;
}


void animation::key_handling() {
  if(double_press_wait && (millis() - button_event_last >= double_press_time)) {
    double_press_wait = false;
    button_event_handled = true;
    key_press();
  }
}
void animation::key_press() {
    Serial.println(F("Changing animations"));
    animations[config.cur_animation]->end();
    config.cur_animation = (config.cur_animation + 1) % NUM_ANIMATIONS;
    animations[config.cur_animation]->begin();
    EEPROM.put(0, config);
}
void animation::key_double(){
  Serial.println("Double press - nothing happens");
}

void button_up(){
  static long last_pressed = 0;
  Serial.println(F("Button up"));
  if (animations[config.cur_animation]->button_up(last_pressed)) {
    last_pressed = millis();
  }
}
