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

#include "Arduino.h"


// I found out my china dfplayer mini need isAck = false
#define ISACK false

#define DEBUG true

// Storage
#include <EEPROM.h>

#if defined(__AVR_ATmega328P__)  // Arduino UNO, NANO
#include "ArduinoSTL.h"
#endif

#include <iterator>
#ifndef ARDUINO_ESP8266_NODEMCU
#include <vector>
#include <iostream>
#endif
#include <memory>
#include <string>
#include <stdexcept>

// For the sound
// TODO: ESP32 does not requires (and does not like) SoftwareSerial. Use Hardwareserial instead
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

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
//#define NUM_LEDS 137 // The misfortunate led diffusor stripe has 137 leds...
#define NUM_LEDS 60 // 1m ws2812 60led/m
//#define NUM_LEDS 300 // 5m ws2812 60led/m

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
//#define DATA_PIN 5
//#define DATA_PIN 6
// #define DATA_PIN 9 // ESP32 C3 has the boot button on pin 9
#define DATA_PIN 4 // ESP32 C3 has the boot button on pin 9
//#define DATA_PIN 2
#define CLOCK_PIN 12

//#define BUTTON_PIN 3
#define BUTTON_PIN 9 // for the ESP32 C3 build in button
//#define busyPin 10  // to DFPlayer
#define busyPin 7  // to DFPlayer

#ifdef ARDUINO_ESP8266_NODEMCU
// Using pin 16 for nodemcu => flash button onboard
#undef BUTTON_PIN
#define BUTTON_PIN 0
#endif

#define SOUND false
#define BUTTON true


#define click_delay 10
#define double_click_time 250

#ifdef ESP32
  // ESP32 headers
  #undef DATA_PIN
  #define DATA_PIN 4 
  // ESP32 C3 has the boot button on pin 9
  #undef BUTTON_PIN
  #define BUTTON_PIN 9 // for the ESP32 C3 build in button

  //#include <DNSServer.h>
  #include <ESPAsyncDNSServer.h>
  #include <ESPAsyncWebServer.h>
  //#include <WiFi.h>
  #include <esp_wifi.h>			//Used for mpdu_rx_disable android workaround
  #include "ArduinoJson.h"  
  #include "AsyncJson.h"
  
  JsonVariant vari;

  void setup_esp();
#endif
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

struct CloudConfig {
  uint8_t magic = 13371;
  uint8_t volume = 25;
  uint8_t cur_animation = 0;
  uint8_t setupcount = 0;
};

CloudConfig config;


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
    // Returns the name of the animation
    virtual const char* name() {
      return "animation";
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
    virtual const char* name();
};

lightning::lightning(CRGB *leds, int num_leds, long sleep, unsigned short folder){
  this->leds = leds;
  this->num_leds = num_leds;
  this->sleep = sleep;
  this->folder = folder;
  this->files_available = -1;
}

const char* lightning::name() {
  return "Lightning";
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
  if (sound_available) {
    myDFPlayer.pause();
  }
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

    virtual const char* name() {
      return "Unicorn";
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

    virtual const char* name() {
      return "Color Roll";
    }

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
          Serial.print(F(" (try "));Serial.print(tries);Serial.print(F("):"));
          Serial.println(files_available);
          #endif
          delay(200);
        }
        Serial.print(F("Files_available"));
        Serial.println(files_available);
      }
      Serial.println(F("setup color_roll done"));
    }

    /// Called when mode is switching to this animation
    virtual void begin() {
      if (sound_available) {
        myDFPlayer.enableLoop();
        myDFPlayer.loopFolder(folder);
        myDFPlayer.start();
        Serial.println(F("Singing lulabies"));
      }
    }
    /// Called when mode is leaving this animation
    virtual void end() {
      if (sound_available) {
        myDFPlayer.pause();
        myDFPlayer.disableLoop();
        Serial.println(F("Stopped singing"));
      }
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
  
  Serial.println(F("Begin setup"));
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

  #ifdef ESP32
  if (!EEPROM.begin(512))
  {
    Serial.println("EEPROM failed to initialise");
    while (true);
  }
  else
  {
    Serial.println("EEPROM initialised");
  }
  #endif

  Serial.println(F("Read Config"));
  EEPROM.get(0, config);
  CloudConfig cfg_default;
  if (config.magic != cfg_default.magic) {
  Serial.print(F("Default CONFIG: "));
    config = cfg_default;
  }
  Serial.print(F("Animation: "));
  Serial.println(config.cur_animation);
  //config.cur_animation = (config.cur_animation + 1) % NUM_ANIMATIONS;
  config.cur_animation = (config.cur_animation + 1);
  if (config.cur_animation >= NUM_ANIMATIONS) {
    config.cur_animation = 0;
  }
  Serial.println(config.cur_animation);
  config.setupcount = (config.setupcount + 1);
  Serial.print(F("Setup: "));
  Serial.println(config.setupcount);
  EEPROM.put(0, config);
  #ifdef ESP32
  EEPROM.commit();
  #endif
  
  Serial.println(F("Write Config"));
  delay(50);

  // initialize sound output
  if (SOUND) {
    mySoftwareSerial.begin(9600);
  }

  Serial.println(F("Begin clouding"));

  if (SOUND && !myDFPlayer.begin(mySoftwareSerial, false)) {  //Use softwareSerial to communicate with mp3.
    #if DEBUG
    Serial.println(F("Unable to initialize sound output:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    #endif
    sound_available = false;
  } else {
    if (SOUND) {
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
      }else{
        sound_available = false;
      }
    }
  }

  #ifdef ESP32
  setup_esp();
  #endif

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
    Serial.println(F("Changing animations2"));
    config.cur_animation = (config.cur_animation + 1) % NUM_ANIMATIONS;
    Serial.println(config.cur_animation);
    delay(100);
    animations[config.cur_animation]->begin();
    EEPROM.put(0, config);
    #ifdef ESP32
    EEPROM.commit();
    #endif
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

#ifdef ESP32
/*********************************
**          ESP32               **
**********************************/

/**
  Roadmap
  [x] Run a webserver showing the status
  [x] API for changing the mode
  [x] Simple frontend for changing the mode
  [ ] color picker?
**/

const char *ssid = "CloudControl";  // FYI The SSID can't have a space in it.
// const char * password = "12345678"; //Atleast 8 chars
const char *password = NULL;  // no password

#define MAX_CLIENTS 4	// ESP32 supports up to 10 but I have not tested it yet
#define WIFI_CHANNEL 6	// 2.4ghz channel 6 https://en.wikipedia.org/wiki/List_of_WLAN_channels#2.4_GHz_(802.11b/g/n/ax)

const IPAddress localIP(4, 3, 2, 1);		   // the IP address the web server, Samsung requires the IP to be in public space
const IPAddress gatewayIP(4, 3, 2, 1);		   // IP address of the network should be the same as the local IP for captive portals
const IPAddress subnetMask(255, 255, 255, 0);  // no need to change: https://avinetworks.com/glossary/subnet-mask/

const String localIPURL = "http://4.3.2.1";	 // a string version of the local IP with http, used for redirecting clients to your webpage

//DNSServer dnsServer;
AsyncDNSServer dnsServer;
AsyncWebServer server(80);

void setup_dns(AsyncDNSServer &dnsServer, const IPAddress &localIP) {

	// Set the TTL for DNS response and start the DNS server
	dnsServer.setTTL(3600);
	dnsServer.start(53, "*", localIP);
}

void setup_wifi() {
  start_access_point(ssid, password, localIP, gatewayIP);
  setup_dns(dnsServer, localIP);
}

void start_access_point(const char *ssid, const char *password, const IPAddress &localIP, const IPAddress &gatewayIP) {
	// Set the WiFi mode to access point and station
	WiFi.mode(WIFI_MODE_AP);

	// Define the subnet mask for the WiFi network
	const IPAddress subnetMask(255, 255, 255, 0);

	// Configure the soft access point with a specific IP and subnet mask
	WiFi.softAPConfig(localIP, gatewayIP, subnetMask);

	// Start the soft access point with the given ssid, password, channel, max number of clients
	WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, MAX_CLIENTS);

	// Disable AMPDU RX on the ESP32 WiFi to fix a bug on Android
	esp_wifi_stop();
	esp_wifi_deinit();
	wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
	my_config.ampdu_rx_enable = false;
	esp_wifi_init(&my_config);
	esp_wifi_start();
	vTaskDelay(100 / portTICK_PERIOD_MS);  // Add a small delay
}

void connect_wifi() {
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

}

void replace_all(
    std::string& s,
    std::string const& toReplace,
    std::string const& replaceWith
) {
    std::string buf;
    std::size_t pos = 0;
    std::size_t prevPos;

    // Reserves rough estimate of final size of string.
    buf.reserve(s.size());

    while (true) {
        prevPos = pos;
        pos = s.find(toReplace, pos);
        if (pos == std::string::npos)
            break;
        buf.append(s, prevPos, pos - prevPos);
        buf += replaceWith;
        pos += toReplace.size();
    }

    buf.append(s, prevPos, s.size() - prevPos);
    s.swap(buf);
}

/************
 * HTTP Handler Functions
 ************/

const char index_html[] PROGMEM = R"=====(
  <!DOCTYPE html> <html>
    <head>
      <title>Captive Cloud Portal</title>
      <style>
        body {background-color:#0b0b48;}
        h1 {color: white;}
        h2 {color: white;}
        a {color: white;}
      </style>
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <script>
      function switch_animation(num){
        const xhttp = new XMLHttpRequest();
        xhttp.onload = function() {
          animation = JSON.parse(this.responseText);
          document.getElementById("cloudmode").innerHTML = animation.name;
        }
        xhttp.open("POST", "api/animation", true);
        xhttp.setRequestHeader("Content-type", "application/json");
        xhttp.send(JSON.stringify({"animation":num}));
      }
      </script>
    </head>
    <body>
      <h1>Welcome to the Sky!</h1>
      <h2>Cloud Mode: <span id='cloudmode'>{mode}</span> </h2>
      <p>
      {modes}
      </p>
    </body>
  </html>
)=====";

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

void http_root(AsyncWebServerRequest *request){
  // request->send(200, "text/plain", "Hello, world");
  std::string html = std::string(index_html);
  std::string modes = "";
  for(auto i = 0; i < NUM_ANIMATIONS; ++i){
    modes += string_format("\n<a onclick='switch_animation(%d)' href='#'>%s</a>", i, animations[i]->name());
  }
  replace_all(html, "{mode}", animations[config.cur_animation]->name());
  replace_all(html, "{modes}", modes);
	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html.c_str());
  response->addHeader("Cache-Control", "public,max-age=31536000");  // save this file to cache for 1 year (unless you refresh)
  request->send(response);
  Serial.println("Served Basic HTML Page");
}

void http_notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void http_api_animation(AsyncWebServerRequest *request, JsonVariant &json) {
  JsonObject jsonObj = json.as<JsonObject>();
  Serial.print("[POST] /api/animations :");
  serializeJson(jsonObj, Serial);
  Serial.println("");
  long ani = jsonObj["animation"];
  config.cur_animation = ani % NUM_ANIMATIONS;
  EEPROM.commit();
  Serial.print("Api: changing animation to ");
  Serial.println(animations[config.cur_animation]->name());
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  DynamicJsonDocument root(1024);
  root["animation"] = config.cur_animation;
  root["name"] = animations[config.cur_animation]->name();
  serializeJson(root, *response);
  request->send(response);
}

void setup_http() {
  //server.on("/", HTTP_GET, http_root);
  //server.onNotFound(http_notFound);

	//======================== Webserver ========================
	// WARNING IOS (and maybe macos) WILL NOT POP UP IF IT CONTAINS THE WORD "Success" https://www.esp8266.com/viewtopic.php?f=34&t=4398
	// SAFARI (IOS) IS STUPID, G-ZIPPED FILES CAN'T END IN .GZ https://github.com/homieiot/homie-esp8266/issues/476 this is fixed by the webserver serve static function.
	// SAFARI (IOS) there is a 128KB limit to the size of the HTML. The HTML can reference external resources/images that bring the total over 128KB
	// SAFARI (IOS) popup browser has some severe limitations (javascript disabled, cookies disabled)

	// Required
	server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });	// windows 11 captive portal workaround
	server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });								// Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :)

	// Background responses: Probably not all are Required, but some are. Others might speed things up?
	// A Tier (commonly used by modern systems)
	server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });		   // android captive portal redirect
	server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });			   // microsoft redirect
	server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });  // apple call home
	server.on("/canonical.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });	   // firefox captive portal call home
	server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });					   // firefox captive portal call home
	server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });			   // windows call home

	// B Tier (uncommon)
	//  server.on("/chrome-variations/seed",[](AsyncWebServerRequest *request){request->send(200);}); //chrome captive portal call home
	//  server.on("/service/update2/json",[](AsyncWebServerRequest *request){request->send(200);}); //firefox?
	//  server.on("/chat",[](AsyncWebServerRequest *request){request->send(404);}); //No stop asking Whatsapp, there is no internet connection
	//  server.on("/startpage",[](AsyncWebServerRequest *request){request->redirect(localIPURL);});

	// return 404 to webpage icon
	server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });	// webpage icon

	// Serve Basic HTML Page
	server.on("/", HTTP_ANY, http_root);

	// the catch all
	server.onNotFound([](AsyncWebServerRequest *request) {
		request->redirect(localIPURL);
		Serial.print("onnotfound ");
		Serial.print(request->host());	// This gives some insight into whatever was being requested on the serial monitor
		Serial.print(" ");
		Serial.print(request->url());
		Serial.print(" sent redirect to " + localIPURL + "\n");
	});

  //API
	//server.on("/api/animation", HTTP_ANY, http_api_mode);
  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/api/animation", http_api_animation);
  server.addHandler(handler);

  server.begin();
}

void setup_esp() {
  setup_wifi();
  setup_http();
}

#endif
