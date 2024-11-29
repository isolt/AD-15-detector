#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <FastLED.h>

//onst int LED_PIN = 13;  //FOR FOR MINI

static bool advertising = false;
static bool connected = false;


//LED Stuff
//#define LED_PIN A4  //RGB LED array
#define LED_PIN 4
#define NUM_LEDS 4
#define BRIGHTNESS 125
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define FRAMES_PER_SECOND 120
CRGB leds[NUM_LEDS];
uint8_t gHue = 0;  // rotating "base color" used by many of the patterns
bool startUp = true;
int gMin = 80;
int gMax = 200;
int gV = gMin;
int gChange = 1;



class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      advertising = false;
      connected = true;
      //digitalWrite(LED_PIN, LOW);
    };

    void onDisconnect(BLEServer* pServer) {
      connected = false;
      //digitalWrite(LED_PIN, HIGH);
    }
};


void ensureAdvertising() {
  if (advertising) {
    return;
  }

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setScanResponse(true);
  // Functions that help with iPhone connections issue (???)
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  advertising = true;
}


void setup() {
  // Serial only used for debug messages.
  //Serial.begin(4800);

  pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);

  BLEDevice::init("Radioactive");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  //led stuff
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  //startup blink

  for (int i = 0; i < 3; i++) {
  leds[3] = CRGB::Green;
  FastLED.show();
  delay(500);
  leds[3] = CRGB::Black;
  FastLED.show();
  delay(500);
  }
  FastLED.clear();
  FastLED.show();
}

void loop() {
  if (!connected) {
    ensureAdvertising();
    FastLED.setBrightness(0);
    FastLED.show();
    startUp = true;
  }

  if (connected) {
    if (startUp == true) {

      for (int i = 0; i < 4; i++) {
        leds[i] = CRGB::White;
        FastLED.setBrightness(0);
      }
      FastLED.show();

      for (int r = 0; r < 3; r++) {
        //delay(50);

        for (int f = 50; f <= 255; f++) {
          FastLED.setBrightness(f);
          FastLED.show();
          delay(3);
        }

        for (int f = 255; f >= 50; f--) {
          FastLED.setBrightness(f);
          FastLED.show();
          delay(3);
        }
        //delay(50);
      }
      startUp = false;
      //delay(50);

      for (int i = 0; i < 4; i++) {
        leds[i] = CRGB::Red;
      }
      FastLED.setBrightness(100);
      FastLED.show();

      for (int f = 0; f <= 255; f++) {
        FastLED.setBrightness(f);
        FastLED.show();
        delay(5);
      }
    }

    for (int i = 0; i < 4; i++) {
      leds[i].setRGB(255, gV, 0);
    }
    // send the 'leds' array out to the actual LED strip
    FastLED.show();
    // insert a delay to keep the framerate modest
    delay(50);
    gV += gChange;

    if (gV >= gMax) {
      gV = gMax;
      gChange = -1;
    }
    if (gV <= gMin) {
      gV = gMin;
      gChange = 1;
    }
  }
}
