/* Detector 1189
     Bases on Fake Geiger Counter with BLE Beacon on ESP32 by Olivier LANVIN 2022
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
   Rewritten by Ben and Stephen 2024
*/
//Libraries-------------------------------------------------------
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Ticker.h>
#include <Adafruit_NeoPixel.h>


//Debug----------------------------------------------------------------

#define DEBUG 1  //1 for serial debugging, 0 for production

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif


//Pins-----------------------------------------------------------------
const int LED_PIN = 22;          // onboard led pin (depends on board type)
const int TONE_OUTPUT_PIN = 27;  // piezo buzzer pin
#define NEO_PIXELS 16            //led pin
const int tButton = 17;          // trigger button
const int flashPin = 18;         // flash



// neopixel
#define NUMPIXELS 11  //total
Adafruit_NeoPixel NeoPixels = Adafruit_NeoPixel(NUMPIXELS, NEO_PIXELS, NEO_GRB + NEO_KHZ800);
int bLED = 0;  //blue LED intensity
int gLED = 0;  //green LED intensity


//Flash
int fDelay = 300;       //flash duration
int fToneStart = 4000;  //start charge tone frequency
int fToneMax = 12000;   //max tone to reach
bool tPressed = false;  //is the trigger pressed
unsigned long button_time = 0;
unsigned long last_button_time = 0;


//BLueTooth strength
const int T_DISTANCE = 200;
const int MIN_RSSI = -100;
const int MAX_RSSI = -50;
const int MIN_RAD = 5;
const int MAX_RAD = 255;
const int MAX_DELAY_MILLISECONDS = 5000;
// Make this much less than the max-delay to be robust to missed scans.
const int SCAN_FREQUENCY_SECONDS = 1;


static char targetBeaconName[] = "Radioactive";
static char detectedBeaconName[] = "XXXXXXXXXXX";
static int detectedStrengthRssi = MIN_RSSI;
static long int lastDetectedTimestamp = 0;
static bool connected = false;
static BLEScan* pBLEScan;
static BLEAdvertisedDevice* myDevice;


//Piezo
const int TONE_PWM_CHANNEL = 1;         // analog pwm channel
const int FREQ = 3500;                  // resonance=3550 Hz    ADAPT TO YOUR OWN TYPE OF PIEZO
int detectionThreshold = 50;            // start detection value
const int MAX_DIST = pow(MIN_RSSI, 2);  //max "distance"
int distance = MAX_DIST;



class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {

  // This will be called anytime we see an advertised device
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Check this is the device we're looking for.
    unsigned char static buf[] = "XXXXXXXXXXXXXXXXXX";
    ((String)(advertisedDevice.toString().c_str())).getBytes(buf, 18);
    for (int i = 0; i < 12; i++) {
      detectedBeaconName[i] = (char)buf[i + 6];
    }
    detectedBeaconName[11] = '\0';

    // If this is the device we're looking for...
    if (strcmp(detectedBeaconName, targetBeaconName) == 0) {
      lastDetectedTimestamp = millis();
      detectedStrengthRssi = advertisedDevice.getRSSI();
      // Save the device for later use.
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
    }
  }
};


int rssiToRad(int rssi) {
  // Constrain for safety (e.g. a zero or positive RSSI).
  int constrainedRssi = constrain(rssi, MIN_RSSI, MAX_RSSI);
  distance = pow(rssi, 2);  //set the current distance for the feedback
  int rad = map(
    // Square power to get a value that increases linearly with
    // distance (inverse square law)
    pow(constrainedRssi, 2),
    // Map from rssi^2 range (remember that rssi is negative, so
    // MAX_RSSI^2 squared is less than MIN_RSSI^2)
    pow(MAX_RSSI, 2), pow(MIN_RSSI, 2),
    // ...to range of (255, 5), which will mean that low rssi/power is
    // closer to 5.
    MAX_RAD, MIN_RAD);
  return constrain(rad, MIN_RAD, MAX_RAD);
}


class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
    debugln("Device connected");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    debugln("Device disconnected");
  }
};


void ensureConnected() {
  // If already connected, nothing to do, so return immediately.
  if (connected) {
    return;
  }

  debugln("Connecting!");
  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(myDevice);
}


void tick() {
  if ((random(distance)) < detectionThreshold) {  // pseudo-random pulse
    digitalWrite(LED_PIN, LOW);                   // flash led
    ledcWriteTone(TONE_PWM_CHANNEL, FREQ);
    delay(5);
    ledcWriteTone(TONE_PWM_CHANNEL, 0);
    digitalWrite(LED_PIN, HIGH);
  }
}

Ticker tickerSet;

//button interrupt
void IRAM_ATTR isr() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    tPressed = true;
    last_button_time = button_time;
  }
}


//setup-----------------------------------------------------------------------
void setup() {
  // Serial only used for debug messages.
  Serial.begin(4800);


  //PINS
  pinMode(LED_PIN, OUTPUT);
  pinMode(flashPin, OUTPUT);
  pinMode(TONE_OUTPUT_PIN, OUTPUT);

  pinMode(tButton, INPUT_PULLUP);
  attachInterrupt(tButton, isr, FALLING);

  digitalWrite(LED_PIN, HIGH);  // led off (depends on board type)
  digitalWrite(flashPin, LOW);

  // configure neo pixels
  NeoPixels.begin();
  NeoPixels.setBrightness(255);
  NeoPixels.show();  // Initialize all pixels to 'off'


  //PIEZO
  ledcSetup(1, 100, 8);  // 100 Hz PWM, 8-bit resolution
  ledcWrite(1, 0);
  ledcAttachPin(TONE_OUTPUT_PIN, TONE_PWM_CHANNEL);  // assign buzzer
  tickerSet.attach_ms(30, tick);                     // minimal delay between pulses


  BLEDevice::init("");
  // Create new scan
  pBLEScan = BLEDevice::getScan();
  // Set callbacks to be called anytime we see an advertised device.
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  // Active scan uses more power, but get results faster
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  // Less or equal setInterval value (buy why?)
  pBLEScan->setWindow(99);
}

//Loop---------------------------------------------------------------------------------
void loop() {

  // If we haven't seen the device advertised in a while, set the
  // strength back to the minimum
  if ((millis() - lastDetectedTimestamp) > MAX_DELAY_MILLISECONDS) {
    detectedStrengthRssi = MIN_RSSI;
  }

  // Start the scan
  pBLEScan->start(SCAN_FREQUENCY_SECONDS, false);
  // Delete results fromBLEScan buffer to release memory
  pBLEScan->clearResults();


  int rad = rssiToRad(detectedStrengthRssi);
  //debugln(rad);
  //debugln(tPressed);

  //button press
  if (tPressed == true) {
    tPressed = false;
    if (rad > T_DISTANCE) {
      ensureConnected();
      detectionThreshold = 50;
    }

    tickerSet.attach_ms(5000, tick);
    digitalWrite(flashPin, HIGH);
    delay(fDelay);
    digitalWrite(flashPin, LOW);

    detectionThreshold = 1;
    for (float freq = fToneStart; freq <= fToneMax; freq += 100) {
      ledcWriteTone(TONE_PWM_CHANNEL, freq);
      delay(20);
    }
    ledcWriteTone(TONE_PWM_CHANNEL, 0);
    //noTone(TONE_OUTPUT_PIN);
    tickerSet.attach_ms(30, tick);
    detectionThreshold = 50;
  }



  if (!connected) {
    //piezo
    if (rad <= 6) detectionThreshold = 50;
    else detectionThreshold = 2000;  // if nothing is detected set threashhold low,

    //LED OUTPUTS

    //indicators
    if (rad > 6) NeoPixels.setPixelColor(3, NeoPixels.Color(0, 0, 254));
    else NeoPixels.setPixelColor(3, NeoPixels.Color(0, 0, 0));
    if (rad > 75) NeoPixels.setPixelColor(2, NeoPixels.Color(0, 0, 254));
    else NeoPixels.setPixelColor(2, NeoPixels.Color(0, 0, 0));
    if (rad > 150) NeoPixels.setPixelColor(1, NeoPixels.Color(0, 0, 254));
    else NeoPixels.setPixelColor(1, NeoPixels.Color(0, 0, 0));
    if (rad > 200) NeoPixels.setPixelColor(0, NeoPixels.Color(0, 0, 254));
    else NeoPixels.setPixelColor(0, NeoPixels.Color(0, 0, 0));


    if (rad <= 5) {
      gLED = 0;
      bLED = 0;
    }

    else if (rad >= 6 && rad <= 125) {
      bLED = map(rad, 6, 125, 10, 255);
      gLED = 0;
    }

    else if (rad > 125) {
      bLED = map(rad, 126, 140, 255, 0);
      gLED = map(rad, 126, 200, 0, 255);
      if (bLED < 0) bLED = 0;
      if (gLED > 255) gLED = 255;
    }

    for (int k = 4; k <= NUMPIXELS; k++) {
      NeoPixels.setPixelColor(k, NeoPixels.Color(0, gLED, bLED));
    }


    NeoPixels.show();
  }


  if (connected) {
    rad = 5;
    for (int k = 0; k <= NUMPIXELS; k++) {
      NeoPixels.setPixelColor(k, NeoPixels.Color(0, 0, 0));
    }
    NeoPixels.show();
  }
  debug("Rssi ");
  debug(detectedStrengthRssi);
  debug("  rad ");
  debug(rad);
  debug("  distance ");
  debug(distance);
  debug("  d threshold ");
  debugln(detectionThreshold);
}
