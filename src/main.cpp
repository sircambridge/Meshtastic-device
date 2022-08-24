#include <FastLED.h>

FASTLED_USING_NAMESPACE
#include "GPS.h"
#include "MeshRadio.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "airtime.h"
#include "buzz.h"
#include "configuration.h"
#include "error.h"
#include "power.h"
// #include "rom/rtc.h"
//#include "DSRRouter.h"
#include "ReliableRouter.h"
// #include "debug.h"
#include "FSCommon.h"
#include "RTC.h"
#include "SPILock.h"
#include "concurrency/OSThread.h"
#include "concurrency/Periodic.h"
#include "debug/axpDebug.h"
#include "debug/einkScan.h"
#include "debug/i2cScan.h"
#include "graphics/Screen.h"
#include "main.h"
#include "modules/Modules.h"
#include "shutdown.h"
#include "sleep.h"
#include "target_specific.h"
#include <Wire.h>
// #include <driver/rtc_io.h>

#include "mesh/http/WiFiAPClient.h"

#ifdef ARCH_ESP32
#include "mesh/http/WebServer.h"

#ifdef USE_NEW_ESP32_BLUETOOTH
#include "esp32/ESP32Bluetooth.h"
#else
#include "nimble/BluetoothUtil.h"
#endif

#endif

#if HAS_WIFI
#include "mesh/wifi/WiFiServerAPI.h"
#include "mqtt/MQTT.h"
#endif

#include "LLCC68Interface.h"
#include "RF95Interface.h"
#include "SX1262Interface.h"
#include "SX1268Interface.h"

#if HAS_BUTTON
#include "ButtonThread.h"
#endif
#include "PowerFSMThread.h"

#include <Adafruit_NeoPixel.h>

using namespace concurrency;

// We always create a screen object, but we only init it if we find the hardware
graphics::Screen *screen;

// Global power status
meshtastic::PowerStatus *powerStatus = new meshtastic::PowerStatus();

// Global GPS status
meshtastic::GPSStatus *gpsStatus = new meshtastic::GPSStatus();

// Global Node status
meshtastic::NodeStatus *nodeStatus = new meshtastic::NodeStatus();

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
#define PIN 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(120, PIN, NEO_RGB + NEO_KHZ800);



/// The I2C address of our display (if found)
uint8_t screen_found;
uint8_t screen_model;

// The I2C address of the cardkb or RAK14004 (if found)
uint8_t cardkb_found;
// 0x02 for RAK14004 and 0x00 for cardkb
uint8_t kb_model;

// The I2C address of the Faces Keyboard (if found)
uint8_t faceskb_found;

// The I2C address of the RTC Module (if found)
uint8_t rtc_found;

bool eink_found = true;

uint32_t serialSinceMsec;

bool axp192_found;

// Array map of sensor types (as array index) and i2c address as value we'll find in the i2c scan
uint8_t nodeTelemetrySensorsMap[7] = {0, 0, 0, 0, 0, 0, 0};

Router *router = NULL; // Users of router don't care what sort of subclass implements that API

const char *getDeviceName()
{
    uint8_t dmac[6];

    getMacAddr(dmac);

    // Meshtastic_ab3c or Shortname_abcd
    static char name[20];
    sprintf(name, "%02x%02x", dmac[4], dmac[5]);
    // if the shortname exists and is NOT the new default of ab3c, use it for BLE name.
    if ((owner.short_name != NULL) && (strcmp(owner.short_name, name) != 0)) {
        sprintf(name, "%s_%02x%02x", owner.short_name, dmac[4], dmac[5]);
    } else {
        sprintf(name, "Meshtastic_%02x%02x", dmac[4], dmac[5]);
    }
    return name;
}

static int32_t ledBlinker()
{
    return loop_gene();
    // return 50;
    // static bool ledOn;
    // ledOn ^= 1;

    // setLed(ledOn);

    // have a very sparse duty cycle of LED being on, unless charging, then blink 0.5Hz square wave rate to indicate that
    // return powerStatus->getIsCharging() ? 1000 : (ledOn ? 1 : 1000);
}

uint32_t timeLastPowered = 0;

#if HAS_BUTTON
bool ButtonThread::shutdown_on_long_stop = false;
#endif

static Periodic *ledPeriodic;
static OSThread *powerFSMthread, *buttonThread;
#if HAS_BUTTON
uint32_t ButtonThread::longPressTime = 0;
#endif

RadioInterface *rIf = NULL;

/**
 * Some platforms (nrf52) might provide an alterate version that supresses calling delay from sleep.
 */
__attribute__((weak, noinline)) bool loopCanSleep()
{
    return true;
}

#define NUM_LEDS      50
#define LED_TYPE   WS2812
#define COLOR_ORDER   GRB
#define DATA_PIN        4
//#define CLK_PIN       4
#define VOLTS          5
#define MAX_MA       300
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          200
#define FRAMES_PER_SECOND  120

typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; 

// CRGBArray<NUM_LEDS> leds;
void setup()
{
    concurrency::hasBeenSetup = true;
    // setCPUFast(false);
    // clock_prescale_set(clock_div_1);

    // strip.begin();
    // strip.setBrightness(50);
    // strip.show();

    FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
    FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

#ifdef SEGGER_STDOUT_CH
    auto mode = false ? SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL : SEGGER_RTT_MODE_NO_BLOCK_TRIM;
#ifdef NRF52840_XXAA
    auto buflen = 4096; // this board has a fair amount of ram
#else
    auto buflen = 256; // this board has a fair amount of ram
#endif
    SEGGER_RTT_ConfigUpBuffer(SEGGER_STDOUT_CH, NULL, NULL, buflen, mode);
#endif

#ifdef DEBUG_PORT
    if (!config.device.serial_disabled) {
        consoleInit(); // Set serial baud rate and init our mesh console
    }
#endif

    serialSinceMsec = millis();

    DEBUG_MSG("\n\n//\\ E S H T /\\ S T / C\n\n");

    initDeepSleep();

    // Testing this fix für erratic T-Echo boot behaviour
#if defined(TTGO_T_ECHO) && defined(PIN_EINK_PWR_ON)
    pinMode(PIN_EINK_PWR_ON, OUTPUT);
    digitalWrite(PIN_EINK_PWR_ON, HIGH);
#endif

#ifdef VEXT_ENABLE
    pinMode(VEXT_ENABLE, OUTPUT);
    digitalWrite(VEXT_ENABLE, 0); // turn on the display power
#endif

#ifdef RESET_OLED
    pinMode(RESET_OLED, OUTPUT);
    digitalWrite(RESET_OLED, 1);
#endif

    bool forceSoftAP = 0;

#ifdef BUTTON_PIN
#ifdef ARCH_ESP32

    // If the button is connected to GPIO 12, don't enable the ability to use
    // meshtasticAdmin on the device.
    pinMode(BUTTON_PIN, INPUT);

#ifdef BUTTON_NEED_PULLUP
    gpio_pullup_en((gpio_num_t)BUTTON_PIN);
    delay(10);
#endif

    // BUTTON_PIN is pulled high by a 12k resistor.
    if (!digitalRead(BUTTON_PIN)) {
        forceSoftAP = 1;
        DEBUG_MSG("Setting forceSoftAP = 1\n");
    }

#endif
#endif

    OSThread::setup();

    ledPeriodic = new Periodic("Blink", ledBlinker);

    fsInit();

    // router = new DSRRouter();
    router = new ReliableRouter();

#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
#elif HAS_WIRE
    Wire.begin();
#endif

#ifdef PIN_LCD_RESET
    // FIXME - move this someplace better, LCD is at address 0x3F
    pinMode(PIN_LCD_RESET, OUTPUT);
    digitalWrite(PIN_LCD_RESET, 0);
    delay(1);
    digitalWrite(PIN_LCD_RESET, 1);
    delay(1);
#endif

    scanI2Cdevice();
#ifdef RAK4630
    // scanEInkDevice();
#endif

#if HAS_BUTTON
    // Buttons & LED
    buttonThread = new ButtonThread();
#endif

#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, 1 ^ LED_INVERTED); // turn on for now
#endif

    // Hello
    DEBUG_MSG("Meshtastic hwvendor=%d, swver=%s\n", HW_VENDOR, optstr(APP_VERSION));

#ifdef ARCH_ESP32
    // Don't init display if we don't have one or we are waking headless due to a timer event
    if (wakeCause == ESP_SLEEP_WAKEUP_TIMER)
        screen_found = 0; // forget we even have the hardware

    esp32Setup();
#endif

#ifdef ARCH_NRF52
    nrf52Setup();
#endif
    playStartMelody();
    // We do this as early as possible because this loads preferences from flash
    // but we need to do this after main cpu iniot (esp32setup), because we need the random seed set
    nodeDB.init();

    // Currently only the tbeam has a PMU
    power = new Power();
    power->setStatusHandler(powerStatus);
    powerStatus->observe(&power->newStatus);
    power->setup(); // Must be after status handler is installed, so that handler gets notified of the initial configuration

    // Init our SPI controller (must be before screen and lora)
    initSPI();
#ifndef ARCH_ESP32
    SPI.begin();
#else
    // ESP32
    SPI.begin(RF95_SCK, RF95_MISO, RF95_MOSI, RF95_NSS);
    SPI.setFrequency(4000000);
#endif

    // Initialize the screen first so we can show the logo while we start up everything else.
    screen = new graphics::Screen(screen_found);

    readFromRTC(); // read the main CPU RTC at first (in case we can't get GPS time)

    gps = createGps();

    if (gps)
        gpsStatus->observe(&gps->newStatus);
    else
        DEBUG_MSG("Warning: No GPS found - running without GPS\n");

    nodeStatus->observe(&nodeDB.newStatus);

    service.init();

    // Now that the mesh service is created, create any modules
    setupModules();

    // Do this after service.init (because that clears error_code)
#ifdef AXP192_SLAVE_ADDRESS
    if (!axp192_found)
        RECORD_CRITICALERROR(CriticalErrorCode_NoAXP192); // Record a hardware fault for missing hardware
#endif

        // Don't call screen setup until after nodedb is setup (because we need
        // the current region name)
#if defined(ST7735_CS) || defined(USE_EINK) || defined(ILI9341_DRIVER)
    screen->setup();
#else
    if (screen_found)
        screen->setup();
#endif

    screen->print("Started...\n");

    // We have now loaded our saved preferences from flash

    // ONCE we will factory reset the GPS for bug #327
    if (gps && !devicestate.did_gps_reset) {
        DEBUG_MSG("GPS FactoryReset requested\n");
        if (gps->factoryReset()) { // If we don't succeed try again next time
            devicestate.did_gps_reset = true;
            nodeDB.saveToDisk();
        }
    }

#ifdef SX126X_ANT_SW
    // make analog PA vs not PA switch on SX126x eval board work properly
    pinMode(SX126X_ANT_SW, OUTPUT);
    digitalWrite(SX126X_ANT_SW, 1);
#endif

    // radio init MUST BE AFTER service.init, so we have our radio config settings (from nodedb init)

#if defined(RF95_IRQ)
    if (!rIf) {
        rIf = new RF95Interface(RF95_NSS, RF95_IRQ, RF95_RESET, SPI);
        if (!rIf->init()) {
            DEBUG_MSG("Warning: Failed to find RF95 radio\n");
            delete rIf;
            rIf = NULL;
        } else {
            DEBUG_MSG("RF95 Radio init succeeded, using RF95 radio\n");
        }
    }
#endif

#if defined(USE_SX1262)
    if (!rIf) {
        rIf = new SX1262Interface(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY, SPI);
        if (!rIf->init()) {
            DEBUG_MSG("Warning: Failed to find SX1262 radio\n");
            delete rIf;
            rIf = NULL;
        } else {
            DEBUG_MSG("SX1262 Radio init succeeded, using SX1262 radio\n");
        }
    }
#endif

#if defined(USE_SX1268)
    if (!rIf) {
        rIf = new SX1268Interface(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY, SPI);
        if (!rIf->init()) {
            DEBUG_MSG("Warning: Failed to find SX1268 radio\n");
            delete rIf;
            rIf = NULL;
        } else {
            DEBUG_MSG("SX1268 Radio init succeeded, using SX1268 radio\n");
        }
    }
#endif

#if defined(USE_LLCC68)
    if (!rIf) {
        rIf = new LLCC68Interface(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY, SPI);
        if (!rIf->init()) {
            DEBUG_MSG("Warning: Failed to find LLCC68 radio\n");
            delete rIf;
            rIf = NULL;
        } else {
            DEBUG_MSG("LLCC68 Radio init succeeded, using LLCC68 radio\n");
        }
    }
#endif

#if !HAS_RADIO
    if (!rIf) {
        rIf = new SimRadio;
        if (!rIf->init()) {
            DEBUG_MSG("Warning: Failed to find simulated radio\n");
            delete rIf;
            rIf = NULL;
        } else {
            DEBUG_MSG("Using SIMULATED radio!\n");
        }
    }
#endif

#if HAS_WIFI
    mqttInit();
#endif

    // Initialize Wifi
    initWifi(forceSoftAP);

#ifdef ARCH_ESP32
    // Start web server thread.
    webServerThread = new WebServerThread();
#endif

#ifdef ARCH_PORTDUINO
    initApiServer();
#endif

    // Start airtime logger thread.
    airTime = new AirTime();

    if (!rIf)
        RECORD_CRITICALERROR(CriticalErrorCode_NoRadio);
    else {
        router->addInterface(rIf);

        // Calculate and save the bit rate to myNodeInfo
        // TODO: This needs to be added what ever method changes the channel from the phone.
        myNodeInfo.bitrate = (float(Constants_DATA_PAYLOAD_LEN) / (float(rIf->getPacketTime(Constants_DATA_PAYLOAD_LEN)))) * 1000;
        DEBUG_MSG("myNodeInfo.bitrate = %f bytes / sec\n", myNodeInfo.bitrate);
    }

    // This must be _after_ service.init because we need our preferences loaded from flash to have proper timeout values
    PowerFSM_setup(); // we will transition to ON in a couple of seconds, FIXME, only do this for cold boots, not waking from SDS
    powerFSMthread = new PowerFSMThread();

    // setBluetoothEnable(false); we now don't start bluetooth until we enter the proper state
    // setCPUFast(false); // 80MHz is fine for our slow peripherals
}

uint32_t rebootAtMsec;   // If not zero we will reboot at this time (used to reboot shortly after the update completes)
uint32_t shutdownAtMsec; // If not zero we will shutdown at this time (used to shutdown from python or mobile client)

// If a thread does something that might need for it to be rescheduled ASAP it can set this flag
// This will supress the current delay and instead try to run ASAP.
bool runASAP;

void loop()
{
    // DEBUG_MSG("11\n");
    runASAP = false;

    // axpDebugOutput.loop();

    // heap_caps_check_integrity_all(true); // FIXME - disable this expensive check
    // loop_gene();
    // return;

#ifdef ARCH_ESP32
    esp32Loop();
#endif
#ifdef ARCH_NRF52
    nrf52Loop();
#endif
    powerCommandsCheck();

    // For debugging
    // if (rIf) ((RadioLibInterface *)rIf)->isActivelyReceiving();

#ifdef DEBUG_STACK
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 10 * 1000L) {
        lastPrint = millis();
        meshtastic::printThreadInfo("main");
    }
#endif

    // TODO: This should go into a thread handled by FreeRTOS.
    // handleWebResponse();

    service.loop();

    long delayMsec = mainController.runOrDelay();

    /* if (mainController.nextThread && delayMsec)
        DEBUG_MSG("Next %s in %ld\n", mainController.nextThread->ThreadName.c_str(),
                  mainController.nextThread->tillRun(millis())); */

    // We want to sleep as long as possible here - because it saves power
    // if (!runASAP && loopCanSleep()) {
    //     // if(delayMsec > 100) DEBUG_MSG("sleeping %ld\n", delayMsec);
    //     mainDelay.delay(delayMsec);
    // }
    // if (didWake) DEBUG_MSG("wake!\n");
}

int32_t loop_gene()
{
    // DEBUG_MSG("\n\nGENE GENE GENE\n\n");
    // Some example procedures showing how to display to the pixels:
    // colorWipe(strip.Color(255, 0, 0), 50); // Red
    // DEBUG_MSG("\n\n1GENE GENE GENE\n\n");
    // colorWipe(strip.Color(0, 255, 0), 50); // Green
    // DEBUG_MSG("\n\n3GENE GENE GENE\n\n");
    //   colorWipe(strip.Color(0, 0, 255), 50); // Blue
    // //colorWipe(strip.Color(0, 0, 0, 255), 50); // White RGBW
    //   // Send a theater pixel chase in...
    //   theaterChase(strip.Color(127, 127, 127), 50); // White
    //   theaterChase(strip.Color(127, 0, 0), 50); // Red
    //   theaterChase(strip.Color(0, 0, 127), 50); // Blue
    // theaterChaseRainbow(20);
    // return 1000;
    // rainbowCycle(50);
    // return 1000;
    
    //   rainbowCycle(20);
    //   theaterChaseRainbow(50);
    return rainbow_gene(20); 
    gPatterns[gCurrentPatternNumber]();

    // send the 'leds' array out to the actual LED strip
    FastLED.show();  
    // insert a delay to keep the framerate modest
    // FastLED.delay(1000/FRAMES_PER_SECOND); 

    // do some periodic updates
    EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
    EVERY_N_SECONDS( 10 ) { nextPattern(); } 
    
    // return rainbow_gene(20);// change patterns periodically
    return 20;
}

uint16_t counter = 0;
float speed = 3.0;
uint16_t j = 0;
int32_t rainbow_gene(uint8_t wait)
{
    uint16_t i;
    j=j+int(speed);
    counter++;
    if(counter == 55){ // 55 is fast 
      // speed = 50;
      counter=0;
    }
    speed = speed - (speed-3)/20; // 20 is fast
    // DEBUG_MSG("j= %i\n", j);
    // DEBUG_MSG("getCpuFrequencyMhz()= %i\n", getCpuFrequencyMhz());
    
    if (j >= 255) {
        j = 0;
    }
    // if(frame == 255) frame = 0;
    // for(j=0; j<256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
        // strip.setPixelColor(i, Wheel((i + j) & 255));
        strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    // delay(1000);
    // }
    return 20;
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait)
{
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, c);
        strip.show();
        delay(wait);
    }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
    uint16_t i, j;

    for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
        for (i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }
        strip.show();
        delay(wait);
    }
}

// Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait)
{
    for (int j = 0; j < 10; j++) { // do 10 cycles of chasing
        for (int q = 0; q < 3; q++) {
            for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
                strip.setPixelColor(i + q, c); // turn every third pixel on
            }
            strip.show();

            delay(wait);

            for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
                strip.setPixelColor(i + q, 0); // turn every third pixel off
            }
        }
    }
}

// Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait)
{
    for (int j = 0; j < 256; j++) { // cycle all 256 colors in the wheel
        for (int q = 0; q < 3; q++) {
            for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
                strip.setPixelColor(i + q, Wheel((i + j) % 255)); // turn every third pixel on
            }
            strip.show();

            delay(wait);

            for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
                strip.setPixelColor(i + q, 0); // turn every third pixel off
            }
        }
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170) {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


// List of patterns to cycle through.  Each is defined as a separate function below.
  
// void loop()
// {
//   // Call the current pattern function once, updating the 'leds' array

// }

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( int chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}