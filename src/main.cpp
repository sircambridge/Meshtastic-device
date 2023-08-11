#include "GPS.h"
#include "MeshRadio.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "ReliableRouter.h"
#include "airtime.h"
#include "buzz.h"
#include "configuration.h"
#include "error.h"
#include "power.h"
// #include "debug.h"
#include "FSCommon.h"
#include "RTC.h"
#include "SPILock.h"
#include "concurrency/OSThread.h"
#include "concurrency/Periodic.h"
#include "detect/ScanI2C.h"
#include "detect/ScanI2CTwoWire.h"
#include "detect/axpDebug.h"
#include "detect/einkScan.h"
#include "graphics/Screen.h"
#include "main.h"
#include "mesh/generated/meshtastic/config.pb.h"
#include "modules/Modules.h"
#include "shutdown.h"
#include "sleep.h"
#include "target_specific.h"
#include <Wire.h>
#include <memory>
// #include <driver/rtc_io.h>

// #include "mesh/eth/ethClient.h"
// #include "mesh/http/WiFiAPClient.h"

#define GENE_ENABLED 1
#define CONFIG_ESP_SYSTEM_ESP32_SRAM1_REGION_AS_IRAM 1

#ifdef GENE_ENABLED
#include <FastLED.h>
#include <Adafruit_NeoPixel.h>
#endif
#ifdef ARCH_ESP32
// #include "mesh/http/WebServer.h"
#include "nimble/NimbleBluetooth.h"
NimbleBluetooth *nimbleBluetooth;
#endif

#ifdef ARCH_NRF52
#include "NRF52Bluetooth.h"
NRF52Bluetooth *nrf52Bluetooth;
#endif

#if HAS_WIFI
#include "mesh/api/WiFiServerAPI.h"
#include "mqtt/MQTT.h"
#endif

#if HAS_ETHERNET
#include "mesh/api/ethServerAPI.h"
#include "mqtt/MQTT.h"
#endif

#include "LLCC68Interface.h"
#include "RF95Interface.h"
#include "SX1262Interface.h"
#include "SX1268Interface.h"
#include "SX1280Interface.h"
#ifdef ARCH_STM32WL
#include "STM32WLE5JCInterface.h"
#endif
#if !HAS_RADIO && defined(ARCH_PORTDUINO)
#include "platform/portduino/SimRadio.h"
#endif

#if HAS_BUTTON
#include "ButtonThread.h"
#endif
#include "PowerFSMThread.h"

#if !defined(ARCH_PORTDUINO) && !defined(ARCH_STM32WL)
#include "AccelerometerThread.h"
#endif

using namespace concurrency;

// We always create a screen object, but we only init it if we find the hardware
graphics::Screen *screen;

// Global power status
meshtastic::PowerStatus *powerStatus = new meshtastic::PowerStatus();

// Global GPS status
meshtastic::GPSStatus *gpsStatus = new meshtastic::GPSStatus();

// Global Node status
meshtastic::NodeStatus *nodeStatus = new meshtastic::NodeStatus();
#ifdef GENE_ENABLED
#define PIN 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(120, PIN, NEO_RGB + NEO_KHZ800);
#endif
// Scan for I2C Devices

/// The I2C address of our display (if found)
ScanI2C::DeviceAddress screen_found = ScanI2C::ADDRESS_NONE;

// The I2C address of the cardkb or RAK14004 (if found)
ScanI2C::DeviceAddress cardkb_found = ScanI2C::ADDRESS_NONE;
// 0x02 for RAK14004 and 0x00 for cardkb
uint8_t kb_model;

// The I2C address of the RTC Module (if found)
ScanI2C::DeviceAddress rtc_found = ScanI2C::ADDRESS_NONE;
// The I2C address of the Accelerometer (if found)
ScanI2C::DeviceAddress accelerometer_found = ScanI2C::ADDRESS_NONE;
// The I2C address of the RGB LED (if found)
ScanI2C::FoundDevice rgb_found = ScanI2C::FoundDevice(ScanI2C::DeviceType::NONE, ScanI2C::ADDRESS_NONE);

#if !defined(ARCH_PORTDUINO) && !defined(ARCH_STM32WL)
ATECCX08A atecc;
#endif

bool eink_found = true;

uint32_t serialSinceMsec;

bool pmu_found;

// Array map of sensor types (as array index) and i2c address as value we'll find in the i2c scan
uint8_t nodeTelemetrySensorsMap[_meshtastic_TelemetrySensorType_MAX + 1] = {
    0}; // one is enough, missing elements will be initialized to 0 anyway.

Router *router = NULL; // Users of router don't care what sort of subclass implements that API

const char *getDeviceName()
{
    uint8_t dmac[6];

    getMacAddr(dmac);

    // Meshtastic_ab3c or Shortname_abcd
    static char name[20];
    snprintf(name, sizeof(name), "%02x%02x", dmac[4], dmac[5]);
    // if the shortname exists and is NOT the new default of ab3c, use it for BLE name.
    if ((owner.short_name != NULL) && (strcmp(owner.short_name, name) != 0))
    {
        snprintf(name, sizeof(name), "%s_%02x%02x", owner.short_name, dmac[4], dmac[5]);
    }
    else
    {
        snprintf(name, sizeof(name), "Meshtastic_%02x%02x", dmac[4], dmac[5]);
    }
    return name;
}

static int32_t ledBlinker()
{
#ifdef GENE_ENABLED
    return loop_gene();
#endif
    static bool ledOn;
    ledOn ^= 1;

    setLed(ledOn);

    // have a very sparse duty cycle of LED being on, unless charging, then blink 0.5Hz square wave rate to indicate that
    return powerStatus->getIsCharging() ? 1000 : (ledOn ? 1 : 1000);
}

uint32_t timeLastPowered = 0;

#if HAS_BUTTON
bool ButtonThread::shutdown_on_long_stop = false;
#endif

static Periodic *ledPeriodic;
static Periodic *gps_update_periodic;
static OSThread *powerFSMthread;
#if HAS_BUTTON
static OSThread *buttonThread;
uint32_t ButtonThread::longPressTime = 0;
#endif
static OSThread *accelerometerThread;
SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);

RadioInterface *rIf = NULL;

/**
 * Some platforms (nrf52) might provide an alterate version that supresses calling delay from sleep.
 */
__attribute__((weak, noinline)) bool loopCanSleep()
{
    return true;
}

#ifdef GENE_ENABLED
#define NUM_LEDS 50
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
#define DATA_PIN 4
// #define CLK_PIN       4
#define VOLTS 5
#define MAX_MA 300
CRGB leds[NUM_LEDS];
#define BRIGHTNESS 200
#define FRAMES_PER_SECOND 120
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {rainbow, rainbowWithGlitter, confetti, sinelon, juggle};
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0;
#endif
void setup()
{
    concurrency::hasBeenSetup = true;
    meshtastic_Config_DisplayConfig_OledType screen_model =
        meshtastic_Config_DisplayConfig_OledType::meshtastic_Config_DisplayConfig_OledType_OLED_AUTO;
    OLEDDISPLAY_GEOMETRY screen_geometry = GEOMETRY_128_64;

#ifdef GENE_ENABLED
    FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_MA);
    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
        .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
#endif
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
    consoleInit(); // Set serial baud rate and init our mesh console
#endif

    serialSinceMsec = millis();

    LOG_INFO("\n\n//\\ E S H T /\\ S T / C\n\n");

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

#ifdef BUTTON_PIN
#ifdef ARCH_ESP32

    // If the button is connected to GPIO 12, don't enable the ability to use
    // meshtasticAdmin on the device.
    pinMode(config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN, INPUT);

#ifdef BUTTON_NEED_PULLUP
    gpio_pullup_en((gpio_num_t)(config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN));
    delay(10);
#endif

#endif
#endif

    OSThread::setup();

    ledPeriodic = new Periodic("Blink", ledBlinker);
    // gps_update_periodic = new Periodic("GPS", gene_press);

    fsInit();

#ifdef I2C_SDA1
    Wire1.begin(I2C_SDA1, I2C_SCL1);
#endif

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

#ifdef RAK4630
    // We need to enable 3.3V periphery in order to scan it
    pinMode(PIN_3V3_EN, OUTPUT);
    digitalWrite(PIN_3V3_EN, HIGH);

#ifndef USE_EINK
    // RAK-12039 set pin for Air quality sensor
    pinMode(AQ_SET_PIN, OUTPUT);
    digitalWrite(AQ_SET_PIN, HIGH);
#endif
#endif

    // Currently only the tbeam has a PMU
    // PMU initialization needs to be placed before i2c scanning
    power = new Power();
    power->setStatusHandler(powerStatus);
    powerStatus->observe(&power->newStatus);
    power->setup(); // Must be after status handler is installed, so that handler gets notified of the initial configuration

    // We need to scan here to decide if we have a screen for nodeDB.init() and because power has been applied to
    // accessories
    auto i2cScanner = std::unique_ptr<ScanI2CTwoWire>(new ScanI2CTwoWire());

    LOG_INFO("Scanning for i2c devices...\n");

#ifdef I2C_SDA1
    Wire1.begin(I2C_SDA1, I2C_SCL1);
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE1);
#endif

#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE);
#elif HAS_WIRE
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE);
#endif

    auto i2cCount = i2cScanner->countDevices();
    if (i2cCount == 0)
    {
        LOG_INFO("No I2C devices found\n");
    }
    else
    {
        LOG_INFO("%i I2C devices found\n", i2cCount);
    }

#ifdef ARCH_ESP32
    // Don't init display if we don't have one or we are waking headless due to a timer event
    if (wakeCause == ESP_SLEEP_WAKEUP_TIMER)
    {
        LOG_DEBUG("suppress screen wake because this is a headless timer wakeup");
        i2cScanner->setSuppressScreen();
    }
#endif

    auto screenInfo = i2cScanner->firstScreen();
    screen_found = screenInfo.type != ScanI2C::DeviceType::NONE ? screenInfo.address : ScanI2C::ADDRESS_NONE;

    if (screen_found.port != ScanI2C::I2CPort::NO_I2C)
    {
        switch (screenInfo.type)
        {
        case ScanI2C::DeviceType::SCREEN_SH1106:
            screen_model = meshtastic_Config_DisplayConfig_OledType::meshtastic_Config_DisplayConfig_OledType_OLED_SH1106;
            break;
        case ScanI2C::DeviceType::SCREEN_SSD1306:
            screen_model = meshtastic_Config_DisplayConfig_OledType::meshtastic_Config_DisplayConfig_OledType_OLED_SSD1306;
            break;
        case ScanI2C::DeviceType::SCREEN_ST7567:
        case ScanI2C::DeviceType::SCREEN_UNKNOWN:
        default:
            screen_model = meshtastic_Config_DisplayConfig_OledType::meshtastic_Config_DisplayConfig_OledType_OLED_AUTO;
        }
    }

#define UPDATE_FROM_SCANNER(FIND_FN)

    auto rtc_info = i2cScanner->firstRTC();
    rtc_found = rtc_info.type != ScanI2C::DeviceType::NONE ? rtc_info.address : rtc_found;

    auto kb_info = i2cScanner->firstKeyboard();

    if (kb_info.type != ScanI2C::DeviceType::NONE)
    {
        cardkb_found = kb_info.address;
        switch (kb_info.type)
        {
        case ScanI2C::DeviceType::RAK14004:
            kb_model = 0x02;
            break;
        case ScanI2C::DeviceType::CARDKB:
        default:
            // use this as default since it's also just zero
            kb_model = 0x00;
        }
    }

    pmu_found = i2cScanner->exists(ScanI2C::DeviceType::PMU_AXP192_AXP2101);

    /*
     * There are a bunch of sensors that have no further logic than to be found and stuffed into the
     * nodeTelemetrySensorsMap singleton. This wraps that logic in a temporary scope to declare the temporary field
     * "found".
     */

    // Only one supported RGB LED currently
    rgb_found = i2cScanner->find(ScanI2C::DeviceType::NCP5623);

#if !defined(ARCH_PORTDUINO) && !defined(ARCH_STM32WL)
    auto acc_info = i2cScanner->firstAccelerometer();
    accelerometer_found = acc_info.type != ScanI2C::DeviceType::NONE ? acc_info.address : accelerometer_found;
    LOG_DEBUG("acc_info = %i\n", acc_info.type);
#endif

#define STRING(S) #S

#define SCANNER_TO_SENSORS_MAP(SCANNER_T, PB_T)                    \
    {                                                              \
        auto found = i2cScanner->find(SCANNER_T);                  \
        if (found.type != ScanI2C::DeviceType::NONE)               \
        {                                                          \
            nodeTelemetrySensorsMap[PB_T] = found.address.address; \
            LOG_DEBUG("found i2c sensor %s\n", STRING(PB_T));      \
        }                                                          \
    }

    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::BME_680, meshtastic_TelemetrySensorType_BME680)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::BME_280, meshtastic_TelemetrySensorType_BME280)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::BMP_280, meshtastic_TelemetrySensorType_BMP280)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::INA260, meshtastic_TelemetrySensorType_INA260)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::INA219, meshtastic_TelemetrySensorType_INA219)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::MCP9808, meshtastic_TelemetrySensorType_MCP9808)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::MCP9808, meshtastic_TelemetrySensorType_MCP9808)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::SHT31, meshtastic_TelemetrySensorType_SHT31)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::SHTC3, meshtastic_TelemetrySensorType_SHTC3)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::LPS22HB, meshtastic_TelemetrySensorType_LPS22)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::QMC6310, meshtastic_TelemetrySensorType_QMC6310)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::QMI8658, meshtastic_TelemetrySensorType_QMI8658)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::QMC5883L, meshtastic_TelemetrySensorType_QMC5883L)
    SCANNER_TO_SENSORS_MAP(ScanI2C::DeviceType::PMSA0031, meshtastic_TelemetrySensorType_PMSA003I)

    i2cScanner.reset();

#ifdef HAS_SDCARD
    setupSDCard();
#endif

#ifdef RAK4630
    // scanEInkDevice();
#endif

    // LED init

#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, 1 ^ LED_INVERTED); // turn on for now
#endif

    // Hello
    LOG_INFO("Meshtastic hwvendor=%d, swver=%s\n", HW_VENDOR, optstr(APP_VERSION));

#ifdef ARCH_ESP32
    esp32Setup();
#endif

#ifdef ARCH_NRF52
    nrf52Setup();
#endif
    // We do this as early as possible because this loads preferences from flash
    // but we need to do this after main cpu iniot (esp32setup), because we need the random seed set
    nodeDB.init();

    // If we're taking on the repeater role, use flood router
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_REPEATER)
        router = new FloodingRouter();
    else
        router = new ReliableRouter();

#if HAS_BUTTON
    // Buttons. Moved here cause we need NodeDB to be initialized
    buttonThread = new ButtonThread();
#endif

    playStartMelody();

    // fixed screen override?
    if (config.display.oled != meshtastic_Config_DisplayConfig_OledType_OLED_AUTO)
        screen_model = config.display.oled;

#if defined(USE_SH1107)
    screen_model = meshtastic_Config_DisplayConfig_OledType_OLED_SH1107; // set dimension of 128x128
    display_geometry = GEOMETRY_128_128;
#endif

#if defined(USE_SH1107_128_64)
    screen_model = meshtastic_Config_DisplayConfig_OledType_OLED_SH1107; // keep dimension of 128x64
#endif

#if !defined(ARCH_PORTDUINO) && !defined(ARCH_STM32WL)
    if (acc_info.type != ScanI2C::DeviceType::NONE)
    {
        accelerometerThread = new AccelerometerThread(acc_info.type);
    }
#endif

    // Init our SPI controller (must be before screen and lora)
    initSPI();
#ifdef ARCH_RP2040
#ifdef HW_SPI1_DEVICE
    SPI1.setSCK(RF95_SCK);
    SPI1.setTX(RF95_MOSI);
    SPI1.setRX(RF95_MISO);
    pinMode(RF95_NSS, OUTPUT);
    digitalWrite(RF95_NSS, HIGH);
    SPI1.begin(false);
#else                      // HW_SPI1_DEVICE
    SPI.setSCK(RF95_SCK);
    SPI.setTX(RF95_MOSI);
    SPI.setRX(RF95_MISO);
    SPI.begin(false);
#endif                     // HW_SPI1_DEVICE
#elif !defined(ARCH_ESP32) // ARCH_RP2040
    SPI.begin();
#else
    // ESP32
    SPI.begin(RF95_SCK, RF95_MISO, RF95_MOSI, RF95_NSS);
    SPI.setFrequency(4000000);
#endif

    // Initialize the screen first so we can show the logo while we start up everything else.
    screen = new graphics::Screen(screen_found, screen_model, screen_geometry);

    readFromRTC(); // read the main CPU RTC at first (in case we can't get GPS time)

    gps = createGps();

    if (gps)
    {
        gpsStatus->observe(&gps->newStatus);
    }
    else
    {
        LOG_WARN("No GPS found - running without GPS\n");
    }

    nodeStatus->observe(&nodeDB.newStatus);

    service.init();

    // Now that the mesh service is created, create any modules
    setupModules();

// Do this after service.init (because that clears error_code)
#ifdef HAS_PMU
    if (!pmu_found)
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_NO_AXP192); // Record a hardware fault for missing hardware
#endif

// Don't call screen setup until after nodedb is setup (because we need
// the current region name)
#if defined(ST7735_CS) || defined(USE_EINK) || defined(ILI9341_DRIVER)
    screen->setup();
#else
    if (screen_found.port != ScanI2C::I2CPort::NO_I2C)
        screen->setup();
#endif

    screen->print("Started...\n");

    // We have now loaded our saved preferences from flash

    // ONCE we will factory reset the GPS for bug #327
    if (gps && !devicestate.did_gps_reset)
    {
        LOG_WARN("GPS FactoryReset requested\n");
        if (gps->factoryReset())
        { // If we don't succeed try again next time
            devicestate.did_gps_reset = true;
            nodeDB.saveToDisk(SEGMENT_DEVICESTATE);
        }
    }

#ifdef SX126X_ANT_SW
    // make analog PA vs not PA switch on SX126x eval board work properly
    pinMode(SX126X_ANT_SW, OUTPUT);
    digitalWrite(SX126X_ANT_SW, 1);
#endif

#ifdef HW_SPI1_DEVICE
    LockingArduinoHal *RadioLibHAL = new LockingArduinoHal(SPI1, spiSettings);
#else // HW_SPI1_DEVICE
    LockingArduinoHal *RadioLibHAL = new LockingArduinoHal(SPI, spiSettings);
#endif

    // radio init MUST BE AFTER service.init, so we have our radio config settings (from nodedb init)
#if defined(USE_STM32WLx)
    if (!rIf)
    {
        rIf = new STM32WLE5JCInterface(RadioLibHAL, SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
        if (!rIf->init())
        {
            LOG_WARN("Failed to find STM32WL radio\n");
            delete rIf;
            rIf = NULL;
        }
        else
        {
            LOG_INFO("STM32WL Radio init succeeded, using STM32WL radio\n");
        }
    }
#endif

#if !HAS_RADIO && defined(ARCH_PORTDUINO)
    if (!rIf)
    {
        rIf = new SimRadio;
        if (!rIf->init())
        {
            LOG_WARN("Failed to find simulated radio\n");
            delete rIf;
            rIf = NULL;
        }
        else
        {
            LOG_INFO("Using SIMULATED radio!\n");
        }
    }
#endif

#if defined(RF95_IRQ)
    if (!rIf)
    {
        rIf = new RF95Interface(RadioLibHAL, RF95_NSS, RF95_IRQ, RF95_RESET, RF95_DIO1);
        if (!rIf->init())
        {
            LOG_WARN("Failed to find RF95 radio\n");
            delete rIf;
            rIf = NULL;
        }
        else
        {
            LOG_INFO("RF95 Radio init succeeded, using RF95 radio\n");
        }
    }
#endif

#if defined(USE_SX1262)
    if (!rIf)
    {
        rIf = new SX1262Interface(RadioLibHAL, SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
        if (!rIf->init())
        {
            LOG_WARN("Failed to find SX1262 radio\n");
            delete rIf;
            rIf = NULL;
        }
        else
        {
            LOG_INFO("SX1262 Radio init succeeded, using SX1262 radio\n");
        }
    }
#endif

#if defined(USE_SX1268)
    if (!rIf)
    {
        rIf = new SX1268Interface(RadioLibHAL, SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
        if (!rIf->init())
        {
            LOG_WARN("Failed to find SX1268 radio\n");
            delete rIf;
            rIf = NULL;
        }
        else
        {
            LOG_INFO("SX1268 Radio init succeeded, using SX1268 radio\n");
        }
    }
#endif

#if defined(USE_LLCC68)
    if (!rIf)
    {
        rIf = new LLCC68Interface(RadioLibHAL, SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
        if (!rIf->init())
        {
            LOG_WARN("Failed to find LLCC68 radio\n");
            delete rIf;
            rIf = NULL;
        }
        else
        {
            LOG_INFO("LLCC68 Radio init succeeded, using LLCC68 radio\n");
        }
    }
#endif

#if defined(USE_SX1280)
    if (!rIf)
    {
        rIf = new SX1280Interface(RadioLibHAL, SX128X_CS, SX128X_DIO1, SX128X_RESET, SX128X_BUSY);
        if (!rIf->init())
        {
            LOG_WARN("Failed to find SX1280 radio\n");
            delete rIf;
            rIf = NULL;
        }
        else
        {
            LOG_INFO("SX1280 Radio init succeeded, using SX1280 radio\n");
        }
    }
#endif

    // check if the radio chip matches the selected region

    if ((config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_LORA_24) && (!rIf->wideLora()))
    {
        LOG_WARN("Radio chip does not support 2.4GHz LoRa. Reverting to unset.\n");
        config.lora.region = meshtastic_Config_LoRaConfig_RegionCode_UNSET;
        nodeDB.saveToDisk(SEGMENT_CONFIG);
        if (!rIf->reconfigure())
        {
            LOG_WARN("Reconfigure failed, rebooting\n");
            screen->startRebootScreen();
            rebootAtMsec = millis() + 5000;
        }
    }

#if HAS_WIFI || HAS_ETHERNET
    mqttInit();
#endif

#ifndef ARCH_PORTDUINO
    // Initialize Wifi
    // initWifi();

    // Initialize Ethernet
    // initEthernet();
#endif

#ifdef ARCH_ESP32
    // Start web server thread.
    // webServerThread = new WebServerThread();
#endif

#ifdef ARCH_PORTDUINO
    initApiServer(TCPPort);
#endif

    // Start airtime logger thread.
    airTime = new AirTime();

    if (!rIf)
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_NO_RADIO);
    else
    {
        router->addInterface(rIf);

        // Log bit rate to debug output
        LOG_DEBUG("LoRA bitrate = %f bytes / sec\n", (float(meshtastic_Constants_DATA_PAYLOAD_LEN) /
                                                      (float(rIf->getPacketTime(meshtastic_Constants_DATA_PAYLOAD_LEN)))) *
                                                         1000);
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

extern meshtastic_DeviceMetadata getDeviceMetadata()
{
    meshtastic_DeviceMetadata deviceMetadata;
    strncpy(deviceMetadata.firmware_version, optstr(APP_VERSION), sizeof(deviceMetadata.firmware_version));
    deviceMetadata.device_state_version = DEVICESTATE_CUR_VER;
    deviceMetadata.canShutdown = pmu_found || HAS_CPU_SHUTDOWN;
    deviceMetadata.hasBluetooth = HAS_BLUETOOTH;
    deviceMetadata.hasWifi = HAS_WIFI;
    deviceMetadata.hasEthernet = HAS_ETHERNET;
    deviceMetadata.role = config.device.role;
    deviceMetadata.position_flags = config.position.position_flags;
    deviceMetadata.hw_model = HW_VENDOR;
    deviceMetadata.hasRemoteHardware = moduleConfig.remote_hardware.enabled;
    return deviceMetadata;
}

void loop()
{
    runASAP = false;

    // axpDebugOutput.loop();

    // heap_caps_check_integrity_all(true); // FIXME - disable this expensive check

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
    if (millis() - lastPrint > 10 * 1000L)
    {
        lastPrint = millis();
        meshtastic::printThreadInfo("main");
    }
#endif

    // TODO: This should go into a thread handled by FreeRTOS.
    // handleWebResponse();

    service.loop();

    long delayMsec = mainController.runOrDelay();

    // /* if (mainController.nextThread && delayMsec)
    //     LOG_DEBUG("Next %s in %ld\n", mainController.nextThread->ThreadName.c_str(),
    //               mainController.nextThread->tillRun(millis())); */

    // // We want to sleep as long as possible here - because it saves power
    if (!runASAP && loopCanSleep())
    {
        // if(delayMsec > 100) LOG_DEBUG("sleeping %ld\n", delayMsec);
        mainDelay.delay(delayMsec);
    }
    // if (didWake) LOG_DEBUG("wake!\n");
}
#ifdef GENE_ENABLED

int32_t current_pattern = 0;
int32_t get_current_pattern()
{
    return current_pattern;
}
int the_intensity_const = 3;
int32_t gene_press()
{
    // current_pattern++;
    // service.onGPSChanged(gpsStatus);
    LOG_DEBUG("GENE press!\n");
    int32_t time_ms = getTime_ms();
    time_ms = abs(time_ms);
    LOG_DEBUG("[GENE] getTime_ms = %i\n", time_ms);
    // nextPattern();
    LOG_DEBUG("numNodes = %d", nodeDB.getNumNodes());
    char coordinateLine[100];
    // NodeInfo *me = nodeDB.getNodeByIndex(nodeDB.getNodeNum());
    int nodeIndex = (nodeIndex + 1) % nodeDB.getNumNodes();
    LOG_DEBUG("[GENE] nodeIndex = %i", nodeIndex);
    meshtastic_NodeInfo *node = nodeDB.getNodeByIndex(nodeIndex);
    const char *username = node->has_user ? node->user.long_name : "Unknown Name";
    LOG_DEBUG("[GENE]  %s\n", username);
    LOG_DEBUG("[GENE]  getTime_ms() %i\n", getTime_ms());
    LOG_DEBUG("[GENE] mylat = %i lon = %i \n", int32_t(gpsStatus->getLatitude()), int32_t(gpsStatus->getLongitude()));
    the_intensity_const = 1;
    int within50_feet = 0;
    int within100_feet = 0;
    for (size_t i = 0; i < nodeDB.getNumNodes(); i++)
    {
        meshtastic_NodeInfo *n = nodeDB.getNodeByIndex(i);
        const char *usernamee = n->has_user ? n->user.long_name : "Unknown Name";
        meshtastic_Position &p = n->position;
        double distance = distanceBetween(gpsStatus->getLatitude(), gpsStatus->getLongitude(), p.latitude_i, p.longitude_i);
        // LOG_DEBUG("[GENE] distance = %f\n", distance);
        LOG_DEBUG("[GENE] %s node %i lat = %i lon = %i distance = %f \n", usernamee, i, p.latitude_i, p.longitude_i, distance);
        if (distance == 0.0)
            continue;
        if (distance < 50)
            within50_feet++;
        if (distance < 100)
            within100_feet++;
    }
    if (within100_feet > 2)
        the_intensity_const = 2;
    if (within100_feet > 5)
        the_intensity_const = 3;
    if (within50_feet > 15)
        the_intensity_const = 4;
    LOG_DEBUG("[GENE] within100_feet = %i\n", within100_feet);
    LOG_DEBUG("[GENE] within50_feet = %i\n", within50_feet);
    LOG_DEBUG("[GENE] the_intensity_const = %i\n", the_intensity_const);
    return 10 * 1000;
}
// double distanceBetween(double lat1, double long1, double lat2, double long2)
double distanceBetween(int32_t my_lat_i, int32_t my_lon_i, int32_t this_lat_i, int32_t this_lon_i)
{
    double lat1 = double(my_lat_i / 10000000.0);
    double long1 = double(my_lon_i / 10000000.0);
    double lat2 = double(this_lat_i) / 10000000.0;
    double long2 = double(this_lon_i) / 10000000.0;
    // LOG_DEBUG("[GENE] lat1 = %f long1 = %f lat2 = %f long2 = %f\n", lat1, long1, lat2, long2);
    // returns distance in meters between two positions, both specified
    // as signed decimal-degrees latitude and longitude. Uses great-circle
    // distance computation for hypothetical sphere of radius 6372795 meters.
    // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    // Courtesy of Maarten Lamers
    double delta = radians(long1 - long2);
    double sdlong = sin(delta);
    double cdlong = cos(delta);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double slat1 = sin(lat1);
    double clat1 = cos(lat1);
    double slat2 = sin(lat2);
    double clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = sq(delta);
    delta += sq(clat2 * sdlong);
    delta = sqrt(delta);
    double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * 6372795;
}
void gene_got_txt_msg(const char *tempBuf)
{
    LOG_DEBUG("[GENE] got txt tempBuf  %s");
}
void gene_long_press()
{
    LOG_DEBUG("[GENE] gene_long_press!\n");
}
int lastgCurrentPatternNumber = 0;
float counter = 0.0;
int current_led = 0;
float speed2 = 100;
int pos1 = 0;
int the_seed_color = 0;
int32_t loop_gene()
{
    // LOG_DEBUG("\n\nGENE GENE GENE\n\n");
    // Some example procedures showing how to display to the pixels:
    // colorWipe(strip.Color(255, 0, 0), 50); // Red
    // LOG_DEBUG("\n\n1GENE GENE GENE\n\n");
    // colorWipe(strip.Color(0, 255, 0), 50); // Green
    // LOG_DEBUG("\n\n3GENE GENE GENE\n\n");
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
    int32_t time_ms = getTime_ms();
    // LOG_DEBUG("[GENE] time_ms = %i\n", time_ms);
    // j = 255 * (-time_ms % 5000) / (5000);
    int total_patterns = 5;
    int how_many_seconds_per_pattern = 60;
    if (true)
    {
        gCurrentPatternNumber = total_patterns * (abs(time_ms) % (1000 * how_many_seconds_per_pattern * total_patterns)) /
                                (1000 * how_many_seconds_per_pattern * total_patterns);
        if (gCurrentPatternNumber != lastgCurrentPatternNumber)
        {
            // LOG_DEBUG("[GENE] gCurrentPatternNumber = %i\n", gCurrentPatternNumber);
            lastgCurrentPatternNumber = gCurrentPatternNumber;
        }
    }
    // gCurrentPatternNumber = 5;
    // gCurrentPatternNumber = 4;
    // LOG_DEBUG("[GENE] gCurrentPatternNumber = %i\n", gCurrentPatternNumber);
    if (gCurrentPatternNumber == 5)
    {
        FastLED.setBrightness(100);
        // current_pixel++;
        counter += speed2;
        the_intensity_const = 3;
        current_led = counter / 100;
        // gHue = 255 * (-getTime_ms() % (10000)) / (10000);
        if (counter == 1)
        {
            speed2 = 100;
        }
        if (current_led <= 60)
        {
            leds[current_led] = CHSV(gHue + random8(64), 200, 255);
            fadeToBlackBy(leds, NUM_LEDS, 10 * the_intensity_const);
            if (speed2 > 10)
            {
                speed2 -= 0.5;
            }
        }
        else if (current_led > 60 && current_led < 80)
        {
            FastLED.setBrightness(200);
            for (size_t i = 0; i < the_intensity_const; i++)
            {
                int pos = 60 + random16(60);
                CRGB tmp = CHSV(gHue + random8(64), 200, 255);
                leds[pos] += CRGB(tmp.g, tmp.r, tmp.b);
            }
            fadeToBlackBy(leds, NUM_LEDS, 10);
        }
        else if (current_led > 80 && current_led < 120)
        {
            FastLED.setBrightness(200);
            fadeToBlackBy(leds, NUM_LEDS, 10);
        }
        else if (current_led > 120)
        {
            counter = 0;
            speed2 = 100;
            gHue = 255 * (getTime_ms() % (10000)) / (10000);
            LOG_DEBUG("[GENE] gHue = %i\n", gHue);
        }
    }
    if (gCurrentPatternNumber == 0)
    {
        FastLED.setBrightness(30);
        gHue = 255 * (-getTime_ms() % (1000 / the_intensity_const)) / (1000 / the_intensity_const);
        fill_rainbow(leds, NUM_LEDS, gHue, 7);
    }
    if (gCurrentPatternNumber == 1)
    {
        the_intensity_const = 3;
        gHue = 255 * (-getTime_ms() % (1000 / the_intensity_const)) / (1000 / the_intensity_const);
        FastLED.setBrightness(30);
        rainbow();
        // addGlitter(80);
    }
    // glitter
    if (gCurrentPatternNumber == 2)
    {
        FastLED.setBrightness(100);
        the_intensity_const = 3;
        gHue = 255 * (-getTime_ms() % (10000)) / (10000);
        fadeToBlackBy(leds, NUM_LEDS, 10 * the_intensity_const);
        for (size_t i = 0; i < the_intensity_const; i++)
        {
            int pos = random16(NUM_LEDS);
            leds[pos] += CHSV(gHue + random8(64), 200, 255);
        }
    }
    if (gCurrentPatternNumber == 3)
    {
        FastLED.setBrightness(50);
        // uint8_t BeatsPerMinute = 62;
        the_intensity_const = 1;
        uint8_t BeatsPerMinute = the_intensity_const * 30;
        int the_palette = 4 * (-getTime_ms() % (30000)) / (30000);
        CRGBPalette16 palettes[5] = {CloudColors_p, LavaColors_p, OceanColors_p, ForestColors_p};
        CRGBPalette16 palette = palettes[the_palette];
        uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
        for (int i = 0; i < NUM_LEDS; i++)
        { // 9948
            leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
        }
    }
    if (gCurrentPatternNumber == 4)
    {
        FastLED.setBrightness(50);
        Fire2012();
    }
    // if(gCurrentPatternNumber == 5){
    // }
    FastLED.show();
    // if (false) {
    //     rainbow_gene(20);
    // } else {
    //     gPatterns[gCurrentPatternNumber]();
    //     // send the 'leds' array out to the actual LED strip
    //     FastLED.show();
    //     // insert a delay to keep the framerate modest
    //     // FastLED.delay(1000/FRAMES_PER_SECOND);
    //     // do some periodic updates
    //     // int32_t time_ms = getTime_ms();
    //     the_intensity_const = 4;
    //     // 1 is low, 5 is high
    //     gHue = 255 * (-getTime_ms() % (1000/the_intensity_const)) / (1000/the_intensity_const);
    //     // EVERY_N_MILLISECONDS(20)
    //     // {
    //     //     gHue++;
    //     // } // slowly cycle the "base color" through the rainbow
    //     // EVERY_N_SECONDS(10)
    //     // {
    //     //     nextPattern();
    //     // }
    // }
    // return rainbow_gene(20);// change patterns periodically
    return 20;
}
// uint16_t counter = 0;
float speed = 2.0;
uint16_t j = 0;
int32_t rainbow_gene(uint8_t wait)
{
    uint16_t i;
    // j = j + int(speed);
    // counter++;
    // if (counter == 55) { // 55 is fast
    //     // speed = 50;
    //     counter = 0;
    // }
    int32_t time_ms = getTime_ms();
    // LOG_DEBUG("[GENE] time_ms = %i\n", time_ms);
    j = 255 * (-time_ms % 5000) / (5000);
    speed = speed - (speed - 3) / 20; // 20 is fast
    // LOG_DEBUG("j= %i\n", j);
    // speed = 0;
    // LOG_DEBUG("getCpuFrequencyMhz()= %i\n", getCpuFrequencyMhz());
    if (j >= 255)
    {
        j = 0;
    }
    // if(frame == 255) frame = 0;
    // for(j=0; j<256; j++) {
    for (i = 0; i < strip.numPixels(); i++)
    {
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
    for (uint16_t i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, c);
        strip.show();
        delay(wait);
    }
}
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
    uint16_t i, j;
    for (j = 0; j < 256 * 5; j++)
    { // 5 cycles of all colors on wheel
        for (i = 0; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }
        strip.show();
        delay(wait);
    }
}
// Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait)
{
    for (int j = 0; j < 10; j++)
    { // do 10 cycles of chasing
        for (int q = 0; q < 3; q++)
        {
            for (uint16_t i = 0; i < strip.numPixels(); i = i + 3)
            {
                strip.setPixelColor(i + q, c); // turn every third pixel on
            }
            strip.show();
            delay(wait);
            for (uint16_t i = 0; i < strip.numPixels(); i = i + 3)
            {
                strip.setPixelColor(i + q, 0); // turn every third pixel off
            }
        }
    }
}
// Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait)
{
    for (int j = 0; j < 256; j++)
    { // cycle all 256 colors in the wheel
        for (int q = 0; q < 3; q++)
        {
            for (uint16_t i = 0; i < strip.numPixels(); i = i + 3)
            {
                strip.setPixelColor(i + q, Wheel((i + j) % 255)); // turn every third pixel on
            }
            strip.show();
            delay(wait);
            for (uint16_t i = 0; i < strip.numPixels(); i = i + 3)
            {
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
    if (WheelPos < 85)
    {
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
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
    gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
}
void rainbow()
{
    // FastLED's built-in rainbow generator
    fill_rainbow(leds, NUM_LEDS, gHue, 7);
}
void rainbowWithGlitter()
{
    FastLED.setBrightness(30);
    the_intensity_const = 5;
    // built-in FastLED rainbow, plus some random sparkly glitter
    rainbow();
    addGlitter(80);
}
void addGlitter(int chanceOfGlitter)
{
    if (random8() < chanceOfGlitter)
    {
        leds[random16(NUM_LEDS)] += CRGB::White;
    }
}
void confetti()
{
    // random colored speckles that blink in and fade smoothly
    FastLED.setBrightness(50);
    fadeToBlackBy(leds, NUM_LEDS, 10);
    int pos = random16(NUM_LEDS);
    leds[pos] += CHSV(gHue + random8(64), 200, 255);
}
void sinelon()
{
    // a colored dot sweeping back and forth, with fading trails
    fadeToBlackBy(leds, NUM_LEDS, 20);
    int pos = beatsin16(13, 0, NUM_LEDS - 1);
    leds[pos] += CHSV(gHue, 255, 192);
}
void bpm()
{
    // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
    FastLED.setBrightness(50);
    uint8_t BeatsPerMinute = 62;
    CRGBPalette16 palette = PartyColors_p;
    uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
    for (int i = 0; i < NUM_LEDS; i++)
    { // 9948
        leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
    }
}
void juggle()
{
    // eight colored dots, weaving in and out of sync with each other
    FastLED.setBrightness(30);
    fadeToBlackBy(leds, NUM_LEDS, 20);
    uint8_t dothue = 0;
    for (int i = 0; i < 8; i++)
    {
        leds[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
        dothue += 32;
    }
}
#define COOLING 55
// #define SPARKING 20
bool gReverseDirection = false;
void Fire2012()
{
    // Array of temperature readings at each simulation cell
    static uint8_t heat[NUM_LEDS];
    // Step 1.  Cool down every cell a little
    for (int i = 0; i < NUM_LEDS; i++)
    {
        heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
    }
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for (int k = NUM_LEDS - 1; k >= 2; k--)
    {
        heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    int SPARKING = 20 + the_intensity_const * 30;
    if (random8() < SPARKING)
    {
        int y = random8(7);
        heat[y] = qadd8(heat[y], random8(160, 255));
    }
    // Step 4.  Map from heat cells to LED colors
    for (int j = 0; j < NUM_LEDS; j++)
    {
        CRGB color = HeatColor(heat[j]);
        int pixelnumber;
        if (gReverseDirection)
        {
            pixelnumber = (NUM_LEDS - 1) - j;
        }
        else
        {
            pixelnumber = j;
        }
        leds[pixelnumber] = color;
    }
}
#endif