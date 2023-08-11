#pragma once

#include "GPSStatus.h"
#include "NodeStatus.h"
#include "PowerStatus.h"
#include "detect/ScanI2C.h"
#include "graphics/Screen.h"
#include "memGet.h"
#include "mesh/generated/meshtastic/config.pb.h"
#include "mesh/generated/meshtastic/telemetry.pb.h"
#include <SPI.h>
#include <map>
#if !defined(ARCH_PORTDUINO) && !defined(ARCH_STM32WL)
#include <SparkFun_ATECCX08a_Arduino_Library.h>
#endif
#if defined(ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2)
#include "nimble/NimbleBluetooth.h"
extern NimbleBluetooth *nimbleBluetooth;
#endif
#ifdef ARCH_NRF52
#include "NRF52Bluetooth.h"
extern NRF52Bluetooth *nrf52Bluetooth;
#endif

extern ScanI2C::DeviceAddress screen_found;
extern ScanI2C::DeviceAddress cardkb_found;
extern uint8_t kb_model;
extern ScanI2C::DeviceAddress rtc_found;
extern ScanI2C::DeviceAddress accelerometer_found;
extern ScanI2C::FoundDevice rgb_found;

extern bool eink_found;
extern bool pmu_found;
extern bool isCharging;
extern bool isUSBPowered;

#if !defined(ARCH_PORTDUINO) && !defined(ARCH_STM32WL)
extern ATECCX08A atecc;
#endif

extern int TCPPort; // set by Portduino

// Global Screen singleton.
extern graphics::Screen *screen;
// extern Observable<meshtastic::PowerStatus> newPowerStatus; //TODO: move this to main-esp32.cpp somehow or a helper class

// extern meshtastic::PowerStatus *powerStatus;
// extern meshtastic::GPSStatus *gpsStatus;
// extern meshtastic::NodeStatusHandler *nodeStatusHandler;

// Return a human readable string of the form "Meshtastic_ab13"
const char *getDeviceName();

extern uint32_t timeLastPowered;

extern uint32_t rebootAtMsec;
extern uint32_t shutdownAtMsec;

extern uint32_t serialSinceMsec;

// If a thread does something that might need for it to be rescheduled ASAP it can set this flag
// This will supress the current delay and instead try to run ASAP.
extern bool runASAP;

void nrf52Setup(), esp32Setup(), nrf52Loop(), esp32Loop(), clearBonds();

meshtastic_DeviceMetadata getDeviceMetadata();

// FIXME, we default to 4MHz SPI, SPI mode 0, check if the datasheet says it can really do that
extern SPISettings spiSettings;

#define GENE_ENABLED 1

#ifdef GENE_ENABLED
int32_t get_current_pattern();
int32_t loop_gene();
void colorWipe(uint32_t c, uint8_t wait);
int32_t rainbow_gene(uint8_t wait);
void rainbowCycle(uint8_t wait), theaterChase(uint32_t c, uint8_t wait), theaterChaseRainbow(uint8_t wait);
uint32_t Wheel(byte WheelPos);
// void drawTwinkles( CRGBSet& L);
// CRGB computeOneTwinkle( uint32_t ms, uint8_t salt);
// uint8_t attackDecayWave8( uint8_t i);
// void coolLikeIncandescent( CRGB& c, uint8_t phase);
void rainbow(), juggle(), bpm(), sinelon(), confetti(), rainbowWithGlitter(), nextPattern();
void addGlitter(int32_t chanceOfGlitter);
int32_t gene_press();
void gene_long_press();
void Fire2012();
double distanceBetween(int32_t my_lat_i, int32_t my_lon_i, int32_t this_lat_i, int32_t this_lon_i);
// CRGB CoolColor( uint8_t temperature);
#endif