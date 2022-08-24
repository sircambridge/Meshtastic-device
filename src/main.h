#pragma once

#include <map>
#include "GPSStatus.h"
#include "NodeStatus.h"
#include "PowerStatus.h"
#include "graphics/Screen.h"
#include "mesh/generated/telemetry.pb.h"

extern uint8_t screen_found;
extern uint8_t screen_model;
extern uint8_t cardkb_found;
extern uint8_t kb_model;
extern uint8_t faceskb_found;
extern uint8_t rtc_found;

extern bool eink_found;
extern bool axp192_found;
extern bool isCharging;
extern bool isUSBPowered;

extern uint8_t nodeTelemetrySensorsMap[7];

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

int32_t loop_gene();
void colorWipe(uint32_t c, uint8_t wait);
int32_t rainbow_gene(uint8_t wait);
void rainbowCycle(uint8_t wait), theaterChase(uint32_t c, uint8_t wait), theaterChaseRainbow(uint8_t wait);

uint32_t Wheel(byte WheelPos);

// void drawTwinkles( CRGBSet& L);
// CRGB computeOneTwinkle( uint32_t ms, uint8_t salt);
// uint8_t attackDecayWave8( uint8_t i);
// void coolLikeIncandescent( CRGB& c, uint8_t phase);

void rainbow() ,juggle(),bpm(),sinelon(),confetti(),rainbowWithGlitter(),nextPattern();
void addGlitter( int32_t chanceOfGlitter);