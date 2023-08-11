#include "RTC.h"
#include "configuration.h"
#include "detect/ScanI2C.h"
#include "main.h"
#include <sys/time.h>
#include <time.h>

static RTCQuality currentQuality = RTCQualityNone;

RTCQuality getRTCQuality()
{
    return currentQuality;
}

// stuff that really should be in in the instance instead...
static uint64_t timeStartMsec;
static uint64_t timeStartUMsec; // Once we have a GPS lock, this is where we hold the initial msec clock that corresponds to that time
static uint64_t zeroOffsetSecs;
static uint64_t zeroOffsetUSecs;

void readFromRTC()
{
    struct timeval tv; /* btw settimeofday() is helpfull here too*/
#ifdef RV3028_RTC
    if (rtc_found.address == RV3028_RTC)
    {
        uint32_t now = millis();
        Melopero_RV3028 rtc;
#ifdef I2C_SDA1
        rtc.initI2C(rtc_found.port == ScanI2C::I2CPort::WIRE1 ? Wire1 : Wire);
#else
        rtc.initI2C();
#endif
        tm t;
        t.tm_year = rtc.getYear() - 1900;
        t.tm_mon = rtc.getMonth() - 1;
        t.tm_mday = rtc.getDate();
        t.tm_hour = rtc.getHour();
        t.tm_min = rtc.getMinute();
        t.tm_sec = rtc.getSecond();
        tv.tv_sec = mktime(&t);
        tv.tv_usec = 0;
        LOG_DEBUG("Read RTC time from RV3028 as %ld\n", tv.tv_sec);
        timeStartMsec = now;
        zeroOffsetSecs = tv.tv_sec;
        if (currentQuality == RTCQualityNone)
        {
            currentQuality = RTCQualityDevice;
        }
    }
#elif defined(PCF8563_RTC)
    if (rtc_found.address == PCF8563_RTC)
    {
        uint32_t now = millis();
        PCF8563_Class rtc;

#ifdef I2C_SDA1
        rtc.begin(rtc_found.port == ScanI2C::I2CPort::WIRE1 ? Wire1 : Wire);
#else
        rtc.begin();
#endif

        auto tc = rtc.getDateTime();
        tm t;
        t.tm_year = tc.year - 1900;
        t.tm_mon = tc.month - 1;
        t.tm_mday = tc.day;
        t.tm_hour = tc.hour;
        t.tm_min = tc.minute;
        t.tm_sec = tc.second;
        tv.tv_sec = mktime(&t);
        tv.tv_usec = 0;
        LOG_DEBUG("Read RTC time from PCF8563 as %ld\n", tv.tv_sec);
        timeStartMsec = now;
        zeroOffsetSecs = tv.tv_sec;
        if (currentQuality == RTCQualityNone)
        {
            currentQuality = RTCQualityDevice;
        }
    }
#else
    if (!gettimeofday(&tv, NULL))
    {
        uint32_t now = millis();
        LOG_DEBUG("Read RTC time as u %d\n", tv.tv_usec);
        timeStartMsec = now;
        timeStartUMsec = micros();
        zeroOffsetSecs = tv.tv_sec;
        zeroOffsetUSecs = tv.tv_usec;
    }
#endif
}

/// If we haven't yet set our RTC this boot, set it from a GPS derived time
bool perhapsSetRTC(RTCQuality q, const struct timeval *tv)
{
    return perhapsSetRTC(q, tv, 0);
}
bool perhapsSetRTC(RTCQuality q, struct tm &t)
{
    return perhapsSetRTC(q, t, 0);
}
/// If we haven't yet set our RTC this boot, set it from a GPS derived time
bool perhapsSetRTC(RTCQuality q, const struct timeval *tv, int tm_csec)
{
    static uint32_t lastSetMsec = 0;
    uint32_t now = millis();

    bool shouldSet;
    if (q > currentQuality)
    {
        currentQuality = q;
        shouldSet = true;
        LOG_DEBUG("Upgrading time to RTC %ld secs (quality %d)\n", tv->tv_sec, q);
        LOG_DEBUG("Upgrading time to RTC %d secs (quality %d)\n", tv->tv_usec, q);
        LOG_DEBUG("Upgrading time to RTC %d tm_csec (quality %d)\n", tm_csec, q);
    }
    else if (q == RTCQualityGPS && (now - lastSetMsec) > (12 * 60 * 60 * 1000UL))
    {
        // Every 12 hrs we will slam in a new GPS time, to correct for local RTC clock drift
        shouldSet = true;
        LOG_DEBUG("Reapplying external time to correct clock drift %ld secs\n", tv->tv_sec);
    }
    else
        shouldSet = false;

    if (shouldSet)
    {
        lastSetMsec = now;

        // This delta value works on all platforms
        timeStartMsec = now;
        zeroOffsetSecs = tv->tv_sec;

        // If this platform has a setable RTC, set it
#ifdef RV3028_RTC
        if (rtc_found.address == RV3028_RTC)
        {
            Melopero_RV3028 rtc;
#ifdef I2C_SDA1
            rtc.initI2C(rtc_found.port == ScanI2C::I2CPort::WIRE1 ? Wire1 : Wire);
#else
            rtc.initI2C();
#endif
            tm *t = localtime(&tv->tv_sec);
            rtc.setTime(t->tm_year + 1900, t->tm_mon + 1, t->tm_wday, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
            LOG_DEBUG("RV3028_RTC setTime %02d-%02d-%02d %02d:%02d:%02d %ld\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                      t->tm_hour, t->tm_min, t->tm_sec, tv->tv_sec);
        }
#elif defined(PCF8563_RTC)
        if (rtc_found.address == PCF8563_RTC)
        {
            PCF8563_Class rtc;

#ifdef I2C_SDA1
            rtc.begin(rtc_found.port == ScanI2C::I2CPort::WIRE1 ? Wire1 : Wire);
#else
            rtc.begin();
#endif
            tm *t = localtime(&tv->tv_sec);
            rtc.setDateTime(t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
            LOG_DEBUG("PCF8563_RTC setDateTime %02d-%02d-%02d %02d:%02d:%02d %ld\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                      t->tm_hour, t->tm_min, t->tm_sec, tv->tv_sec);
        }
#elif defined(ARCH_ESP32)
        settimeofday(tv, NULL);
#endif

        // nrf52 doesn't have a readable RTC (yet - software not written)
#ifdef HAS_RTC
        readFromRTC();
#endif

        return true;
    }
    else
    {
        return false;
    }
}

bool perhapsSetRTC(RTCQuality q, struct tm &t, int tm_csec)
{
    /* Convert to unix time
    The Unix epoch (or Unix time or POSIX time or Unix timestamp) is the number of seconds that have elapsed since January 1, 1970
    (midnight UTC/GMT), not counting leap seconds (in ISO 8601: 1970-01-01T00:00:00Z).
    */
    time_t res = mktime(&t);
    struct timeval tv;
    tv.tv_sec = res;
    tv.tv_usec = tm_csec;
    // tv.tv_usec = 0; // time.centisecond() * (10 / 1000);

    // LOG_DEBUG("Got time from GPS month=%d, year=%d, unixtime=%ld\n", t.tm_mon, t.tm_year, tv.tv_sec);
    if (t.tm_year < 0 || t.tm_year >= 300)
    {
        // LOG_DEBUG("Ignoring invalid GPS month=%d, year=%d, unixtime=%ld\n", t.tm_mon, t.tm_year, tv.tv_sec);
        return false;
    }
    else
    {
        return perhapsSetRTC(q, &tv, tm_csec);
    }
}

uint32_t getTime_ms()
{

    return -(((uint32_t)millis() - timeStartMsec)) + zeroOffsetSecs * 1000;
}

uint32_t getTime()
{
    return (((uint32_t)millis() - timeStartMsec) / 1000) + zeroOffsetSecs;
}

uint32_t getValidTime(RTCQuality minQuality)
{
    return (currentQuality >= minQuality) ? getTime() : 0;
}
