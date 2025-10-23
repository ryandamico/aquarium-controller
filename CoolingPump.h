// CoolingPump.h — header‑only helper that pulls the manual cooling‑pump test
// mode (formerly `flag__coolingPumpTest`) into an isolated class.
//
//  ✱  Uses project‑standard **DS18B20.h** driver (no DallasTemperature).
//  ✱  Non‑blocking; suitable for watchdog‑strict firmware.
//  ✱  Depends on existing helpers   `temp_isButton1Pressed()` / `temp_isButton2Pressed()`
//     via injected callbacks, so we don’t re‑implement any debouncing.
//
//  ✱  Also includes `runPumpCycleWithLogging()` helper to:
//     – read+publish temps
//     – run pump for 15 seconds
//     – read+publish again
//
// ────────────────────────────────────────────────────────────────────────────
//  Build‑system notes & compile‑error fixes
//  ---------------------------------------
//  • We **do not** redefine `LCDUniversal` or `ButtonPress`; instead we forward‑
//    declare them.  Make sure the real headers are included *somewhere earlier*
//    in the compilation unit (e.g. in aquarium_controller.ino or a pre‑compiled
//    “globals.h”).  This prevents the “redefinition of class LCDUniversal” and
//    the enum‑scoping mismatch you hit.
//  • `roundf()` lives in <math.h>; we include it here so any downstream unit
//    that drags in MixingStationIO.h is covered.
//
//  If you still see duplicate‑definition errors, verify the project doesn’t
//  have two copies of LCDUniversal.h under different relative paths — the
//  Device‑OS build system treats those as separate files.
//
// © 2025  Aquarium‑Controller project • MIT licence

#pragma once

#include <Particle.h>
#include <functional>
#include <math.h>              // for roundf() and friends
#include <Adafruit_MCP23017.h>
#include "DS18B20.h"          // One‑Wire temperature driver

// Forward declarations of types that live elsewhere in the codebase.
#include "buttons.h"    // Same enum used by temp_isButtonXPressed()
class LCDUniversal;            // Thin RGB‑LCD wrapper you already ship

class CoolingPump {
public:
    inline void begin(DS18B20* bus,
                      const uint8_t addrIn[8],
                      const uint8_t addrOut[8],
                      Adafruit_MCP23017* ioExp,
                      uint8_t pumpPin,
                      uint8_t ledPin,
                      std::function<ButtonPress(void)> readBtn1,
                      std::function<ButtonPress(void)> readBtn2,
                      LCDUniversal* lcd);

    inline void tick();
    inline void enterTestMode();
    inline void exitTestMode();

    /**
     * Reads current inlet/outlet temps, logs to Particle cloud,
     * runs the cooling pump for 15 seconds, then logs temps again.
     * This method blocks for ~15 sec.
     */
    inline void runPumpCycleWithLogging();

private:
    enum class State : uint8_t { IDLE, TEST };
    State state = State::IDLE;

    DS18B20*           sensors = nullptr;
    Adafruit_MCP23017* mcp     = nullptr;
    LCDUniversal*      lcd     = nullptr;

    std::function<ButtonPress(void)> readBtn1Cb;
    std::function<ButtonPress(void)> readBtn2Cb;

    uint8_t addrIn [8] = {0};
    uint8_t addrOut[8] = {0};
    uint8_t pumpPin    = 0;
    uint8_t ledPin     = 0;

    unsigned long lastTempUpdate = 0;
    unsigned long lastCycleMs     = 0;

    inline bool btn1Pressed() const { return readBtn1Cb && (readBtn1Cb() == ButtonPress::PRESSED); }
    inline bool btn2Pressed() const { return readBtn2Cb && (readBtn2Cb() == ButtonPress::PRESSED); }

    inline float  readTempF(const uint8_t addr[8]);
    inline void   showTemps(float inF, float outF);
    inline void   cleanup();
};

inline void CoolingPump::runPumpCycleWithLogging() {
    unsigned long now = millis();
    unsigned long elapsed = now - lastCycleMs;

    float in1  = readTempF(addrIn);
    float out1 = readTempF(addrOut);

    Particle.publish("cooling_pump_start", String::format("in=%.1fF out=%.1fF elapsed=%lus",
                        in1, out1, elapsed / 1000), PRIVATE);

    mcp->digitalWrite(pumpPin, LOW); // ON
    delay(15000);
    mcp->digitalWrite(pumpPin, HIGH); // OFF

    float in2  = readTempF(addrIn);
    float out2 = readTempF(addrOut);

    Particle.publish("cooling_pump_end", String::format("in=%.1fF out=%.1fF", in2, out2), PRIVATE);
    lastCycleMs = millis();
}

inline void CoolingPump::begin(DS18B20* bus,
                                const uint8_t addrIn_[8],
                                const uint8_t addrOut_[8],
                                Adafruit_MCP23017* ioExp,
                                uint8_t pumpPin_,
                                uint8_t ledPin_,
                                std::function<ButtonPress(void)> readBtn1,
                                std::function<ButtonPress(void)> readBtn2,
                                LCDUniversal* lcd_) {
    sensors  = bus;
    mcp      = ioExp;
    lcd      = lcd_;
    pumpPin  = pumpPin_;
    ledPin   = ledPin_;
    readBtn1Cb = readBtn1;
    readBtn2Cb = readBtn2;
    memcpy(addrIn , addrIn_ , sizeof(addrIn));
    memcpy(addrOut, addrOut_, sizeof(addrOut));

    mcp->pinMode(pumpPin, OUTPUT);
    mcp->digitalWrite(pumpPin, HIGH);
    mcp->pinMode(ledPin, OUTPUT);
    mcp->digitalWrite(ledPin, LOW);

    if (lcd) {
        lcd->clear();
        lcd->display("Cooling pump", "idle", 1);
    }
}

inline void CoolingPump::tick() {
    switch (state) {
        case State::IDLE:
            if (btn1Pressed()) {
                state = State::TEST;
                lastTempUpdate = 0;
                if (lcd) lcd->display("Cooling pump", "test mode", 1);
            }
            break;

        case State::TEST:
            if (millis() - lastTempUpdate > 3000UL) {
                lastTempUpdate = millis();
                mcp->digitalWrite(ledPin, HIGH);
                float inF  = readTempF(addrIn);
                float outF = readTempF(addrOut);
                mcp->digitalWrite(ledPin, LOW);
                showTemps(inF, outF);
            }

            mcp->digitalWrite(pumpPin, btn1Pressed() ? LOW : HIGH);
            if (btn2Pressed()) exitTestMode();
            break;
    }
}

inline void CoolingPump::enterTestMode() {
    state = State::TEST;
    lastTempUpdate = 0;
    if (lcd) lcd->display("Cooling pump", "test mode", 1);
}

inline void CoolingPump::exitTestMode() {
    state = State::IDLE;
    cleanup();
}

inline float CoolingPump::readTempF(const uint8_t addr[8]) {
    float c = sensors->getTemperature(addr);
    return sensors->convertToFahrenheit(c);
}

inline void CoolingPump::showTemps(float inF, float outF) {
    if (!lcd) return;
    char buf[17];
    snprintf(buf, sizeof(buf), "%s -> %s",
             isnan(inF) ? "--" : String(inF,1).c_str(),
             isnan(outF)? "--" : String(outF,1).c_str());
    lcd->setCursor(0,0); lcd->send_string("Cooling pump");
    lcd->setCursor(0,1); lcd->send_string(buf);
}

inline void CoolingPump::cleanup() {
    mcp->digitalWrite(pumpPin, HIGH);
    mcp->digitalWrite(ledPin , LOW);
    if (lcd) lcd->display("Cooling pump", "test ended", 1);
}