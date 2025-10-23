// CoolingPump.h — header‑only helper that pulls the manual cooling‑pump test
// mode (formerly `flag__coolingPumpTest`) into an isolated class.
//
//  ✱  Uses project‑standard **DS18B20.h** driver (no DallasTemperature).
//  ✱  Non‑blocking; suitable for watchdog‑strict firmware.
//  ✱  Depends on existing helpers   `temp_isButton1Pressed()` / `temp_isButton2Pressed()`
//     via injected callbacks, so we don’t re‑implement any debouncing...

#pragma once

#include "Particle.h"
#include <functional>
#include "DS18B20.h"
#include "LCDUniversal.h"
#include "buttons.h"
#include <Adafruit_MCP23017.h>

class CoolingPump {
public:
    inline void begin(DS18B20* bus,
                      const uint8_t* addrIn_,
                      const uint8_t* addrOut_,
                      const uint8_t* addrTank_,
                      Adafruit_MCP23017* ioExp,
                      uint8_t pumpPin_,
                      uint8_t ledPin_,
                      std::function<ButtonPress(void)> readBtn1,
                      std::function<ButtonPress(void)> readBtn2,
                      LCDUniversal* lcd_) {
        sensors     = bus;
        mcp         = ioExp;
        lcd         = lcd_;
        pumpPin     = pumpPin_;
        ledPin      = ledPin_;
        readBtn1Cb  = readBtn1;
        readBtn2Cb  = readBtn2;
        memcpy(addrIn , addrIn_ , sizeof(addrIn));
        memcpy(addrOut, addrOut_, sizeof(addrOut));
        memcpy(addrTank, addrTank_, sizeof(addrTank));

        if (mcp) {
            mcp->pinMode(pumpPin, OUTPUT);
            mcp->digitalWrite(pumpPin, LOW);
            mcp->pinMode(ledPin, OUTPUT);
            mcp->digitalWrite(ledPin, LOW);
        }
    }

    inline void tick() {
        switch (state) {
            case State::IDLE:
                updateAndCacheTemps(); // will update temperatures every ~500ms
                /*
                if (btn1Pressed()) {
                    state = State::TEST;
                    lastTempDisplayUpdate = 0;
                    if (lcd) {
                        lcd->display("Btn: Cooling pmp", "Hold Btn2=Exit", 1);
                    }
                    while (btn1Pressed()) Particle.process();
                }
                */
                break;

            case State::TEST:
                if (millis() - lastTempDisplayUpdate > 3000UL) {
                    lastTempDisplayUpdate = millis();
                    updateAndCacheTemps();
                    showTemps(lastTempInF, lastTempOutF, lastTankTempF);
                }
                if (mcp) mcp->digitalWrite(pumpPin, btn1Pressed() ? HIGH : LOW);
                if (btn2Pressed()) {
                    while (btn2Pressed()) Particle.process();
                    exitTestMode();
                }
                break;
        }
    }

    inline void enterTestMode()   { state = State::TEST;          }
    inline void exitTestMode()    { state = State::IDLE; cleanup();}

    inline float getInletTempF()  { 
        updateAndCacheTemps(true);
        return lastTempInF;  
    }
    inline float getOutletTempF() { 
        updateAndCacheTemps(true);
        return lastTempOutF; 
    }
    inline float getTankTempF() {
        updateAndCacheTemps(true);
        return lastTankTempF; 
    }

    inline void runPumpCycleWithLogging() {
        const unsigned long cycleTimeMs = 2 * 60 * 1000;
        const unsigned long logInterval = 10000; // log every 10 sec
        unsigned long start = millis();
        unsigned long lastLog = 0;

        updateTempsAndPublish("Cooling start");

        if (mcp) {
            mcp->digitalWrite(pumpPin, HIGH); // ON
            pumpStartMillis = millis();
        } else {
            Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Error: Unable to start heat transfer pump""\" }"), PRIVATE);
        }
        
        if (mcp->digitalRead(pumpPin) != HIGH) {
            Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Error: Unable to start heat transfer pump (2)""\" }"), PRIVATE);
        }
        
        while (millis() - start < cycleTimeMs) {
            if (millis() - lastLog >= logInterval) {
                updateTempsAndPublish("Cooling");
                lastLog = millis();
            }
            Particle.process();
        }
        if (mcp) mcp->digitalWrite(pumpPin, LOW);  // OFF

        updateTempsAndPublish("Cooling end");
    }

private:
    enum class State { IDLE, TEST };
    State state = State::IDLE;

    DS18B20* sensors = nullptr;
    Adafruit_MCP23017*  mcp     = nullptr;
    LCDUniversal*       lcd     = nullptr;

    uint8_t addrIn[8];
    uint8_t addrOut[8];
    uint8_t addrTank[8];
    uint8_t pumpPin = 0;
    uint8_t ledPin  = 0;

    std::function<ButtonPress(void)>  readBtn1Cb;
    std::function<ButtonPress(void)>  readBtn2Cb;

    float lastTempInF  = NAN;
    float lastTempOutF = NAN;
    float lastTankTempF = NAN;

    unsigned long lastTempUpdate = 0;
    unsigned long pumpStartMillis  = 0;
    unsigned long lastTempDisplayUpdate = 0;

    inline bool btn1Pressed() const { return readBtn1Cb && readBtn1Cb() == ButtonPress::PRESSED; }
    inline bool btn2Pressed() const { return readBtn2Cb && readBtn2Cb() == ButtonPress::PRESSED; }

    inline float readTempF(const uint8_t* addr) {
        const int MAX_RETRIES = 3;
        double _temp = NAN;
        int i = 0;
    
        sensors->setAddress((uint8_t*)addr);  // <- IMPORTANT
    
        do {
            _temp = sensors->getTemperature(addr);
        } while (!sensors->crcCheck() && MAX_RETRIES > i++);
    
        if (i < MAX_RETRIES) {
            return sensors->convertToFahrenheit(_temp);
        } else {
            return NAN;
        }
    }

    inline void showTemps(float inF, float outF, float tankF) {
        if (!lcd) return;
        lcd->setCursor(0,0);
        lcd->send_string("Btn: Cooling pmp");
        lcd->setCursor(0,1);
        char buf[17];
        snprintf(buf,sizeof(buf),
                 "%s -> %s",
                 isnan(inF)  ? "--" : String(inF ,1).c_str(),
                 isnan(outF) ? "--" : String(outF,1).c_str());
        lcd->send_string(buf);
    }

    inline void cleanup() {
        if (mcp) {
            mcp->digitalWrite(pumpPin, LOW);
            mcp->digitalWrite(ledPin , LOW);
        }
        if (lcd) {
            lcd->clear();
            lcd->display("Cooling pump","test ended",1);
        }
    }

    void updateAndCacheTemps(bool force=false) {
        if (force || millis() - lastTempUpdate > 1000) {
            lastTempUpdate = millis();
            lastTempInF  = readTempF(addrIn);
            lastTempOutF = readTempF(addrOut);
            lastTankTempF = readTempF(addrTank);
        }
    }

    inline void updateTempsAndPublish(const char* label) {
        updateAndCacheTemps();

        char buf[64];
        snprintf(buf, sizeof(buf), "%s: diff=%.2fF in=%.1fF out=%.1fF deltaSec=%d",
                 label,
                 lastTempOutF - lastTempInF,
                 lastTempInF,
                 lastTempOutF,
                 round((millis() - pumpStartMillis) / 1000.0));
        Particle.publish("cooling_pump", buf, PRIVATE);

//        pumpStartMillis = millis();
    }
};
