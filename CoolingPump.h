// CoolingPump.h — consolidated, duplicate‑free
// -----------------------------------------------------------------------------
// * Single‑bus, three hard‑coded DS18B20 addresses.
// * Burst‑cooling state machine (Outlet‑Rise) + logging + push notifications.
// * Optional HW‑watchdog pet callback.
// * Flags: VERBOSE_PUSH for chatty mode.
// * Public helpers: setEnabled / isEnabled, isCooling, getStats, direct temp
//   accessors for Particle.variable.
#pragma once

#include "Particle.h"
#include <Adafruit_MCP23017.h>
#include "DS18B20.h"
#include "PushNotification.h"
#include "buttons.h"
#include "LCDUniversal.h"


// ───────────────────────── FLAGS
constexpr bool VERBOSE_PUSH = true;

// ───────────────────────── LOG + PUSH HELPERS
inline void logEvent(const char* txt) {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < 1000UL) return;             // 1Hz max
    last = now;
    if (Particle.connected()) Particle.publish("cooler_evt", txt, PRIVATE);
}
inline void pushMsg(const char* txt, bool always = false) {
    if (!always && !VERBOSE_PUSH) return;
    static PushNotification pn(120s);
    pn.sendWithCooldown(txt);
}
 
class CoolingPump {
    public:
        // ───── Runtime control ─────
        void setEnabled(bool en) { _enabled = en; }
        bool isEnabled() const   { return _enabled; }
        bool isCooling() const   { return _state == State::BURST; }
    
        struct Stats {
            double tankInitF, currentTankF, minTankF, maxTankF;
            double inletF, outletF;
            double pumpMinutes;
        };
        Stats getStats() const {
            _updateTemps();
            return Stats{ _tankInitF, _tTank, _minTankF, _maxTankF,
                          _tIn, _tOut, _totalPumpMs / 60000.0 };
        }
    
        // Temp getters for Particle.variable
        double getInletTempF()   const { _updateTemps(); return _tIn; }
        double getOutletTempF()  const { _updateTemps(); return _tOut; }
        double getTankTempF()    const { _updateTemps(); return _tTank; }        
        
        struct Cfg {
            float    T_HIGH = 74.0f, T_LOW = 72.0f;
            float    dT_MIN_BATH = 4.0f, dT_STOP_COIL = 0.8f;
            uint32_t MIN_RUN_MS = 60000UL, OFF_PAUSE_MS = 300000UL;
            uint32_t RUNTIME_CAP = 90 * 60000UL, NOTIFY_IV_MS = 3UL * 60 * 60 * 1000UL;
        } cfg;
    
        void begin(DS18B20* bus, const uint8_t* addrIn, const uint8_t* addrOut, const uint8_t* addrTank,
                   Adafruit_MCP23017* io, uint8_t pumpPin, uint8_t ledPin,
                   std::function<ButtonPress(void)> btn1, std::function<ButtonPress(void)> btn2,
                   LCDUniversal* lcd, std::function<void(void)> petCb = nullptr) {
            _bus = bus; 

            for (int i = 0; i < 8; i++) _addrIn[i] = addrIn[i];
            for (int i = 0; i < 8; i++) _addrOut[i] = addrOut[i];
            for (int i = 0; i < 8; i++) _addrTank[i] = addrTank[i];
            
            _mcp = io; _pumpPin = pumpPin; _ledPin = ledPin; _pet = petCb;
            _btn1 = btn1; _btn2 = btn2; _lcd = lcd;
            _mcp->pinMode(_pumpPin, OUTPUT); _mcp->digitalWrite(_pumpPin, LOW);
            _mcp->pinMode(_ledPin , OUTPUT); _mcp->digitalWrite(_ledPin , LOW);
            _updateTemps(true);
            _tankInitF = _tTank; _minTankF = _maxTankF = _tTank;
            pushMsg("Cooler controller ready", true);
            logEvent("init_ready");
        }
    
        void tick() {
            if (_pet) _pet();
            if (!_enabled) return;
            _updateTemps();
            if (!_valid) { _pumpOff(); return; }
            _updateStats();
            _stateMachine();
        }
    
    private:
        // ─── Config/state ----
        bool _enabled = false;
        uint8_t _addrIn[8];
        uint8_t _addrOut[8];
        uint8_t _addrTank[8];
        DS18B20* _bus;
        std::function<void(void)> _pet = nullptr;
        Adafruit_MCP23017* _mcp = nullptr; uint8_t _pumpPin = 0, _ledPin = 0;
        std::function<ButtonPress(void)> _btn1, _btn2; LCDUniversal* _lcd = nullptr;
    
        // temps
        mutable float _tIn = NAN;
        mutable float _tOut = NAN;
        mutable float _tTank = NAN;
        mutable bool _valid = false;
        mutable uint32_t _lastReadMs = 0;
        float _tankInitF = NAN, _minTankF = 999, _maxTankF = -999; 
        uint32_t _totalPumpMs = 0;

        // timers / state machine
        enum class State { IDLE, BURST, LOCK }; State _state = State::IDLE;
        uint32_t _burstStart = 0, _lastStop = 0;
    
        // ─── Sensor helpers ----
        float _readF(const uint8_t* addr) const {
            const int MAX = 3; int n = 0; double c = NAN;
            do { c = _bus->getTemperature(const_cast<uint8_t*>(addr), /*forceSelect=*/true); if (_pet) _pet(); } while (!_bus->crcCheck() && ++n < MAX);
            float f = _bus->convertToFahrenheit(c);
            return (f < 0 || f > 120) ? NAN : f;
        }
        void _updateTemps(bool force = false) const {
            if (!force && millis() - _lastReadMs < 1000) return;
            _lastReadMs = millis();
            _tIn = _readF(_addrIn);
            _tOut = _readF(_addrOut);
            _tTank = _readF(_addrTank);
            _valid = !(isnan(_tIn) || isnan(_tOut) || isnan(_tTank));
        }
        void _updateStats() {
            if (!_valid) return;
            if (_tTank < _minTankF) _minTankF = _tTank;
            if (_tTank > _maxTankF) _maxTankF = _tTank;
        }
    
        // ─── GPIO helpers ----
        void _pumpOn()  { _mcp->digitalWrite(_pumpPin, HIGH); _mcp->digitalWrite(_ledPin, HIGH); }
        void _pumpOff() { _mcp->digitalWrite(_pumpPin, LOW ); _mcp->digitalWrite(_ledPin, LOW );  }
    
        // ─── State machine ----
        void _stateMachine() {
            uint32_t now = millis();
            switch (_state) {
                case State::IDLE:
                    if (_tTank < cfg.T_HIGH) return;
                    if ((_tTank - _tIn) < cfg.dT_MIN_BATH) return;
                    _pumpOn(); 
                    _burstStart = now; 
                    _state = State::BURST;
                    pushMsg("Burst start", true); 
                    logEvent("burst_start");
                    break;
                case State::BURST:
                    if (now - _burstStart < cfg.MIN_RUN_MS) break;
                    if ((_tOut - _tIn) < cfg.dT_STOP_COIL || _tTank <= cfg.T_LOW) {
                        _pumpOff(); _totalPumpMs += now - _burstStart; _state = State::LOCK; _lastStop = now;
                        char buf[48]; snprintf(buf, sizeof(buf), "Burst end %.1fF", _tTank);
                        pushMsg(buf, true); logEvent("burst_end");
                    }
                    break;
                case State::LOCK:
                    if (now - _lastStop >= cfg.OFF_PAUSE_MS) _state = State::IDLE;
                    break;
            }
        }
        
};