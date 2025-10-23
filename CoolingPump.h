// CoolingPump.h — Cooling‑loop controller for DIY aquarium chiller
// -----------------------------------------------------------------------------
// Now with burst‑level cooling statistics to evaluate effectiveness.
// -----------------------------------------------------------------------------
#pragma once

#include "Particle.h"
#include <Adafruit_MCP23017.h>
#include "DS18B20.h"
#include "PushNotification.h"
#include "buttons.h"
#include "LCDUniversal.h"

// ---------------------------------------------------------------------------
// Helper utilities (publishing throttled push / log events)
// ---------------------------------------------------------------------------
inline void logEvent(const char* txt) {
    if (Particle.connected()) Particle.publish("cooler_evt", txt, PRIVATE);
}
inline void pushMsg(const char* txt) {
    static PushNotification pn(5min);
    pn.sendWithCooldown(txt);
}
// ---------------------------------------------------------------------------

class CoolingPump {
public:
    // ─────────────────── Public temperature accessors ────────────────────
    double getAquariumTempF() const { _updateTemps(); return _tAquaF; }
    double getInletTempF()    const { _updateTemps(); return _tInletF; }
    double getOutletTempF()   const { _updateTemps(); return _tOutletF; }
    double getBathTempF()     const { _updateTemps(); return _tBathF;  }
    double getTankTempF()     const { return getBathTempF(); } // legacy alias

    // ───────────────────────────── Statistics ─────────────────────────────
    struct Stats {
        // Instantaneous temps
        double aquaCurrentF;
        double inletF, outletF;
        double bathCurrentF;  double currentTankF; // alias
        double bathMinF;      double minTankF;     // alias
        double bathMaxF;      double maxTankF;     // alias
        // Lifetime pump runtime
        double pumpMinutes;
        // Cooling effectiveness
        uint32_t totalBursts;
        double   totalAquaDropF;  // Σ(aquaStart – aquaEnd)
        double   totalBathRiseF;  // Σ(bathEnd  – bathStart)
        double   lastAquaStartF, lastAquaEndF;
        double   lastBathStartF, lastBathEndF;
    };

    Stats getStats() const {
        _updateTemps();
        return {
            _tAquaF,
            _tInletF,
            _tOutletF,
            _tBathF,   _tBathF,
            _bathMinF, _bathMinF,
            _bathMaxF, _bathMaxF,
            _totalPumpMs / 60000.0,
            _totalBursts,
            _totalAquaDropF,
            _totalBathRiseF,
            _lastAquaStartF, _lastAquaEndF,
            _lastBathStartF, _lastBathEndF
        };
    }

    // ───────────────────────────── Lifecycle ─────────────────────────────
    void begin(
        DS18B20*                       busShared,
        const uint8_t*                 addrInlet,
        const uint8_t*                 addrOutlet,
        const uint8_t*                 addrBath,
        DS18B20*                       aquaProbe,
        Adafruit_MCP23017*             io,
        uint8_t                        pumpPin,
        uint8_t                        ledPin,
        std::function<ButtonPress()>   btn1,
        std::function<ButtonPress()>   btn2,
        LCDUniversal*                  lcd,
        std::function<void(void)>      petCb = nullptr)
    {
        _busShared = busShared;
        _aquaProbe = aquaProbe;
        _mcp       = io;
        _pumpPin   = pumpPin;
        _ledPin    = ledPin;
        _pet       = petCb;
        _btn1      = btn1;
        _btn2      = btn2;
        _lcd       = lcd;

        for (int i = 0; i < 8; ++i) {
            _addrInlet[i]  = addrInlet[i];
            _addrOutlet[i] = addrOutlet[i];
            _addrBath[i]   = addrBath[i];
        }

        _mcp->pinMode(_pumpPin, OUTPUT);
        _mcp->digitalWrite(_pumpPin, LOW);
        _mcp->pinMode(_ledPin, OUTPUT);
        _mcp->digitalWrite(_ledPin, LOW);

        _updateTemps(true);
        _bathMinF = _bathMaxF = _tBathF;
        logEvent("cooling_init_ready");
    }

    // ─────────────────────────── Runtime control ──────────────────────────
    void setEnabled(bool en) { _enabled = en; }
    bool isEnabled()  const  { return _enabled; }
    bool isCooling()  const  { return _state == State::BURST; }

    void tick() {
        if (_pet) _pet();
        if (!_enabled) return;

        _updateTemps();
        if (!_valid) { _pumpOff(); return; }

        _updateStats();
        _stateMachine();
    }

    // ─────────────────────────── Configuration ────────────────────────────
    struct Cfg {
        float    T_HIGH       = 78.0f;
        float    T_LOW        = 76.0f;
        float    dT_MIN_BATH  = 4.0f;
        float    dT_STOP_COIL = 0.8f;
        uint32_t MIN_RUN_MS   = 60'000UL;
        uint32_t OFF_PAUSE_MS = 300'000UL;
    } cfg;

private:
    // Handles
    DS18B20* _busShared = nullptr; DS18B20* _aquaProbe = nullptr;
    Adafruit_MCP23017* _mcp = nullptr; uint8_t _pumpPin=0,_ledPin=0;
    std::function<void(void)> _pet; std::function<ButtonPress()> _btn1,_btn2;
    LCDUniversal* _lcd=nullptr;

    // Addresses
    uint8_t _addrInlet[8]{}, _addrOutlet[8]{}, _addrBath[8]{};

    // Cached temps
    mutable float _tAquaF=NAN,_tInletF=NAN,_tOutletF=NAN,_tBathF=NAN;
    mutable bool _valid=false; mutable uint32_t _lastReadMs=0;

    // Stats
    float _bathMinF=999.0f,_bathMaxF=-999.0f; uint32_t _totalPumpMs=0;
    // Burst effectiveness
    uint32_t _totalBursts=0; double _totalAquaDropF=0, _totalBathRiseF=0;
    double _lastAquaStartF=NAN,_lastAquaEndF=NAN,_lastBathStartF=NAN,_lastBathEndF=NAN;
    // Intermediate stats push helper
    uint32_t _lastInterStatsMs = 0;
    static constexpr uint32_t INTER_STATS_MS = 30UL * 60UL * 1000UL; // 30 minutes

    // State machine
    bool _enabled=false; enum class State{IDLE,BURST,LOCK} _state=State::IDLE;
    uint32_t _burstStart=0,_lastStop=0;

    // Sensor reading helpers ------------------------------------------------
    float _readBusF(const uint8_t* a) const {
        double c=_busShared->getTemperature(const_cast<uint8_t*>(a),true);
        float f=_busShared->convertToFahrenheit(c); return (f<0||f>120)?NAN:f;
    }
    float _readAquaF() const {
        double c=_aquaProbe->getTemperature(); float f=_aquaProbe->convertToFahrenheit(c);
        return (f<0||f>120)?NAN:f;
    }
    void _updateTemps(bool force=false) const {
        if(!force&&millis()-_lastReadMs<5000) return; _lastReadMs=millis();
        _tAquaF=_readAquaF(); _tInletF=_readBusF(_addrInlet); _tOutletF=_readBusF(_addrOutlet); _tBathF=_readBusF(_addrBath);
        _valid=!(isnan(_tAquaF)||isnan(_tInletF)||isnan(_tOutletF)||isnan(_tBathF));
    }
    void _updateStats(){ if(_tBathF<_bathMinF) _bathMinF=_tBathF; if(_tBathF>_bathMaxF) _bathMaxF=_tBathF; }

    // GPIO helpers ----------------------------------------------------------
    void _pumpOn(){ _mcp->digitalWrite(_pumpPin,HIGH); _mcp->digitalWrite(_ledPin,HIGH);}    
    void _pumpOff(){ _mcp->digitalWrite(_pumpPin,LOW);  _mcp->digitalWrite(_ledPin,LOW);}    

    // State machine ---------------------------------------------------------
    void _stateMachine(){ const uint32_t now=millis(); switch(_state){
    case State::IDLE:
        if(_tAquaF<cfg.T_HIGH) return;
        if((_tAquaF-_tBathF)<cfg.dT_MIN_BATH) return;
        _pumpOn(); _burstStart = now; _state = State::BURST;
        _lastInterStatsMs = now;
        _lastAquaStartF=_tAquaF; _lastBathStartF=_tBathF;
        char startBuf[64];
        snprintf(startBuf, sizeof(startBuf), "Burst start: Aquarium %.4f°F, Cooling bath %.4f°F", _lastAquaStartF, _lastBathStartF);
        logEvent("burst_start"); pushMsg(startBuf); break;
    case State::BURST:
        // Send intermediate stats every 30 minutes while pump runs
        if(now - _lastInterStatsMs >= INTER_STATS_MS) {
            char midBuf[80];
            double dropSoFar = _lastAquaStartF - _tAquaF;
            snprintf(midBuf, sizeof(midBuf), "Burst %lu min: Aquarium↓%.4f°F (%.1f→%.1f)", (now - _burstStart)/60000UL, dropSoFar, _lastAquaStartF, _tAquaF);
            pushMsg(midBuf);
            _lastInterStatsMs = now;
        }
        if(now - _burstStart < cfg.MIN_RUN_MS) break;
        if((_tOutletF - _tInletF <= cfg.dT_STOP_COIL) || (_tAquaF <= cfg.T_LOW)) {
            _pumpOff(); _totalPumpMs += now - _burstStart; _state = State::LOCK; _lastStop = now;
            _lastAquaEndF = _tAquaF; _lastBathEndF = _tBathF;
            _totalAquaDropF += (_lastAquaStartF - _lastAquaEndF);
            _totalBathRiseF += (_lastBathEndF - _lastBathStartF);
            _totalBursts++;
            char endBuf[80];
            double drop = _lastAquaStartF - _lastAquaEndF;
            snprintf(endBuf, sizeof(endBuf), "Burst end: Aquarium↓%.4f°F (%.1f→%.1f), Cooling bath↑%.4f°F", drop, _lastAquaStartF, _lastAquaEndF, _lastBathEndF - _lastBathStartF);
            logEvent("burst_end"); pushMsg(endBuf);
        }
        break;
        if((_tOutletF-_tInletF<=cfg.dT_STOP_COIL)||(_tAquaF<=cfg.T_LOW)){
            _pumpOff(); _totalPumpMs+=now-_burstStart; _state=State::LOCK; _lastStop=now;
            _lastAquaEndF=_tAquaF; _lastBathEndF=_tBathF;
            _totalAquaDropF+=(_lastAquaStartF-_lastAquaEndF);
            _totalBathRiseF+=(_lastBathEndF-_lastBathStartF);
            _totalBursts++;
            char endBuf[80];
            double drop=_lastAquaStartF-_lastAquaEndF;
            snprintf(endBuf, sizeof(endBuf), "Burst end: Aquarium↓%.4f°F (%.1f→%.1f), Cooling bath↑%.4f°F", drop, _lastAquaStartF, _lastAquaEndF, _lastBathEndF-_lastBathStartF);
            logEvent("burst_end"); pushMsg(endBuf);
        } break;
    case State::LOCK:
        if(now-_lastStop>=cfg.OFF_PAUSE_MS) _state=State::IDLE; break; }}
};