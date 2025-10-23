#define HEATER_ENABLE           HIGH
#define HEATER_DISABLE          LOW // relay is active low to disconnect heater AC plug

class RelayModule {
      
    private:
        Adafruit_MCP23017 *_mcp2;
        int _heaterRelayPin;
        int _canisterFilterRelayIOExpPin;
        bool _isHeaterOn = false;
        bool _isCanisterFilterOn = false;
        
    public:
        RelayModule(int heaterRelayPin, int canisterFilterRelayIOExpPin, Adafruit_MCP23017 *mcp2) {
            _heaterRelayPin = heaterRelayPin;
            _canisterFilterRelayIOExpPin = canisterFilterRelayIOExpPin;
            _mcp2 = mcp2;
        }
        
        void initalize() {
            // turn everything on by default
            
            pinMode(_heaterRelayPin, OUTPUT_OPEN_DRAIN); // "HIGH (1) leaves the output in high impedance state, LOW (0) pulls the output low. Typically used with an external pull-up resistor to allow any of multiple devices to set the value low safely.
            pinSetDriveStrength(_heaterRelayPin, DriveStrength::HIGH); // just in case
            setHeater(true);
            
            // *TODO*: confirm that mcp2 is initialized and this command goes through
            setCanisterFilter(true); // sets pinMode
        }
        
        void setHeater(bool turnOn) {
            _isHeaterOn = turnOn;
            digitalWrite(_heaterRelayPin, turnOn ? HEATER_ENABLE : HEATER_DISABLE);
        }
        
        bool isHeaterOn() {
            return _isHeaterOn;
        }
        
        void setCanisterFilter(bool turnOn) {
            _isCanisterFilterOn = turnOn;
            if (turnOn) {
                WITH_LOCK(Wire) { 
                    _mcp2->pinMode(_canisterFilterRelayIOExpPin, INPUT);
                }
            } else {
                WITH_LOCK(Wire) { 
                    _mcp2->pinMode(_canisterFilterRelayIOExpPin, OUTPUT);
                    _mcp2->digitalWrite(_canisterFilterRelayIOExpPin, LOW);
                } 
            }
        }
        
        bool isCanisterFilterOn() {
            return _isCanisterFilterOn;
        }
};