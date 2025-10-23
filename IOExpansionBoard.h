#include <Adafruit_MCP23017.h>

class IOExpansionBoard {
    private:
        int _disablePin;
        bool _resetPinLogicLevel;
        int _gpioTestPin; // pin reserved to test connectivity; will be set to LOW and should not be touched externally
        byte _i2cAddressAddition;
        
        bool _isConnected() { //bool doNotForceEnable=false) {
            // I/O expander test: ensure that we're connected to the I/O expansion board
            // note: use locking since I2C is not thread safe (including software timers). see https://docs.particle.io/reference/device-os/api/wire-i2c/lock/
            // note: Never use lock() or WITH_LOCK() within a SINGLE_THREADED_BLOCK() as deadlock can occur.
        
            // update: no longer doing this, since it's not obvious from the outside that checking for connectivity could enable the board 
            /*
            bool wasIOExpModuleDisabled = !isEnabled(); 
            if (!doNotForceEnable && wasIOExpModuleDisabled) {
                enableAndReset();
            }
            */
                
            // test 1: read all GPIO bits and make sure we don't get 0xffff back (which happens if the board is disconnected). note that we reserve one GPIO pin to always set low so that 0xffff cannot be a valid result if everything else were to be set to HIGH.
            bool ioTest1Failed = false;
            uint16_t gpioValues = readGPIOAB();
            if (gpioValues == 0xffff) { // note: test is always set to low so 0xffff should never be a valid response
                ioTest1Failed = true;
            }
        
            // test 2: toggle our test bit, reading back values to ensure they change (since I suspect test 1 may not always work, e.g. when the test LED never turned on)
            bool ioTest2Failed = false;
            bool gpioValue;
            WITH_LOCK(Wire) {
                // _gpioTestPin starts low, so set it to HIGH here
                mcp.digitalWrite(_gpioTestPin, HIGH);
            }
            delayMicroseconds(1);
            WITH_LOCK(Wire) {
                gpioValue = mcp.digitalRead(_gpioTestPin);
            }
            if (gpioValue != HIGH) {
                ioTest2Failed = true;
            } else {
                WITH_LOCK(Wire) {
                    // set _gpioTestPin back to LOW and *make sure to leave it there* for other data integrity checks
                    mcp.digitalWrite(_gpioTestPin, LOW);
                }
                delayMicroseconds(1);
                WITH_LOCK(Wire) {
                    gpioValue = mcp.digitalRead(_gpioTestPin);
                }
                if (gpioValue != LOW) {
                    ioTest2Failed = true;
                }
            }
            
// temp
__tmp_test1Failed = ioTest1Failed;
__tmp_test2Failed = ioTest2Failed;
            //_isConnected = !(ioTest1Failed || ioTest2Failed);
            ///return _isConnected;

            /*
            // restore last state
            if (!doNotForceEnable && wasIOExpModuleDisabled) {
                disable();
            }
            */
            
            return !(ioTest1Failed || ioTest2Failed);
        }  

        bool _digitalRead(int pin, bool resetOnFailure=false) {
            // Read all GPIO to ensure that test pin matches expected value as well
            // "For the MCP23017 you specify a pin number from # 0 to 15 for the GPIOA0...GPIOA7, GPIOB0...GPIOB7 pins (i.e. pin 12 is GPIOB4)."
            // I.e. Pin number 0..7 corresponds to A0..A7, and 8..15 corresponds to B0..B7
            uint16_t gpioValues;
            int retryDelaysMs[] = {0, 1, 10};
            for (int i=0; i<3; i++) {
                gpioValues = readGPIOAB();
                bool gpioTestPinValue = bitRead(gpioValues, _gpioTestPin); // see https://garretlab.web.fc2.com/en/arduino/inside/hardware/arduino/avr/cores/arduino/Arduino.h/bit_operation.html
                bool pinValue = bitRead(gpioValues, pin); 
                if (gpioTestPinValue == LOW) { // if we get the expected value, break out of retry loop and return read value
                    lastReadErrorFlag = false;  
                    return pinValue;
                } else { // otherwise, wait briefly and try again
                    debug_digitalReadSafeRetries++;
                    delay(retryDelaysMs[i]);
                }
            }
            
            lastReadErrorFlag = true;
            if (resetOnFailure) {
// temp debugging -- not sure if this is safe at all
// TODO: should we restart if the board is just disconnected?
particle::Future<bool> publishFuture = Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] ALERT: Error confirming read value from IO expansion board. pin: %d; gpioValues: 0x%x, i2cAddressAddition: 0x%x""\" }", pin, gpioValues, _i2cAddressAddition), PRIVATE);
disable(); // set all outputs to safe states before resetting
delay(1000); // hope the message gets sent before restarting, though we don't want to wait long -- our reset flag will convey the alert to the user
System.reset(RESET_REASON_IO_EXP_READ_MISMATCH, RESET_NO_WAIT); // RESET_NO_WAIT so we're not stuck in an undefined state any longer than we need to be
            }
            
            return false; // should never get here!
        }
        
        void _digitalWrite(int pin, bool value, bool resetOnWriteFailure=true) {
            int retryDelaysMs[] = {0, 1, 10};
            for (int i=0; i<3; i++) {
                WITH_LOCK(Wire) {
                    mcp.digitalWrite(pin, value);
                }
                if ((_digitalRead(pin, false) == value) && !lastReadErrorFlag) { // don't reset if the digital read fails; let that happen further down here in  digital write
                    return;
                } else {
                    debug_digitalWriteSafeRetries++;
                    delay(retryDelaysMs[i]);
                }
            }
             
            if (resetOnWriteFailure) {
// temp debugging -- not sure if this is safe at all
// TODO: should we restart if the board is just disconnected?
particle::Future<bool> publishFuture = Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] ALERT: Error reading written value to IO expansion board. pin: %d; value: %d, i2cAddressAddition: 0x%x""\" }", pin, value, _i2cAddressAddition), PRIVATE);
disable(); // set all outputs to safe states before resetting
delay(1000); // hope the message gets sent before restarting, though we don't want to wait long -- our reset flag will convey the alert to the user
System.reset(RESET_REASON_IO_EXP_WRITE_READ_MISMATCH, RESET_NO_WAIT); // RESET_NO_WAIT so we're not stuck in an undefined state any longer than we need to be
            }
        }                
        
    protected:
        Adafruit_MCP23017 mcp;
        
        // "For the MCP23017 you specify a pin number from # 0 to 15 for the GPIOA0...GPIOA7, GPIOB0...GPIOB7 pins (i.e. pin 12 is GPIOB4)."
        // I.e. Pin number 0..7 corresponds to A0..A7, and 8..15 corresponds to B0..B7
        virtual void setPinModes(void) = 0; // this will be called within WITH_LOCK
        
    public:
        int debug_digitalWriteSafeRetries = 0;
        int debug_digitalReadSafeRetries = 0;
        int debug_autoRepairSuccesses1 = 0;
        int debug_autoRepairSuccesses2 = 0;
        bool __tmp_test1Failed = false;
        bool __tmp_test2Failed = false;
        bool lastReadErrorFlag = false; // set on every call to digital read, so the calling function knows whether to disregard the results
    
        IOExpansionBoard(int disablePin, bool resetPinLogicLevel, int gpioTestPin, byte i2cAddressAddition=0x00) {
            _disablePin = disablePin;
            _resetPinLogicLevel = resetPinLogicLevel;
            _gpioTestPin = gpioTestPin;
            _i2cAddressAddition = i2cAddressAddition;
        }
        
        void initializeToDisabled() { // call once to set pin mode, followed by enableAndReset() to enable and reset the board
            pinMode(_disablePin, OUTPUT);
            pinSetDriveStrength(_disablePin, DriveStrength::HIGH); // required to ensure optocoupler LED has enough current, if present
            disable();
        }
        
        bool isEnabled() {
            //return !::digitalRead(D9);
            return !(::digitalRead(_disablePin) == _resetPinLogicLevel); // important: use global namespace when referencing digitalRead(), not our class implementation belos
            //return !(pinReadFast(_disablePin) == _resetPinLogicLevel);
        }
        
        bool enableAndReset() {
            if (_resetPinLogicLevel == HIGH) {
                pinResetFast(_disablePin);
            } else {
                pinSetFast(_disablePin);
            }
            delayMicroseconds(10); // spec says device is active after "0ns". see https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
            WITH_LOCK(Wire) {
                mcp.begin(_i2cAddressAddition);
                mcp.pinMode(_gpioTestPin, OUTPUT);
                mcp.digitalWrite(_gpioTestPin, LOW);
                setPinModes();
            }
            
            return _isConnected(); //(true); // note: we call _isConnected directly to set doNotForceEnable to true (avoiding infinite recursion)
        }
        
        void disable() { // uses pinSetFast() and delayMicroseconds() to be ISR-safe
            if (_resetPinLogicLevel == HIGH) {
                pinSetFast(_disablePin);
            } else {
                pinResetFast(_disablePin);
            }
            delayMicroseconds(5); // spec says 1us is minimum to reset board, and 1us is max time for board to set outpits to high impedience. see https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
        }
        
        bool digitalReadOrReset(int pin) {
            return _digitalRead(pin, true);
        }
        
        bool digitalRead(int pin) {
            return _digitalRead(pin, false);
        }

        void digitalWriteOrReset(int pin, bool value) {
            // void _digitalWrite(int pin, bool value, resetOnWriteFailure=true)
            _digitalWrite(pin, value, true);
        }
        
        void digitalWrite(int pin, bool value) {
            _digitalWrite(pin, value, false);
        }        

        bool isConnected(bool autoRepair=false) { // wrapper function
            bool connected = _isConnected();
            
            if (!connected && autoRepair) {
                // update: does trying again fix the issue?
                delay(1);
                connected = _isConnected();
                if (connected) {
                    debug_autoRepairSuccesses1++;
                } else {
                    // update: no longer doing this since we don't want this method to change the enabled state of the board
                    /*
                    // if we're not connected, try to reinitialize
                    // TODO: consider setting flag here to monitor how often this happens
                    disable();
                    enableAndReset();
                    connected = _isConnected();
                    if (connected) {
                        debug_autoRepairSuccesses2++;
                    }
                    */
                }
            }
            return connected;
        }

        uint16_t readGPIOAB() {
            uint16_t gpioValues;
            WITH_LOCK(Wire) {
                gpioValues = mcp.readGPIOAB(); // returns 0xffff if the device is disconnected
            }
            return gpioValues;
        }
        
        
};