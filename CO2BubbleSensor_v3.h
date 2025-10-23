












// **** TODO ***** stopCO2() is getting called repeatedly on faults, and _co2StopTime plus other state variables keep getting reset to zero (meaning that downstream use of those variables is inaccurate)
// **** TODO ***** add critical alerts back in once bug(s) are fixed (search for "too high (" -- two spots
















// TODO: change volatile tick variable to a simple bool set true by the ISR anytime a new bubble is detected
// *TODO* Implement IO exp error checking + reset 
// TODO: store isCO2Started() in flag for I2C-less reads?
// TODO: implement cool-down period to prevent repeating CO2 fault alerts
// TODO: test all fault states



#define FLASH_TEST_LED
#define BUBBLES_PER_SECOND_MIN 0.25
#define BUBBLES_PER_SECOND_MAX 1.25

#define BUBBLE_BUFFER_LENGTH 10

// CO2 should start/stop at 00 past the hours below
#define CO2_START_HOUR 7
#define CO2_STOP_HOUR 16

#define CIRCULAR_BUFFER_INT_SAFE
#define CIRCULAR_BUFFER_XS // note: limits index to a byte, so 255 items max
#include "CircularBuffer.h"

class CO2BubbleSensor_v3 {
      
    private:
        Adafruit_MCP23017 *_mcp2;
        int _bubbleSensorPin;
        int _ioExpSolenoidControlPin;
        const bool BUBBLE_DETECTED = true; // pulse high when bubble detected
        
        // state variables cleared when CO2 is turned on
        CircularBuffer<system_tick_t, BUBBLE_BUFFER_LENGTH> _bubbleBuffer; // array of millis() timestamps when bubbles were detected
        int bubbleBufferCopy[BUBBLE_BUFFER_LENGTH];
        int bubbleBufferCopySize;
        system_tick_t _co2StartTime = 0;
        system_tick_t _co2StopTime = 0;
        int _bubbleCountAtCo2StopTime = 0;
        volatile system_tick_t _lastBubbleTime = 0; // TODO: do we need to disable/enable interrupts whenever we read this since it's > ~8 bits?
        volatile int _bubbleCount = 0;
        // end state variables

        void bubblePulse_handler() {
            bool bubbleDetected = (pinReadFast(_bubbleSensorPin) == BUBBLE_DETECTED);
            
            #ifdef FLASH_TEST_LED
                if (bubbleDetected) {
                    pinSetFast(LED);
                } else {
                    pinResetFast(LED);
                }
            #endif

            if (bubbleDetected) {
                _bubbleCount++;
                _lastBubbleTime = millis();
                _bubbleBuffer.push(_lastBubbleTime);
            }
        }
        
        void _attachInterrupt() {
            bool success = attachInterrupt(_bubbleSensorPin, &CO2BubbleSensor_v3::bubblePulse_handler, this, CHANGE); // note: this doesn't seem to work when called from startup() vs setup()
            if (!success) {
                System.reset(RESET_REASON_CO2_ATTACH_IRQ_FAILED, RESET_NO_WAIT); // make sure we know if this fails for some reason
            }
        }

        void _detachInterrupt() {
            detachInterrupt(_bubbleSensorPin);
        }

    public:
        enum Faults {
            NONE, // no faults
            BPS_TOO_LOW, // bubbles per second is too low after a reasonable amount of time post-start
            BPS_TOO_HIGH, // too many bubbles per second detected (e.g. unexpected tank pressure change)
            BUBBLE_DETECTION_STUCK, // a single bubble has been detected for too long (e.g. sensor problem)
            BUBBLES_CONTINUING_TOO_LONG_AFTER_SHUTOFF, // self-explanatory (e.g. if solenoid gets stuck)
            CO2_ON_AFTER_HOURS, // CO2 injection is running too late in the day
        };
        volatile Faults _currentFault = Faults::NONE;

        CO2BubbleSensor_v3(int bubbleSensorPin, int ioExpSolenoidControlPin, Adafruit_MCP23017 *mcp2) {
            _bubbleSensorPin = bubbleSensorPin;
            _ioExpSolenoidControlPin = ioExpSolenoidControlPin;
            _mcp2 = mcp2;
        }
        
        void initialize() { // NOTE: does not work if called from startup()
            // note: mcp2 should already be initialized, but we can try calling it again
//**TODO** Ensure that _mcp2 is connected before setting pinmodes, etc.
            WITH_LOCK(Wire) {
                _mcp2->pinMode(_ioExpSolenoidControlPin, OUTPUT); // TODO: should this be duplicated with our main code file?
                _mcp2->digitalWrite(_ioExpSolenoidControlPin, LOW);
                _mcp2->digitalWrite(PIN_IOEXP2__LED_2, LOW);

            }
            pinMode(_bubbleSensorPin, INPUT_PULLUP); // woohoo! solved the issue with weak signal by adding _PULLUP
            _detachInterrupt(); // safety just in case this gets called multiple times (would that duplicate interrupt handlers?)
            _attachInterrupt();
        }

        void startCO2(bool simluationOnly=false) {
            _bubbleBuffer.clear();
            _co2StartTime = millis();
            _co2StopTime = 0;
            _bubbleCountAtCo2StopTime = 0;
            _lastBubbleTime = 0;
            _bubbleCount = 0;
            _currentFault = Faults::NONE;
            
            if (!simluationOnly) {
                WITH_LOCK(Wire) {
                    _mcp2->digitalWrite(_ioExpSolenoidControlPin, HIGH);
                    _mcp2->digitalWrite(PIN_IOEXP2__LED_2, HIGH);
                }        
            }
        }
        
        bool isCO2Started() { // NOTE: When mcp2 is disabled in emergency mode in our .ino file, this returns a calue that mistanekly triggers alerts
            //return digitalRead(PIN__CO2_SOLENOID);
            bool state;
//**TODO** return false if MCP board is disconnected            
            WITH_LOCK(Wire) {
                state = _mcp2->digitalRead(_ioExpSolenoidControlPin);
            }
            return state;
        }

        void stopCO2() { // note: monitoring should continue in order to catch BUBBLES_CONTINUING_TOO_LONG_AFTER_SHUTOFF scenario
            // new: only update these state variables if we're actually switching CO2 from on to off
            if (isCO2Started()) {
                _co2StopTime = millis();
                _bubbleCountAtCo2StopTime = _bubbleCount;
            }
            //digitalWrite(PIN__CO2_SOLENOID, LOW);
            WITH_LOCK(Wire) {
                _mcp2->digitalWrite(_ioExpSolenoidControlPin, LOW);
                _mcp2->digitalWrite(PIN_IOEXP2__LED_2, LOW);
            }
        }

        void pushDebugInfo() {
            //static char debugStr[] = 
            //String temp_thisVariableIsABadIdea;
            //temp_thisVariableIsABadIdea.reserve(BUBBLE_BUFFER_LENGTH * 10);            
            String temp_thisVariableIsABadIdea = String("0, ");
            _detachInterrupt(); // disable, then enable interrupt to ensure data isn't changed while we're iterating through it (assuming the odds are low of missing a bubble and skewing the reported bubble rate)
    		using index_t = decltype(_bubbleBuffer)::index_t; // "the following ensures using the right type for the index variable" -- see https://github.com/rlogiacco/CircularBuffer and https://build.particle.io/libs/CircularBuffer/1.3.3/tab/example/CircularBuffer.ino
    		for (index_t i = 0; i < _bubbleBuffer.size(); i++) {
    		    if (i > 0) {
    		        temp_thisVariableIsABadIdea.concat(String(_bubbleBuffer[i] - _bubbleBuffer[i-1]));
    		        temp_thisVariableIsABadIdea.concat(", ");
    		    }
    		}
            _attachInterrupt(); 
            static PushNotification notification_bufferDebug(1min);
            notification_bufferDebug.sendWithCooldown(String::format("CO2 buffer: %s", temp_thisVariableIsABadIdea.c_str()));
        }

        float getBubblesPerSecondCombined(bool debug=false) {
            float bps = getBubblesPerSecond(5, debug); // short enough to see near-realtime fluxuations
            if (bps == 0) {
                return getBubblesPerSecond(120, debug); // long enough to detect 0.01 BPS
            } else {
                return bps;
            }
        }

        float getBubblesPerSecond(int averagingWindowSec=5, bool debug=false) { // returns either a short (~5 sec) average, or a long (~2 minutes --> min resolution of 0.01 BPS) average
            const system_tick_t now = millis();

            // memoize
            static system_tick_t lastBubbleTimeMemoized = 0;
            if (lastBubbleTimeMemoized == 0 || (lastBubbleTimeMemoized != _lastBubbleTime)) {
                lastBubbleTimeMemoized = _lastBubbleTime;
                
                // briefly disable interrupts to copy bubble buffer
                _detachInterrupt(); // disable, then enable interrupt to ensure data isn't changed while we're iterating through it (assuming the odds are low of missing a bubble and skewing the reported bubble rate)
                bubbleBufferCopySize = _bubbleBuffer.size();
        		using index_t = decltype(_bubbleBuffer)::index_t; // "the following ensures using the right type for the index variable" -- see https://github.com/rlogiacco/CircularBuffer and https://build.particle.io/libs/CircularBuffer/1.3.3/tab/example/CircularBuffer.ino
        		for (index_t i = 0; i < bubbleBufferCopySize; i++) {
                    bubbleBufferCopy[i] = _bubbleBuffer[i];
                }
                _attachInterrupt();
            }
            
            // calculate average
            system_tick_t totalTimeBetweenBubblesMs = 0;
            int numBubblesCounted = 0;
            bool passedThreshold = false;            
            for (int i = 0; i < bubbleBufferCopySize; i++) {
                if (debug) {
                    Particle.publish("debug", String::format("i: %d, bubbleBufferCopy[i]: %u", i, bubbleBufferCopy[i]));
                    delay(1000);
                }
    		    if (passedThreshold) {
		            totalTimeBetweenBubblesMs += bubbleBufferCopy[i] - bubbleBufferCopy[i-1];
		            numBubblesCounted++;
			        //avgBubbleIntervalMs += durationMs / (float)(bubbleBufferCopy.size() - 1);
    		    } else if (now - bubbleBufferCopy[i] <= ((system_tick_t)averagingWindowSec*1000)) {
    		        // only consider bubbles which occurred past our threshold timestamp in millis()
    		        passedThreshold = true;
    		        continue; // skip to next data point so we can start calculating elapsed times based on the current timestamp
    		    }
    		}
            
            //delay(1000);
            //Particle.publish("debug", String::format("numBubblesCounted: %d, totalTimeBetweenBubblesMs: %u, debug_indexThatHitThreshold: %d", 
                //numBubblesCounted, totalTimeBetweenBubblesMs, debug_indexThatHitThreshold));
            
            if (numBubblesCounted == 0) {
                return 0;
            } else {
                return (numBubblesCounted / (float)totalTimeBetweenBubblesMs)*1000.0;
            }
        }
        
        /*
        float getBubblesPerSecond(int thresholdSec=5, bool debug=false) {
            //float avgBubbleIntervalMs = 0.0;
            const system_tick_t now = millis();
            system_tick_t totalTimeBetweenBubblesMs = 0;
            int numBubblesCounted = 0;
    		
            _detachInterrupt(); // disable, then enable interrupt to ensure data isn't changed while we're iterating through it (assuming the odds are low of missing a bubble and skewing the reported bubble rate)
            bool passedThreshold = false;
    		using index_t = decltype(_bubbleBuffer)::index_t; // "the following ensures using the right type for the index variable" -- see https://github.com/rlogiacco/CircularBuffer and https://build.particle.io/libs/CircularBuffer/1.3.3/tab/example/CircularBuffer.ino
    		for (index_t i = 0; i < _bubbleBuffer.size(); i++) {
                if (debug) {
                    Particle.publish("debug", String::format("i: %d, _bubbleBuffer[i]: %u", i, _bubbleBuffer[i]));
                    delay(1000);
                }
    		    if (passedThreshold) {
		            totalTimeBetweenBubblesMs += _bubbleBuffer[i] - _bubbleBuffer[i-1];
		            numBubblesCounted++;
			        //avgBubbleIntervalMs += durationMs / (float)(_bubbleBuffer.size() - 1);
    		    } else if (now - _bubbleBuffer[i] <= ((system_tick_t)thresholdSec*1000)) {
    		        // only consider bubbles which occurred past our threshold timestamp in millis()
    		        passedThreshold = true;
    		        continue; // skip to next data point so we can start calculating elapsed times based on the current timestamp
    		    }
    		}
            _attachInterrupt();
            
//delay(1000);
//Particle.publish("debug", String::format("numBubblesCounted: %d, totalTimeBetweenBubblesMs: %u, debug_indexThatHitThreshold: %d", 
    //numBubblesCounted, totalTimeBetweenBubblesMs, debug_indexThatHitThreshold));
            
            if (numBubblesCounted == 0) {
                return 0;
            } else {
                return (numBubblesCounted / (float)totalTimeBetweenBubblesMs)*1000.0;
            }
        }
        */
        
        void integrationTest() {
            // note: we don't want to actually call initialize() here since we're just simulating data
            startCO2(true); // simulation only
            
            // simulate data points (note: taken directly from ISR handler -- make sure code is in sync
            system_tick_t now = millis();
            const int numSimulatedBubbles = 5;
            system_tick_t simulatedBubbleTimes[numSimulatedBubbles] = { now - 12*1000, now - 9*1000, now - 8*1000, now - 7*1000, now - 6*1000};
            for (int i=0; i<numSimulatedBubbles; i++) {
                _bubbleCount++;
                _bubbleBuffer.push(simulatedBubbleTimes[i]);
            }
            
            Particle.publish("getBubblesPerSecondCombined()", String::format("%.2f, %.2f, %.2f, %.2f; %.2f", 
                getBubblesPerSecond(999, false),
                getBubblesPerSecond(3, false),
                getBubblesPerSecond(9, false),
                getBubblesPerSecond(11, false),
                getBubblesPerSecondCombined(false)
            ));
        }

        static int getStartHour() {
            return CO2_START_HOUR;
        }
        
        static int getStopHour() {
            return CO2_STOP_HOUR;
        }
        
        static bool isActiveTimeOfDay() {
            if (!Time.isValid()) {
                PushNotification::send(String::format("WARNING: CO2 injection time cannot be determine because system time."), true);
                return false;   
            }
            return (Time.hour() >= CO2_START_HOUR && Time.hour() < CO2_STOP_HOUR);
        }

        CO2BubbleSensor_v3::Faults getFaultCode() {
            return _currentFault;
        }
        
        void updateFaultCodeAndSendAlerts() {
            bool co2Running = isCO2Started();
            float bps = getBubblesPerSecond(10); // was 5 seconds
            float bpsLong = getBubblesPerSecond(5*60); // note: testing 5 minutes instead of 1 minute. example bad data: "message":"[Aquarium controller] CO2 buffer: 0, 178039, 51, 174372, 171715, 51, 181750, 51, 180783, 50, "



// TODO: case where CO2 should be running/detected but isn't

            
            // ensure that bubble rate isn't too high
            if (_bubbleBuffer.size() == BUBBLE_BUFFER_LENGTH) { // make sure our circular buffer is full so that we hae enough data points
                if (bps > BUBBLES_PER_SECOND_MAX && co2Running) {
                    if (bps != bpsLong) { // NEW: if both values are the same, that means that the bps variable couldn't be set because there weren't enough data points in the seconds parameter -- but if the BPS rate is too high, that shouldn't be the case
                        //if (_currentFault != Faults::BPS_TOO_HIGH)
                        _currentFault = Faults::BPS_TOO_HIGH;
                        stopCO2();
                        static PushNotification notification_BPS_TOO_HIGH(5min);
                        notification_BPS_TOO_HIGH.sendWithCooldown(String::format("WARNING: CO2 injection rate is too high (%.2f bps). Shutting off CO2. Debug: bpsLong: %.2f, _bubbleCount: %d, _bubbleCountAtCo2StopTime: %d", 
                            bps, bpsLong, _bubbleCount, _bubbleCountAtCo2StopTime), false); //////////~~~~~true);
                        pushDebugInfo(); // debug
                    }
                }
            }
            
            // endure that bubble rate isn't too low
            if (co2Running) {
                if (millis() - _co2StartTime > 10*60*1000 && _bubbleCount == 0 ) {
                    _currentFault = Faults::BPS_TOO_LOW;
                    stopCO2();
                    static PushNotification notification_BPS_TOO_LOW_1(5min);
                    notification_BPS_TOO_LOW_1.sendWithCooldown(String::format("WARNING: CO2 not detected after opening solenoid. Shutting off CO2 in case of sensor fault."), true);
                    pushDebugInfo();
                }
                if (millis() - _co2StartTime > 30*60*1000 && bpsLong < BUBBLES_PER_SECOND_MIN && bps < BUBBLES_PER_SECOND_MIN) {
                    _currentFault = Faults::BPS_TOO_LOW;
                    stopCO2();
                    pushDebugInfo();
                    static PushNotification notification_BPS_TOO_LOW_2(5min);
                    notification_BPS_TOO_LOW_2.sendWithCooldown(String::format("WARNING: CO2 injection rate too low (%.2f bps) after opening solenoid. Shutting off CO2. bps: %.2f", bpsLong, bps), true);
                    pushDebugInfo();
                }
            }
            
            // ensure that bubbles have stopped
// TODO: fix bug hacked around via manualBPS -- it's not accurate at all
            float manualBPS = (_bubbleCount - _bubbleCountAtCo2StopTime) / (int)((millis() - _co2StopTime)/1000.0);
            if ((millis() - _co2StopTime > 60*60*1000) && (manualBPS /*bpsLong*/ > 0.075) && !co2Running) {
                _currentFault = Faults::BUBBLES_CONTINUING_TOO_LONG_AFTER_SHUTOFF;
                stopCO2(); // just in case this didn't work previously
                static PushNotification notification_BUBBLES_CONTINUING_TOO_LONG_AFTER_SHUTOFF_1(5min);
                notification_BUBBLES_CONTINUING_TOO_LONG_AFTER_SHUTOFF_1.sendWithCooldown(String::format("ERROR: Possible CO2 leak detected -- injection rate too high (%.2f bps) %d minutes after solenoid shutoff. Check the tank. Debug: millis() - _co2StopTime: %u, _bubbleCount: %d, _bubbleCountAtCo2StopTime: %d, bps: %.2f, bpsLong: %.2f, co2Running: %d", 
                    manualBPS/*bpsLong*/, (int)((millis() - _co2StopTime)/1000.0/60), millis() - _co2StopTime, _bubbleCount, _bubbleCountAtCo2StopTime, bps, bpsLong, co2Running
                    ), false); ////////~~~~~~true);
                pushDebugInfo();
            }
            
            // ensure CO2 isn't running after hours (note: allow some leeway for debugging?)
            //if (!isActiveTimeOfDay() && (co2Running || bpsLongWindow > 0.05)) {
            if (!isActiveTimeOfDay() && Time.minute() > 5 && co2Running) {//bpsLong > 0.05) {
                if (Time.isValid()) { // allow a few minutes' buffer time
                    if (getBubblesPerSecondCombined() >= 0.01) { // new: stop false alerts
                        _currentFault = Faults::BUBBLES_CONTINUING_TOO_LONG_AFTER_SHUTOFF;
                        stopCO2(); // just in case this didn't work previously
                        static PushNotification notification_BUBBLES_CONTINUING_TOO_LONG_AFTER_SHUTOFF_2(15min);
                        notification_BUBBLES_CONTINUING_TOO_LONG_AFTER_SHUTOFF_2.sendWithCooldown(String::format("ERROR: CO2 detected running after hours (%.2f bps). Shutting off CO2.", getBubblesPerSecondCombined()), true);
                        pushDebugInfo();
                    }
                }
            }
            
            // TODO: check for IO exp errors?
            
            // misc: check for sensor stuck in "bubble detected" state
            if (co2Running && pinReadFast(_bubbleSensorPin) == BUBBLE_DETECTED) { // should be unlikely since pulse duty cycle is so low
                delay(50); 
                if (pinReadFast(_bubbleSensorPin) == BUBBLE_DETECTED) {

// NOTE: too many bugs and false alarms here. TODO: try fixing this again later
/*                    
                    // debugging...
                    // important note from docs: "Please note that if the pin is already reading the desired value when the function is called, it will wait for the pin to be the opposite state of the desired value, and then finally measure the duration of the desired value."
                    unsigned long detectionDurationDetected1 = pulseIn(_bubbleSensorPin, BUBBLE_DETECTED); // note: pulseIn() times out and returns 0 after 3000ms. returns value in us.
                    unsigned long detectionDurationDetected2 = pulseIn(_bubbleSensorPin, BUBBLE_DETECTED); // note: pulseIn() times out and returns 0 after 3000ms. returns value in us.
                    
                    _currentFault = Faults::BUBBLE_DETECTION_STUCK;
                    stopCO2();
                    static PushNotification notification_CO2_ON_AFTER_HOURS(5min);
                    notification_CO2_ON_AFTER_HOURS.sendWithCooldown(String::format("WARNING: CO2 sensor fault detected (output may be stuck high). Shutting off CO2. detectionDurationDetected1: %ul ms, detectionDurationDetected2: %ul ms", 
                        bps, detectionDurationDetected1/1000, detectionDurationDetected2/1000), true);
*/
                    
                    
                    
                    /*
                    delay(150);
                    if (pinReadFast(_bubbleSensorPin) == BUBBLE_DETECTED) {
                        _currentFault = Faults::BUBBLE_DETECTION_STUCK;
                        stopCO2();
                        static PushNotification notification_CO2_ON_AFTER_HOURS(5min);
                        notification_CO2_ON_AFTER_HOURS.sendWithCooldown(String::format("WARNING: CO2 sensor fault detected (output may be stuck high). Shutting off CO2.", bps), true);
                    }
                    */
                }
            }

        }        
          
};