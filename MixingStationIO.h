

// *** TODO *** PUSHBUTTON_UPPER and changin 3 different bits in sensor test mode
// *** TODO *** Remove code that publishes a debug message before system reset









// TODO: consider trying to reconnect on digitalRead() and digitalWrite(), etc

// NOTE: don't use mcp.[function] without wrapping in WITH_LOCK(Wire) { ... }




class MixingStationIO : public IOExpansionBoard {

    #define ACTIVE_LOW LOW
    #define INACTIVE_HIGH HIGH

    private:
        LCDUniversal *lcd;
        RelayModule *relayModule;
        Adafruit_MCP23017 *mcp2;
        void setPinModes() { // implement virtual base class
            mcp.pinMode(MixingStationIO::Components::INTERNAL_TEST_GPIO, OUTPUT);
            mcp.digitalWrite(MixingStationIO::Components::INTERNAL_TEST_GPIO, LOW);
            
            mcp.pinMode(MixingStationIO::Components::TEST_LED, OUTPUT);
            mcp.digitalWrite(MixingStationIO::Components::TEST_LED, HIGH);
            
            mcp.pinMode(MixingStationIO::Components::DOSING_PUMP, OUTPUT);
            mcp.digitalWrite(MixingStationIO::Components::DOSING_PUMP, LOW);
            
            mcp.pinMode(MixingStationIO::Components::DRAIN_PUMP, OUTPUT);
            mcp.digitalWrite(MixingStationIO::Components::DRAIN_PUMP, LOW);
            
            mcp.pinMode(MixingStationIO::Components::TMP_OUTPUT_3, OUTPUT);
            mcp.digitalWrite(MixingStationIO::Components::TMP_OUTPUT_3, LOW);
            
            mcp.pinMode(MixingStationIO::Components::RODI_SOLENOID, OUTPUT);
            mcp.digitalWrite(MixingStationIO::Components::RODI_SOLENOID, LOW);
            
            mcp.pinMode(MixingStationIO::Components::TAP_WATER_SOLENOID, OUTPUT);
            mcp.digitalWrite(MixingStationIO::Components::TAP_WATER_SOLENOID, LOW);
            
            mcp.pinMode(MixingStationIO::Components::MAIN_PUMP, OUTPUT);
            mcp.digitalWrite(MixingStationIO::Components::MAIN_PUMP, LOW);
            
            // inputs
            
            mcp.pinMode(MixingStationIO::Components::MAIN_PUMP_WATER_SENSOR, INPUT);
            mcp.pullUp(MixingStationIO::Components::MAIN_PUMP_WATER_SENSOR, HIGH);  // turn on a 100K pullup internally

            mcp.pinMode(MixingStationIO::Components::CALIBRATION_WATER_SENSOR, INPUT);
            mcp.pullUp(MixingStationIO::Components::CALIBRATION_WATER_SENSOR, HIGH);  // turn on a 100K pullup internally

            mcp.pinMode(MixingStationIO::Components::DOSING_PUMP_SENSOR, INPUT);
            mcp.pullUp(MixingStationIO::Components::DOSING_PUMP_SENSOR, HIGH);  // turn on a 100K pullup internally

            mcp.pinMode(MixingStationIO::Components::FLOW_METER_TAP_WATER, INPUT);
            mcp.pullUp(MixingStationIO::Components::FLOW_METER_TAP_WATER, LOW); // ???

            mcp.pinMode(MixingStationIO::Components::FLOW_METER_RODI_WATER, INPUT);
            mcp.pullUp(MixingStationIO::Components::FLOW_METER_RODI_WATER, HIGH);  // turn on a 100K pullup internally

            mcp.pinMode(MixingStationIO::Components::DRAIN_PUMP_SENSOR, INPUT);
            mcp.pullUp(MixingStationIO::Components::DRAIN_PUMP_SENSOR, HIGH);  // turn on a 100K pullup internally
            
            mcp.pinMode(MixingStationIO::Components::PUSHBUTTON_UPPER, INPUT);
            mcp.pullUp(MixingStationIO::Components::PUSHBUTTON_UPPER, HIGH);  // turn on a 100K pullup internally
            
            mcp.pinMode(MixingStationIO::Components::PUSHBUTTON_LOWER, INPUT);
            mcp.pullUp(MixingStationIO::Components::PUSHBUTTON_LOWER, HIGH);  // turn on a 100K pullup internally            
        }

    public:
    
        typedef enum Components { // and corresponding GPIO expansion pin numbers
            // "For the MCP23017 you specify a pin number from # 0 to 15 for the GPIOA0...GPIOA7, GPIOB0...GPIOB7 pins (i.e. pin 12 is GPIOB4)."
            // I.e. Pin number 0..7 corresponds to A0..A7, and 8..15 corresponds to B0..B7
            TEST_LED = 0, //A0
            INTERNAL_TEST_GPIO = 1, //A1
            DOSING_PUMP = 7, //A7
            DRAIN_PUMP = 4, //A4 (double current: 1000mA)
            RODI_SOLENOID = 6, //A6
            TAP_WATER_SOLENOID = 5, //A5
            MAIN_PUMP = 2, // A2 (power MOSFET)
            TMP_OUTPUT_3 = 3, //A3 (double current: 1000mA)
            // note: inputs all set to have pull-up resistors
            CALIBRATION_WATER_SENSOR = 12, //B4 // note: used to be UPPER_WATER_SENSOR // NOTE: sensor is calibrated to CALIBRATION_SENSOR_GALLONS
            MAIN_PUMP_WATER_SENSOR = 11, //B3 // note: used to be LOWER_WATER_SENSOR
            DOSING_PUMP_SENSOR = 10, //B2 
            DRAIN_PUMP_SENSOR = 9, //B1
            FLOW_METER_TAP_WATER = 8, //B0
            FLOW_METER_RODI_WATER = 13, //B5
            PUSHBUTTON_UPPER = 15, //B7 // TODO: why does this change 3 different bits in sensor test mode??
            PUSHBUTTON_LOWER = 14, //B6
        };
        
        typedef enum WaterTypes {
            TAP = 0,
            RODI
        };
        
        system_tick_t waterLastPumpedToAquarium = 0;
        
        #define TANK_WATER_CHANGE_GALLONS               4.25 //5.25 // Note: Current tank is approx 11.5" x 23", with an average depth of 3.5" between liquid level sensors. That equals 4 gallons, so we'll plan for 5.25 to be safe
        #define TANK_WATER_CHANGE_SIPHON_GALLONS        3.75 // testing
        #define CALIBRATION_SENSOR_GALLONS              1.0 // gallons of water the calibration sensor is positioned to detect
        #define MAIN_PUMP_TIMEOUT_SEC                   15*60 // TODO: choose actual values here
        #define TAP_WATER_FILL_TIMEOUT_SEC              8*60 // TODO: choose actual values here
        #define RODI_WATER_FILL_TIMEOUT_SEC             20*60 // TODO: choose actual values here (note: 15min cuts it very close)
        //#define TAP_WATER_SENSOR                        MixingStationIO::Components::LOWER_WATER_SENSOR
        //#define RODI_WATER_SENSOR                       MixingStationIO::Components::UPPER_WATER_SENSOR
        #define MAX_DRAIN_TIME_SECONDS                  20*60 // takes about 20 minutes to drain 8 gallons
        //#define TOTAL_GALLONS_FOR_SEACHEM_PRIME_CALC    10 // TODO: choose actual value // used to calculate Seachem Prime dosage
        #define DOSING_PUMP_ML_PER_SEC                  1.18 // calibrate using dosePrime() with overrideDosageTimeSeconds parameter (tested 0.85s to dispense 1mL)
        #define DOSING_PUMP_MAX_TIME_SEC                15 //10
        #define TAP_WATER_PULSES_PER_GALLON             14272 // @ 0.41GPM //14534 // @ 0.42GPH //15326 // @ 0.44 GPM; 13726 // @ 0.49 GPM; was 14254 before 1/21/24 // // @ 0.48 GPM //14711 // @0.48 GPM. was: 13703 // Average from measurements via dispenseWaterCalibrationMode(): 13893 @ 0.50 GPM, 13542 @ 0.15 GPM, 13674 @ 0.51 GPM
        #define RODI_WATER_PULSES_PER_GALLON            10633 //@ 1.25GPM //10761 // @ 0.95GPM //11286 // @ 1.13 GPM // 10280 // @ 0.95 GPM; was 10637 before 1/21/24 // @ 1.03 GPM //8953  // Average from measurements via dispenseWaterCalibrationMode(): 8968 @ 1.18 GPM, 8892 @ 1.18 GPM, 8998 @ 1.05 GPM
        #define RODI_TO_TAP_TARGET_RATIO                4.23 // see measurements below as well as google doc notes/calculations
        
        MixingStationIO(int disablePin, bool resetPinLogicLevel, int gpioTestPin, byte i2cAddressAddition, LCDUniversal *lcd_, RelayModule *relayModule_, Adafruit_MCP23017 *mcp2_) : IOExpansionBoard(disablePin, resetPinLogicLevel, gpioTestPin, i2cAddressAddition) { //IOExpansionBoard(int disablePin, bool resetPinLogicLevel, int gpioTestPin, byte i2cAddressAddition=0x00) {
            lcd = lcd_;
            relayModule = relayModule_;
            mcp2 = mcp2_;
        }
        
        /*
        void dispenseWaterAndTestFlowMeter() { // use this function to dispense a set volume of water using a flow meter and check its accuracy
            const int pulsesPerGallon = 8400; // rodi // tap: 16150;//15384; // measured x pulses to fill 16oz
            //const int pulsesPerGallon = 14160; // tap // measured x pulses to fill 16oz
            bool cancelledViaButtonPress;
            unsigned long debug_numInvalidReadings;
            dispenseWater(0.0625, 0.0625, cancelledViaButtonPress, debug_numInvalidReadings);
        }
        */
        


        // testing: overload used to calibrate based on 1-gallon liquid level sensor. returns number of pulses until liquid level sensor is tripped 
        bool dispenseWaterCalibrationMode(bool useTapInsteadOfRodi) {
            float tapWaterGallons = useTapInsteadOfRodi ? 1.5 : 0; // extra gallons just as buffer
            float rodiWaterGallons = useTapInsteadOfRodi ? 0 : 1.5;
            bool cancelledViaButtonPress = false;
            unsigned long debug_numInvalidReadings = 0;
            unsigned long calibrationPulses = 0;
            float calibrationGPM = 0;
            
            drainReservoir();
            
            Particle.publish("dispenseWaterCalibrationMode", String::format("Running water flow meter calibration for %s...", useTapInsteadOfRodi ? "tap water" : "RODI water"));
            bool success = dispenseWater(tapWaterGallons, rodiWaterGallons, true, calibrationPulses, calibrationGPM, cancelledViaButtonPress, debug_numInvalidReadings);
            
            bool totalSuccess = true;
            if (success && !cancelledViaButtonPress && debug_numInvalidReadings == 0) {
                PushNotification::send(String::format("%s flow meter calibration complete. calibrationModePulses: %u, calibrationGPM: %.2f", useTapInsteadOfRodi ? "Tap water" : "RODI water", calibrationPulses, calibrationGPM));
            } else {
                totalSuccess = false;
                PushNotification::send(String::format("%s calibration failed. Flags: success: %d, cancelledViaButtonPress: %d, debug_numInvalidReadings: %u, calibrationPulses: %u", 
                    useTapInsteadOfRodi ? "Tap water" : "RODI water", success, cancelledViaButtonPress, debug_numInvalidReadings, calibrationPulses));
                //Particle.publish("dispenseWaterCalibrationMode", String::format("Error running calibration. success: %d, cancelledViaButtonPress: %d, debug_numInvalidReadings: %u, calibrationPulses: %u", 
                    //success, cancelledViaButtonPress, debug_numInvalidReadings, calibrationPulses));
            }
            
            drainReservoir();
            PushNotification::send("Calibration draining complete");
            return totalSuccess;
        }
        
        
        
        bool dispenseWater(float totalMixedGallons, bool &cancelledViaButtonPress, unsigned long &debug_numInvalidReadings) {
            // ex: target of 10 gal --> rodi target of 8.09, tap target of 1.91
            float rodiWaterGallons = (totalMixedGallons * RODI_TO_TAP_TARGET_RATIO) / (RODI_TO_TAP_TARGET_RATIO + 1);
            float tapWaterGallons = totalMixedGallons - rodiWaterGallons;
            
            Particle.publish("dispenseWater()", String::format("totalMixedGallons: %.2f, tapWaterGallons: %.2f, rodiWaterGallons %.2f, rodiWaterGallons/tapWaterGallons: %.2f",
                totalMixedGallons, tapWaterGallons, rodiWaterGallons, rodiWaterGallons/tapWaterGallons));
            
            // sanity check
            if (abs(1 - (rodiWaterGallons/tapWaterGallons)/RODI_TO_TAP_TARGET_RATIO) > 0.01) {
                PushNotification::send(String::format("debug error: rodiWaterGallons/tapWaterGallons != RODI_TO_TAP_TARGET_RATIO. rodiWaterGallons: %.2f, tapWaterGallons: %.2f", rodiWaterGallons, tapWaterGallons), true);
                return false;
            }

            return dispenseWater(tapWaterGallons, rodiWaterGallons, cancelledViaButtonPress, debug_numInvalidReadings);
        }

//**TODO** implement handling if this returns false

        // overload
        bool dispenseWater(float tapWaterGallons, float rodiWaterGallons, bool &cancelledViaButtonPress, unsigned long &debug_numInvalidReadings) {
            unsigned long calibrationModePulses = 0; // dummy
            float calibrationModeGPM = 0; // dummy
            return dispenseWater(tapWaterGallons, rodiWaterGallons, false, calibrationModePulses, calibrationModeGPM, cancelledViaButtonPress, debug_numInvalidReadings);
        }

        bool dispenseWater(float tapWaterGallons, float rodiWaterGallons, bool isCalibrationMode, unsigned long &calibrationModePulses, float &calibrationModeGPM, bool &cancelledViaButtonPress, unsigned long &debug_numInvalidReadings) {

            // note: the mixing tank must be drained prior to calling this method to ensure accurate filling

            const unsigned long tap_targetPulses = (int)roundf(TAP_WATER_PULSES_PER_GALLON*tapWaterGallons);
            unsigned long tap_pulseCount = 0;
            unsigned long tap_pulseCountSinceLastGap = 0;
            bool tap_lastFlowMeterValue;

            const unsigned long rodi_targetPulses = (int)roundf(RODI_WATER_PULSES_PER_GALLON*rodiWaterGallons);
            unsigned long rodi_pulseCount = 0;
            unsigned long rodi_pulseCountSinceLastGap = 0;
            bool rodi_lastFlowMeterValue;

            cancelledViaButtonPress = false;
            debug_numInvalidReadings = 0;
            
            uint16_t gpioValues;
            unsigned long elapsedTimeNotCountingPulsesMs = 0;
            system_tick_t lastDisplayUpdateStart = 0;
            system_tick_t countExtraPulsesUntilTimestamp = 0;
            unsigned long tapTotalFillTimeMs = 0;
            unsigned long rodiTotalFillTimeMs = 0;
            
            if (isCalibrationMode) {
                // drain again to ensure we're starting without any liquid
                lcd->display("Draining", "again...", 1);
                drainReservoir();
                if (this->digitalReadOrReset(MixingStationIO::Components::CALIBRATION_WATER_SENSOR) == ACTIVE_LOW) {
                    PushNotification::send("Debug: Water detected in mixing tank after draining. Must drain completely before running calibration.");
                    calibrationModePulses = 0;
                    calibrationModeGPM = 0;
                    return false;
                }
            }
            
            if (tapWaterGallons + rodiWaterGallons > 15) {
                PushNotification::send("Error: Requested volume too large.");
                return false;
            }
            
            lcd->display("Dispensing", "water...", 1);

            if (tapWaterGallons > 0) {
                digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, HIGH);
            }
            if (rodiWaterGallons > 0) {
                digitalWriteOrReset(MixingStationIO::Components::RODI_SOLENOID, HIGH);
            }
            
            system_tick_t start = millis();
            system_tick_t fillingComplete = 0;
            bool error = false;
            bool calibrationSensorTripped = false;

            //while ((tap_pulseCount + tap_pulseCountSinceLastGap < tap_targetPulses) || (rodi_pulseCount + rodi_pulseCountSinceLastGap < rodi_targetPulses)) {
            while (fillingComplete == 0 || millis() - fillingComplete < 3*1000) {
                // safety timer in case a flow meter is disconnected
                if (((tap_pulseCount == 0 && tap_targetPulses != 0) || (rodi_pulseCount == 0 && rodi_targetPulses != 0)) && (millis() - start > 3 * 1000)) {
                    error = true;
                    break;
                }
                
                // safety timer in case this goes on for too long
                if (millis() - start > 20*60*1000) {
                    error = true;
                    break;
                }
                
                // check to see if filling is complete. if so, wait a bit longer to count any final flow meter pulses
                if (fillingComplete == 0 && (tap_pulseCount + tap_pulseCountSinceLastGap >= tap_targetPulses) && (rodi_pulseCount + rodi_pulseCountSinceLastGap >= rodi_targetPulses)) {
                    fillingComplete = millis();
                }
                
                // read values and validate using known entities
                gpioValues = readGPIOAB();
                if ((bitRead(gpioValues, MixingStationIO::Components::INTERNAL_TEST_GPIO) == HIGH)) { // update: we now wait a bit after both solenoids are turned off, so this last part doesn't make sense anymore:  || ((bitRead(gpioValues, MixingStationIO::Components::TAP_WATER_SOLENOID) == LOW) && (bitRead(gpioValues, MixingStationIO::Components::RODI_SOLENOID) == LOW))) {
                    debug_numInvalidReadings++;
                    continue;
                }

                // cancel if button pressed
                if ((bitRead(gpioValues, MixingStationIO::Components::PUSHBUTTON_UPPER) == LOW) || (bitRead(gpioValues, MixingStationIO::Components::PUSHBUTTON_LOWER) == LOW)) {
                    cancelledViaButtonPress = true;
                    break;
                }

                // count flow meter pulses

                bool tap_flowMeterValue = bitRead(gpioValues, MixingStationIO::Components::FLOW_METER_TAP_WATER);
                if (tap_flowMeterValue != tap_lastFlowMeterValue) {
                    tap_pulseCount++;
                    tap_lastFlowMeterValue = tap_flowMeterValue;
                    if ((countExtraPulsesUntilTimestamp != 0) && (millis() <= countExtraPulsesUntilTimestamp)) {
                        tap_pulseCountSinceLastGap++;
                    }
                    if ((tap_pulseCount + tap_pulseCountSinceLastGap >= tap_targetPulses) && (bitRead(gpioValues, MixingStationIO::Components::TAP_WATER_SOLENOID) == HIGH)) {
                        digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, LOW); // turn off when full
                        tapTotalFillTimeMs = millis() - start;
                    }
                }

                bool rodi_flowMeterValue = bitRead(gpioValues, MixingStationIO::Components::FLOW_METER_RODI_WATER);
                if (rodi_flowMeterValue != rodi_lastFlowMeterValue) {
                    rodi_pulseCount++;
                    rodi_lastFlowMeterValue = rodi_flowMeterValue;
                    if ((countExtraPulsesUntilTimestamp != 0) && (millis() <= countExtraPulsesUntilTimestamp)) {
                        rodi_pulseCountSinceLastGap++;
                    }
                    if ((rodi_pulseCount + rodi_pulseCountSinceLastGap >= rodi_targetPulses) && (bitRead(gpioValues, MixingStationIO::Components::RODI_SOLENOID) == HIGH)) {
                        digitalWriteOrReset(MixingStationIO::Components::RODI_SOLENOID, LOW); // turn off when full
                        rodiTotalFillTimeMs = millis() - start;
                    }
                }
                
                // TODO: if the water level just tripped the calibration sensor (positioned at CALIBRATION_SENSOR_GALLONS gallons), make sure we're within a reasonable accuracy threshold
                if (!calibrationSensorTripped && (bitRead(gpioValues, MixingStationIO::Components::CALIBRATION_WATER_SENSOR) == ACTIVE_LOW)) {
                    calibrationSensorTripped = true;

                    // if we're in calibration mode, see if our one-gallon calibration sensor has been triggered
                    if (isCalibrationMode) {
                        // turn off solenoids
                        digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, LOW);
                        digitalWriteOrReset(MixingStationIO::Components::RODI_SOLENOID, LOW);
                        // if we happen to be in "catchup" mode, our pulse counting won't be accurate -- just fail the test so it can be retried
                        if (countExtraPulsesUntilTimestamp != 0) {
                            PushNotification::send("Debug: Water calibration sensor tripped, but while pulse counting was in 'catchup' mode. Try running calibration againn.");
                            calibrationModePulses = 0;
                            calibrationModeGPM = 0;
                            break;                       
                        } else {
                            if (tapWaterGallons != 0) {
                                calibrationModePulses = tap_pulseCount + tap_pulseCountSinceLastGap;
                            } else {
                                calibrationModePulses = rodi_pulseCount + rodi_pulseCountSinceLastGap;
                            }
                            calibrationModeGPM = CALIBRATION_SENSOR_GALLONS / ((millis() - start)/1000/60.0);
                            //PushNotification::send(String::format("Water flow meter calibration complete. calibrationModePulses: %u, calibrationModeGPM: %.2f", calibrationModePulses, calibrationModeGPM));
                            break;
                        }
                    } else {
                        // TODO: what about case where we're in "catchup" mode? (countExtraPulsesUntilTimestamp != 0)

                        float expectedRODIGallons = (CALIBRATION_SENSOR_GALLONS * RODI_TO_TAP_TARGET_RATIO) / (RODI_TO_TAP_TARGET_RATIO + 1);
                        float expectedTapGallons = CALIBRATION_SENSOR_GALLONS - expectedRODIGallons;                        
                        
                        
                        //unsigned int expectedPulses = (float)CALIBRATION_SENSOR_GALLONS * (TAP_WATER_PULSES_PER_GALLON + RODI_WATER_PULSES_PER_GALLON);
                        unsigned int expectedPulses = expectedRODIGallons*RODI_WATER_PULSES_PER_GALLON + expectedTapGallons*TAP_WATER_PULSES_PER_GALLON;
                        
                        
                        
                        
                        
                        unsigned int actualPulses = tap_pulseCount + tap_pulseCountSinceLastGap + rodi_pulseCount + rodi_pulseCountSinceLastGap;
                        float errorPct = (float)actualPulses / expectedPulses;
                        float avgGPM = CALIBRATION_SENSOR_GALLONS / ((millis() - start)/1000/60.0);
    
                        if (abs(1 - errorPct) > 0.075) {
                            PushNotification::send(String::format("Error: Mixing tank water volume beyond error threshold. countExtraPulsesUntilTimestamp: %u, expectedPulses: %u, actualPulses: %u, errorPct: %.2f%%, avgGPM: %.2f",
                                countExtraPulsesUntilTimestamp, expectedPulses, actualPulses, (1-errorPct)*100, avgGPM));
                            error = true;
                            break;
                        } else {
                            static particle::Future<bool> publishFuture = PushNotification::send(String::format("Debug: Mixing tank water volume check: countExtraPulsesUntilTimestamp: %u, expectedPulses: %u, actualPulses: %u, errorPct: %.2f%%, avgGPM: %.2f",
                                countExtraPulsesUntilTimestamp, expectedPulses, actualPulses, (1-errorPct)*100, avgGPM));
                        }                        
                    }
                }

                // if we're not playing catchup counting pulses, periodically update the display as well as call Particle.publish()
                if (countExtraPulsesUntilTimestamp == 0) {
                    // periodicaly update display and call Particle.process()
                    if ((millis() - lastDisplayUpdateStart > 1000) || ((tap_pulseCount + tap_pulseCountSinceLastGap >= tap_targetPulses) && (rodi_pulseCount + rodi_pulseCountSinceLastGap > rodi_targetPulses))) { // force display when we hit 100%
                        lastDisplayUpdateStart = millis();

                        // take a break from pulse counting to attend to the display and Particle.process()
                        lcd->setCursor(0,0); // ~12ms
                        lcd->send_string(String::format("Tap water: %d%%  ", tap_targetPulses == 0 ? 0 : (int)roundf(100*(tap_pulseCount + tap_pulseCountSinceLastGap)/(float)tap_targetPulses))); // TODO: try minimizing length of string to speed this up
                        lcd->setCursor(0,1); // ~12ms
                        lcd->send_string(String::format("RODI water: %d%%  ", rodi_targetPulses == 0 ? 0 : (int)roundf(100*(rodi_pulseCount + rodi_pulseCountSinceLastGap)/(float)rodi_targetPulses))); // TODO: try minimizing length of string to speed this up
                        Particle.process(); 

                        // make up for lost time by double-counting pulses for however long it took to update the display/etc (since we assume flow rates are basically constant within this time frame)
                        unsigned long elapsedTime = (millis() - lastDisplayUpdateStart);
                        elapsedTimeNotCountingPulsesMs += elapsedTime;
                        countExtraPulsesUntilTimestamp = millis() + elapsedTime; // we'll double-count pulses until we hit this timestamp 

                        // new: safety check if we've filled CALIBRATION_SENSOR_GALLONS*2 but haven't tripped the sensor, something's wrong
                        if (!calibrationSensorTripped && 
                            ((tap_pulseCount + tap_pulseCountSinceLastGap)/(float)TAP_WATER_PULSES_PER_GALLON + (rodi_pulseCount + rodi_pulseCountSinceLastGap)/(float)RODI_WATER_PULSES_PER_GALLON) > CALIBRATION_SENSOR_GALLONS*2) {
                            static particle::Future<bool> publishFuture = PushNotification::send("Debug: Too much water filled without tripping calibration level sensor.");
                            error = true;
                            break;
                        }
                    } 
                } else if (millis() > countExtraPulsesUntilTimestamp) {
                    countExtraPulsesUntilTimestamp = 0; // reset flag if we're past the cutoff timestamp
                }

            };
            
            unsigned long elapsedTimeMs = millis() - start;
            
            // safety check
            digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, LOW);
            digitalWriteOrReset(MixingStationIO::Components::RODI_SOLENOID, LOW);
            
            if (isCalibrationMode) {
                return (calibrationModePulses != 0);
            }
            
            float actualTapWaterGallons = (tap_pulseCount + tap_pulseCountSinceLastGap) / (float)TAP_WATER_PULSES_PER_GALLON;
            float avgTapWaterGPM = actualTapWaterGallons/((float)tapTotalFillTimeMs/1000/60.0);
            float tapWaterAccouracyPct = actualTapWaterGallons / tapWaterGallons;
            
            float actualRodiWaterGallons = (rodi_pulseCount + rodi_pulseCountSinceLastGap) / (float)RODI_WATER_PULSES_PER_GALLON;
            float avgRodiWaterGPM = actualRodiWaterGallons/((float)rodiTotalFillTimeMs/1000/60.0);
            float rodiWaterAccouracyPct = actualRodiWaterGallons / rodiWaterGallons;
            
            PushNotification::send(String::format("Finished filling mixing tank in %.2f minutes. Actuals: Tap water: %.2f gallons (%.2f%% of target, %.2f GPM avg). RODI: %.2f gallons (%.2f%% of target, %.2f GPM avg)",
                elapsedTimeMs/1000/60.0, actualTapWaterGallons, tapWaterAccouracyPct*100, avgTapWaterGPM, actualRodiWaterGallons, rodiWaterAccouracyPct*100, avgRodiWaterGPM
            ));
            
            Particle.publish("debug", String::format("abs(1 - rodiWaterAccouracyPct): %.2f, abs(1 - tapWaterAccouracyPct) %.2f", 
                abs(1 - rodiWaterAccouracyPct), abs(1 - tapWaterAccouracyPct)
            ));
            
            if (error) {
                PushNotification::send(String::format("ERROR: dispenseWater() error flag set to true. elapsedTimeMs: %u, tap_pulseCount: %u, rodi_pulseCount: %u",
                    elapsedTimeMs, tap_pulseCount, rodi_pulseCount));
                return false;
            }
            
            if (abs(1 - rodiWaterAccouracyPct) > 0.05 || abs(1 - tapWaterAccouracyPct) > 0.05) {
                PushNotification::send(String::format("Warning: Water ratio is past error threshold. Returning false. rodiWaterAccouracyPct: %.2f%%, tapWaterAccouracyPct: %.2f%%", 
                    rodiWaterAccouracyPct*100, tapWaterAccouracyPct*100));
                return false;
            }
            
            if (debug_numInvalidReadings > 0) {
                PushNotification::send(String::format("Warning: debug_numInvalidReadings was %u in dispenseWater()", debug_numInvalidReadings), true);
                return false;
            }
            
            /*
            Particle.publish("debug: tap", String::format("pulseCount: %u, pulseCountSinceLastGap: %u, targetPulses: %u",
                tap_pulseCount, tap_pulseCountSinceLastGap, tap_targetPulses));
            Particle.publish("debug: RODI", String::format("pulseCount: %u, pulseCountSinceLastGap: %u, targetPulses: %u",
                rodi_pulseCount, rodi_pulseCountSinceLastGap, rodi_targetPulses));
            Particle.publish("debug: other", String::format("elapsedTimeNotCountingPulsesMs: %ums (~~%.2f%%), elapsedTimeMs (max of both pumps): %ums, invalidReadingCount: %u",
                elapsedTimeNotCountingPulsesMs, 100*((float)elapsedTimeNotCountingPulsesMs/(elapsedTimeNotCountingPulsesMs+elapsedTimeMs)), elapsedTimeMs, debug_numInvalidReadings));
            */
            
            return true;
        }
        
        /*
        void temp_measureWaterAtBothSensors() {
            // results: 
            //  - lower sensor: solenoid: 5, lowerButtonPressed: 0, pulseCount: 10387, pulse counting time: 164391ms; elapsedTimeMs: 170546ms, elapsedTimeNotCountingPulsesMs: 6155ms, invalidReadingCount: 0
            //  - upper sensor: solenoid: 6, lowerButtonPressed: 0, pulseCount: 25934, pulse counting time: 443228ms; elapsedTimeMs: 460435ms, elapsedTimeNotCountingPulsesMs: 17207ms, invalidReadingCount: 0
            //  - TDS meter: 59ppm
            //  - given TAP_WATER_PULSES_PER_GALLON of 14160 --> 10387 pulses = 0.73 gallons
            //  - given RODI_WATER_PULSES_PER_GALLON of 8400 --> 25934 pulses = 3.09 gallons
            //  - ratio of rodi:tap --> 4.23 (vs calculated target amount in google doc of 4.33)

            const Components solenoid = MixingStationIO::Components::TAP_WATER_SOLENOID; //MixingStationIO::Components::TAP_WATER_SOLENOID;
            const Components flowMeter = MixingStationIO::Components::FLOW_METER_TAP_WATER; //MixingStationIO::Components::FLOW_METER_TAP_WATER;
            const Components sensorToReach = MixingStationIO::Components::UPPER_WATER_SENSOR;
            
            temp_dispenseWaterAndCountPulses2(
                MixingStationIO::Components::TAP_WATER_SOLENOID,
                MixingStationIO::Components::FLOW_METER_TAP_WATER,
                MixingStationIO::Components::LOWER_WATER_SENSOR
            );
            PushNotification::send("1 complete");

            temp_dispenseWaterAndCountPulses2(
                MixingStationIO::Components::RODI_SOLENOID,
                MixingStationIO::Components::FLOW_METER_RODI_WATER,
                MixingStationIO::Components::UPPER_WATER_SENSOR
            );
            PushNotification::send("2 complete");
        }
        */
        
        // alt version for counting pulses to reach a given liquid sensor
        void temp_dispenseWaterAndCountPulses2(Components solenoid, Components flowMeter, Components sensorToReach) { // use this function to estimate how many pulses it takes to fill a given volume of water, e.g. 1923 pulses to dispense 16oz of water
            uint16_t gpioValues;
            bool isValidReading;
            bool lowerButtonPressed = false;
            bool waterToggledOn = false;
            unsigned long invalidReadingCount = 0;
            system_tick_t start = millis();
            system_tick_t upperButtonLastPressed = 0;

            bool lastFlowMeterValue = false;
            unsigned long pulseCount = 0;
            unsigned long elapsedTimeMs = 0;
            system_tick_t lastDisplayUpdate = 0;
            unsigned long elapsedTimeNotCountingPulsesMs = 0;
            system_tick_t particleProcessLastCalled = 0;

            bool sensorReached = false;
            
            digitalWriteOrReset(solenoid, HIGH);
            
            while (!sensorReached && !lowerButtonPressed) {
                // read values and validate using known entities
                gpioValues = readGPIOAB();
                bool testGpioValue = bitRead(gpioValues, MixingStationIO::Components::INTERNAL_TEST_GPIO);
                bool drainPumpValue = bitRead(gpioValues, MixingStationIO::Components::DRAIN_PUMP);
                //bool tapWaterSolenoidValue = bitRead(gpioValues, solenoid);
                isValidReading = (!testGpioValue && !drainPumpValue);// && tapWaterSolenoidValue);
                if (!isValidReading) {                
                    invalidReadingCount++;
                    continue;
                }

                // check for stop condition
                sensorReached = (bitRead(gpioValues, sensorToReach) == ACTIVE_LOW);
                lowerButtonPressed = (bitRead(gpioValues, MixingStationIO::Components::PUSHBUTTON_LOWER) == ACTIVE_LOW);

                // count flow meter pulses
                bool flowMeterValue = bitRead(gpioValues, flowMeter);
                
                // **NEW** count rising edges only (vs any edge)
                if (flowMeterValue != lastFlowMeterValue) {
                    if (flowMeterValue) { // rising edge
                        pulseCount++;
                    }
                    lastFlowMeterValue = flowMeterValue;
                }

                // periodicaly update display
                if (millis() - lastDisplayUpdate > 1000) {
                    lastDisplayUpdate = millis();
                    system_tick_t updateStart = millis();
                    lcd->setCursor(0,0); // ~12ms
                    lcd->send_string(String::format("Pulses: %d   ", pulseCount));
                    elapsedTimeNotCountingPulsesMs += (millis() - updateStart);
                }
                
                // TODO: call Particle.process() periodically?
                if (millis() - particleProcessLastCalled > 1000) { // ??
                    system_tick_t processStart = millis();
                    Particle.process(); 
                    elapsedTimeNotCountingPulsesMs += (millis() - processStart);
                    particleProcessLastCalled = millis();
                } 
            };
            
            if (start != 0) {
                 elapsedTimeMs += (millis() - start); // add any remaining time
            }
            
            digitalWriteOrReset(solenoid, LOW);

            Particle.publish("readings", String::format("solenoid: %d, lowerButtonPressed: %d, pulseCount: %u, pulse counting time: %ums; elapsedTimeMs: %ums, elapsedTimeNotCountingPulsesMs: %ums, invalidReadingCount: %u", 
                solenoid, lowerButtonPressed, pulseCount, elapsedTimeMs - elapsedTimeNotCountingPulsesMs, elapsedTimeMs, elapsedTimeNotCountingPulsesMs, invalidReadingCount));
            
            lcd->display(
                String::format("pulseCount: %u", pulseCount),
                String::format("elapsedTimeMs: %ums", elapsedTimeMs),
                3
            );
        }
        
        void dispenseWaterAndCountPulses() { // use this function to estimate how many pulses it takes to fill a given volume of water, e.g. 1923 pulses to dispense 16oz of water
            
            const Components solenoid = MixingStationIO::Components::TAP_WATER_SOLENOID; //MixingStationIO::Components::TAP_WATER_SOLENOID;
            const Components flowMeter = MixingStationIO::Components::FLOW_METER_TAP_WATER; //MixingStationIO::Components::FLOW_METER_TAP_WATER;
            
            /** Notes on performance (after removing delays from waveshare library):
             *  - lcd->setCursor(0,0) takes ~2ms
             *  - lcd->send_string(String::format("%.2f", 123.45)) takes ~14ms
             *  - lcd->send_string(String::format("test: %.2f", 123.45)) takes ~27ms
             *  - lcd->send_string(String::format("test longer: %.2f", 123.45)); takes ~42ms
             *  - lcd->send_string(String::format("test: %d", 123)) takes ~20ms
             *  - lcd->send_string(String::format("test longer: %d", 123)); takes ~35ms
             *  - String::format() is faster than String(), at least when tested with float values (~10ms speedup)
             *  - readGPIOAB(); took ~600uS
             */
             
            lcd->display("Btn 1: Toggle", "Btn 2: Finish", 1);
            
            uint16_t gpioValues;
            bool isValidReading;
            bool lowerButtonPressed = false;
            bool waterToggledOn = false;
            unsigned long invalidReadingCount = 0;
            system_tick_t start = millis();
            system_tick_t upperButtonLastPressed = 0;

            bool lastFlowMeterValue = false;
            unsigned long pulseCount = 0;
            unsigned long elapsedTimeMs = 0;
            system_tick_t lastDisplayUpdate = 0;
            unsigned long elapsedTimeNotCountingPulsesMs = 0;
            system_tick_t particleProcessLastCalled = 0;

            do {
                // read values and validate using known entities
                gpioValues = readGPIOAB();
                bool testGpioValue = bitRead(gpioValues, MixingStationIO::Components::INTERNAL_TEST_GPIO);
                bool drainPumpValue = bitRead(gpioValues, MixingStationIO::Components::DRAIN_PUMP);
                //bool tapWaterSolenoidValue = bitRead(gpioValues, solenoid);
                isValidReading = (!testGpioValue && !drainPumpValue);// && tapWaterSolenoidValue);
                if (!isValidReading) {                
                    invalidReadingCount++;
                    continue;
                }

                // check for button presses                

                bool upperButtonPressed = (bitRead(gpioValues, MixingStationIO::Components::PUSHBUTTON_UPPER) == LOW);
                lowerButtonPressed = (bitRead(gpioValues, MixingStationIO::Components::PUSHBUTTON_LOWER) == LOW);
                
                if (upperButtonPressed && (millis() - upperButtonLastPressed > 250)) { // debounce
                    upperButtonLastPressed = millis();
                    if (waterToggledOn) { // if we're toggling water off...
                        elapsedTimeMs += (millis() - start);
                        start = 0; // set to zero as flag we don't have any more to add to elapsedTimeMs
                    } else { // if we're toggling water on...
                        start = millis();
                    }
                    waterToggledOn = !waterToggledOn;
                    digitalWriteOrReset(solenoid, waterToggledOn);
                }


                if (waterToggledOn) {
                    // count flow meter pulses
                    bool flowMeterValue = bitRead(gpioValues, flowMeter);
                    if (flowMeterValue != lastFlowMeterValue) {
                        pulseCount++;
                        lastFlowMeterValue = flowMeterValue;
                    }

                    // periodicaly update display
                    if (millis() - lastDisplayUpdate > 1000) {
                        lastDisplayUpdate = millis();
                        system_tick_t updateStart = millis();
                        lcd->setCursor(0,0); // ~12ms
                        lcd->send_string(String::format("Pulses: %d   ", pulseCount));
                        elapsedTimeNotCountingPulsesMs += (millis() - updateStart);
                    }
                }
                
                // TODO: call Particle.process() periodically?
                if (millis() - particleProcessLastCalled > 1000) { // ??
                    system_tick_t processStart = millis();
                    Particle.process(); 
                    elapsedTimeNotCountingPulsesMs += (millis() - processStart);
                    particleProcessLastCalled = millis();
                } 

            } while (!lowerButtonPressed); // be careful in case isValidReading gets stuck low
            
            if (start != 0) {
                 elapsedTimeMs += (millis() - start); // add any remaining time
            }
            
            digitalWriteOrReset(solenoid, LOW);

            Particle.publish("readings", String::format("pulseCount: %u, pulse counting time: %ums; elapsedTimeMs: %ums, elapsedTimeNotCountingPulsesMs: %ums, invalidReadingCount: %u", 
                pulseCount, elapsedTimeMs - elapsedTimeNotCountingPulsesMs, elapsedTimeMs, elapsedTimeNotCountingPulsesMs, invalidReadingCount));
            
            lcd->display(
                String::format("pulseCount: %u", pulseCount),
                String::format("elapsedTimeMs: %ums", elapsedTimeMs),
                3
            );
        }

        /*
        void testFlowMeter_old() {
            system_tick_t start = millis();
            const system_tick_t DURATION = 3*1000;
            uint16_t gpioValues;
            bool lastFlowMeterValue = false;
            unsigned int pulseCount = 0;
            unsigned int invalidReadingCount = 0;
            system_tick_t particleProcessLastCalled = millis();
            system_tick_t timeSpentInProcess = 0;
            
            // TODO: watchdog timers?
            
            
            digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, HIGH);
            
            do {
                // note: in testing, reading and validating values took ~600uS
                gpioValues = readGPIOAB();
                // validate reading using a mix of values we expect to be both HIGH and LOW
                bool testGpioValue = bitRead(gpioValues, MixingStationIO::Components::INTERNAL_TEST_GPIO);
                bool drainPumpValue = bitRead(gpioValues, MixingStationIO::Components::DRAIN_PUMP);
                bool tapWaterSolenoidValue = bitRead(gpioValues, MixingStationIO::Components::TAP_WATER_SOLENOID);
                bool isValidReading = (!testGpioValue && !drainPumpValue && tapWaterSolenoidValue);
                
                if (isValidReading) {
                    bool flowMeterValue = bitRead(gpioValues, flowMeter);
                    if (flowMeterValue != lastFlowMeterValue) {
                        pulseCount++;
                        lastFlowMeterValue = flowMeterValue;
                    }
                } else {
                    invalidReadingCount++;
                }
                
                if (millis() - particleProcessLastCalled > 500) { // ??
                    system_tick_t processStart = millis();
                    Particle.process(); 
                    timeSpentInProcess += (millis() - processStart);
                    particleProcessLastCalled = millis();
                }
                
                
            } while (millis() - start < DURATION);
            
            digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, LOW);
            
            Particle.publish("debug", String::format("pulseCount: %u, invalidReadingCount: %u, timeSpentInProcess: %ums",
                pulseCount, invalidReadingCount, timeSpentInProcess));
            
        }
        */
        
        // note: this function is unsafe in that you should make sure to check that the aquarium water level sensors definitely work before calling this, to avoid overflowing the tank
        bool pumpWaterToAquarium(int passcode, bool (*isLiquidDetectedTop)()) { 
            
            if (passcode != 123) {
                Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: pumpWaterToAquarium() called without correct passcode""\" }"), PRIVATE);
                return false;
            } else {
                Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Pumping water to aquarium...""\" }"), PRIVATE);
            }

            // make sure we're starting out with water -not- up at the top aquarium water level sensor
            if (isLiquidDetectedTop()) {
                lcd->display("Error: Liauid", "detected top", 3);
                Particle.publish("pumpWaterToAquarium()", "Error: Sensor detected liquid before dosing pump run");
                return false;
            }

            if (relayModule->isCanisterFilterOn()) { // filter may be on or off depending on how we got to this stage
                // turn off canister filter and verify that flow rate is zero
                if (!disableFilterAndHeatWithAlert()) {
                    // note: push alert will already have gone out if we get here
                    return false;
                }
            }

            // ============================================================================================================
            // NOTE: Do not return below this line without first calling enableFilterAndHeatWithAlert() ****
            // ALSO: Do not return without opening up cutoff ball valve
            // ============================================================================================================
            
            // ensure that cutoff electric ball valve is in a closed position
            bool cutoffValveClosed;
            WITH_LOCK(Wire) { 
                cutoffValveClosed = (mcp2->digitalRead(PIN_IOEXP2__CUTOFF_BALL_VALVE) == LOW);
            }
            if (!cutoffValveClosed) {
                lcd->display("Closing cutoff", "valve...");
                WITH_LOCK(Wire) { mcp2->digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, LOW); }
                delay(7*1000);
            }
            
            lcd->display("Pumping water...", "", 0);
            
            unsigned long start = millis();
            unsigned long secondsLastUpdated = 0;
            int tmp_retryAttempts = 0;
            particle::Future<bool> publishFuture; // use to make Publish() asynchronous so it doesn't block (and the pump can't get hung stuck on). details: https://docs.particle.io/firmware/low-power/stop-sleep-cellular/#the-future
            bool error = false;

            this->digitalWriteOrReset(MixingStationIO::Components::MAIN_PUMP, HIGH);
            do {
                // sanity check for connection, in case we have to literally pull the plug while in progress
                if (!isConnected(true)) { // set autoRepair flag, since pumping seems to fail a lot due to blips here
                    delay(10);
                    if (!isConnected()) {
                        lcd->display("Error: IO board", "disconnected", 3, true);
                        publishFuture = Particle.publish("pumpWaterToAquarium()", String::format("Error: IO board disconnected during pump operation"));
                        error = true;
                        break;
                    } else {
                        publishFuture = Particle.publish("pumpWaterToAquarium()", String::format("Error: IO board disconnected during pump operation, but reconnected afterwards"));
                    }
                }

                if (millis() - start >= (MAIN_PUMP_TIMEOUT_SEC*1000)) {
                    lcd->display("Error: Timed out", String::format("after %ums", millis() - start), 3);
                    publishFuture = Particle.publish("pumpWaterToAquarium()", String::format("Error: Main pump timed out after %d ms", MAIN_PUMP_TIMEOUT_SEC*1000));
                    error = true;
                    break;
                }
                
                // new: debugging issue where pump just stops running
                if (this->digitalReadOrReset(MixingStationIO::Components::MAIN_PUMP) != HIGH) {
                    lcd->display("IO error:", "Mismatch", 1);
                    publishFuture = Particle.publish("pumpWaterToAquarium()", "Error: Main pump did not read high; retrying...");
                    this->digitalWriteOrReset(MixingStationIO::Components::MAIN_PUMP, HIGH);
                    tmp_retryAttempts++;
                }
                
                if (tmp_retryAttempts >= 3) {
                    lcd->display("Error: Timed out", String::format("after %d ms", MAIN_PUMP_TIMEOUT_SEC*1000), 1);
                    publishFuture = Particle.publish("pumpWaterToAquarium()", String::format("Error: Main pump timed out after %d ms", MAIN_PUMP_TIMEOUT_SEC*1000));
                    error = true;
                    break;
                }
                
                // print seconds to date so we know we haven't frozen
                if (millis() - secondsLastUpdated >= 1000) {
                    secondsLastUpdated = millis();
                    lcd->setCursor(0,1);
                    lcd->send_string(String::format("%d   ", (int)(MAIN_PUMP_TIMEOUT_SEC - (millis() - start)/1000.0)));
                }
                
                // new: utilize the new MAIN_PUMP_WATER_SENSOR to detect whether the main pump line going upstairs fills with air bubbles
                if ((millis() - start > 5*1000) && (this->digitalReadOrReset(MixingStationIO::Components::MAIN_PUMP_WATER_SENSOR) != ACTIVE_LOW)) {
                    delay(200); // "debounce" to ignore any transient blips
                    if (this->digitalReadOrReset(MixingStationIO::Components::MAIN_PUMP_WATER_SENSOR) != ACTIVE_LOW) {
                        // indicates that the main pump is now pumping air instead of water (i.e. the mixing tank is out of water and we're sucking in air)
                        lcd->display("Error: Out of water", "", 0);
                        publishFuture = Particle.publish("pumpWaterToAquarium()", "Error: Mixing tank is out of water. Air detected in main pump line");
                        error = true;
                        break;
                    }
                }
                
                Particle.process();
                
            } while (!isLiquidDetectedTop());
            this->digitalWriteOrReset(MixingStationIO::Components::MAIN_PUMP, LOW);        

            // open cutoff ball valve back up
            lcd->display("Opening cutoff", "valve...");
            WITH_LOCK(Wire) { mcp2->digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, HIGH); }
            delay(7*1000);

            // turn on canister filter and verify that flow rate is > zero
            if (!enableFilterAndHeatWithAlert()) {
                // note: alert will already have gone out in this case
                error = true;
                //return false;
            }

            // ============================================================================================================
            // NOTE: It's safe to return without calling enableFilterAndHeatWithAlert() below this line
            // ============================================================================================================
            
            waterLastPumpedToAquarium = millis();
            
            if (!publishFuture.isDone()) {
                lcd->display("(Waiting for", "publish...)", 0);
                delay(30*1000); // wait a bit longer
            }
            
            if (error) {
                delay(1000); // just in case this helps any <future> calls to publish() go out
                return false;
            }

            lcd->display("... success!", "", 2);

            
            
            
// temp for debugging
Particle.publish("pumpWaterToAquarium()", "Success!");
            return true;
        }
        
        // **note** may want to pet HW and SW watchdogs just prior to calling this, since we don't call Particle.process() or delay() -- see https://docs.particle.io/reference/device-os/api/cloud-functions/particle-process/
        bool dosePrime(float totalGallonsForDosing=10.0) {//float overrideDosageTimeSeconds=0.0) {


// **TODO** need to pet WDs in do/while loops below and in other functions



            lcd->display("Dosing...", "", 0);

            // make sure the sensor isn't already registering liquid
            if (this->digitalReadOrReset(MixingStationIO::Components::DOSING_PUMP_SENSOR) == ACTIVE_LOW) {
                lcd->display("Error: Liquid", "detected early", 3);
                Particle.publish("dosePrime()", "Error: Sensor detected liquid before dosing pump run");
                return false;
            }
            
            // calculate desired dosage: "Use 1 capful (5 mL) for each 200 L (50 US gallons) of new water" (https://www.seachem.com/prime.php)
            float dosageMl = 5.0*(totalGallonsForDosing/50.0);
            if (dosageMl < 0.5) { // enforce a minimum dosage to be safe
                dosageMl = 0.5;
            }
            if (dosageMl > 3) { // sanity check
                PushNotification::send(String::format("Error: Target dosage over-range. dosageMl: %.2f", dosageMl));
                return false;
            }

            unsigned int calculatedDosageTimeMs = round(dosageMl/(DOSING_PUMP_ML_PER_SEC)*1000);
            /*
            int calculatedDosageTimeMs;
            if (overrideDosageTimeSeconds <= 0) {
                calculatedDosageTimeMs = round(dosageMl/(DOSING_PUMP_ML_PER_SEC)*1000);
                //Particle.publish("dosePrime()", String::format("Calculated dosage time: %d ms for %.2f mL", calculatedDosageTimeMs, dosageMl));
            } else {
                calculatedDosageTimeMs = round(overrideDosageTimeSeconds*1000.0);
                dosageMl = calculatedDosageTimeMs/(DOSING_PUMP_ML_PER_SEC*1000); // recalculate expected dosage if dosage duration is what we're being given
                Particle.publish("dosePrime()", String::format("Overriding dosage time: %d ms (expecting %.2f ml)", calculatedDosageTimeMs, dosageMl));
            }
            */

            unsigned long totalSensorDurationUs = 0;
            unsigned long sensorLastTriggeredTs = 0;
            bool sensorLastTriggerState = false;
            unsigned int numTriggers = 0;

            // testing, as during the 3rd successive retry the device resets
            Particle.process();

            this->digitalWriteOrReset(MixingStationIO::Components::DOSING_PUMP, HIGH);
            unsigned long startUs = micros();
            unsigned long timeToFirstDetectionUs = 0; // now in uS
            bool timedOut = false;
            const unsigned long maxTime = DOSING_PUMP_MAX_TIME_SEC*1000*1000;
            
            // see how often we detect liquid
            // testing v2: using microseconds instead of milliseconds in case millis is too coarse of a resolution to yeild a meaningfully accurate "totalSensorDuration" figure
            unsigned long nowUs;
            do {
                nowUs = micros();
                bool sensorTriggered = (this->digitalReadOrReset(MixingStationIO::Components::DOSING_PUMP_SENSOR) == ACTIVE_LOW);
                if (sensorTriggered && !sensorLastTriggerState) { // if sensor was just triggered
                    sensorLastTriggeredTs = nowUs;
                    sensorLastTriggerState = true;
                } else if (!sensorTriggered && sensorLastTriggerState) { // sensor was just untriggered
                    totalSensorDurationUs += (nowUs - sensorLastTriggeredTs);
                    numTriggers++;
                    sensorLastTriggerState = false;
                    if (numTriggers == 1) {
                        timeToFirstDetectionUs = nowUs - startUs;
                    }
                }
                
                if (nowUs - startUs >= maxTime) {
                    timedOut = true;
                    break;
                }
            } 

            // note: allow up tp 10 seconds to detect the first liquid, e.g. if Prime solution was just swapped out
            while (nowUs - startUs < ((numTriggers == 0) ? 10*1000*1000 : calculatedDosageTimeMs*1000 + timeToFirstDetectionUs)); // note: we measure dosing time based on when the first liquid was detected
            unsigned long actualElapsedTime = micros() - startUs;
            
            this->digitalWriteOrReset(MixingStationIO::Components::DOSING_PUMP, LOW);
            
            if (timedOut) {
                lcd->display("Error: Timed out", String::format("after %d ms", DOSING_PUMP_MAX_TIME_SEC*1000), 3);
                Particle.publish("dosePrime()", String::format("Error: Dosing timed out after %d ms", DOSING_PUMP_MAX_TIME_SEC*1000));
                return false;
            }

            // v1 below
            /*
            do {
                bool sensorTriggered = (this->digitalReadOrReset(MixingStationIO::Components::DOSING_PUMP_SENSOR) == ACTIVE_LOW);
                if (sensorTriggered && !sensorLastTriggerState) { // if sensor was just triggered
                    sensorLastTriggeredTs = millis();
                    sensorLastTriggerState = true;
                } else if (!sensorTriggered && sensorLastTriggerState) { // sensor was just untriggered
                    totalSensorDurationMs += (millis() - sensorLastTriggeredTs);
                    numTriggers++;
                    sensorLastTriggerState = false;
                    if (numTriggers == 1) {
                        timeToFirstDetection = millis() - start;
                    }
                }
                
                if (millis() - start >= DOSING_PUMP_MAX_TIME_SEC*1000) {
                    timedOut = true;
                    break;
                }
            } 

            // note: allow up tp 10 seconds to detect the first liquid, e.g. if Prime solution was just swapped out
            while (millis() - start < ((numTriggers == 0) ? 10*1000 : calculatedDosageTimeMs + timeToFirstDetection)); // note: we measure dosing time based on when the first liquid was detected
            unsigned long actualElapsedTime = millis() - start;
            
            this->digitalWriteOrReset(MixingStationIO::Components::DOSING_PUMP, LOW);
            
            if (timedOut) {
                lcd->display("Error: Timed out", String::format("after %d ms", DOSING_PUMP_MAX_TIME_SEC*1000), 3);
                Particle.publish("dosePrime()", String::format("Error: Dosing timed out after %d ms", DOSING_PUMP_MAX_TIME_SEC*1000));
                return false;
            }
            */
            
            // make sure we detected sufficient liquid being dispensed
            // note: when dosing 1mL, observed num triggers = 126..175 and total sensor duration = 295..325ms; when dosing 2mL, observed num triggers = 335 and total sensor duration = 625ms
            // note: we also only expect 80% of this, to give ourselves a reasonable safety margin
            // older note: In testing this for 1000ms at a time, the average value for totalSensorDurationMs was 360-400ms, with numTriggers from 180-190.
            //   when Water supply was running out, totalSensorDurationMs went down to 22 and then 0 (numTriggers went down to 7 and 0)
            // ** update: when dosePrime() kept failing (with totalSensorDurations of about 75ms), it turned out that the IR RX/TX pair was dirty with dried seachem prime. Wiping them off with your finger fixed the problem, and now the dosing is succeeding (with totalSensorDurations around 198ms)
            // ** update 2: after adding longer tubing for the dosing pump, we started erroring out here. after measuring twice, we got just about 1ml when: totalSensorDurationMs = 104 (then 80), timeToFirstDetection = 313ms (then 44ms), numTriggers = 88 (then 67).
            const float EXPECTED_TOTAL_SENSOR_DURATION_PER_ML_MS = (100 * 0.50); // loosening this up a bit (120 * 0.50); // includes 50% fudge factor  // (300 * 0.50); // includes 50% fudge factor 
            if (totalSensorDurationUs < EXPECTED_TOTAL_SENSOR_DURATION_PER_ML_MS * 1000 * dosageMl) {
                lcd->display("Error: Volume", "too low");
                Particle.publish("dosePrime()", String::format("Error: Dosing sensor detected too little Seachem Prime. totalSensorDurationUs: %u us, dosageMl: %.1f, timeToFirstDetectionUs: %u ms, EXPECTED_TOTAL_SENSOR_DURATION_PER_ML_MS: %.2f, numTriggers: %d", 
                    totalSensorDurationUs, dosageMl, timeToFirstDetectionUs/1000, EXPECTED_TOTAL_SENSOR_DURATION_PER_ML_MS, numTriggers));
                return false;
            } else {
                Particle.publish("dosePrime()", String::format("Successfully dosed %.1f mL of Seachem Prime in %u ms. totalSensorDurationUs: %u ms, timeToFirstDetectionUs: %u ms, numTriggers: %d, EXPECTED_TOTAL_SENSOR_DURATION_PER_ML_MS: %.2f", 
                    dosageMl, actualElapsedTime/1000, totalSensorDurationUs/1000, timeToFirstDetectionUs/1000, numTriggers, EXPECTED_TOTAL_SENSOR_DURATION_PER_ML_MS));
            }

            lcd->display("... success!", "", 2);
            return true;
        }
        
        bool drainReservoir() {
            float sensorDetectionPercent = 0;
            return drainReservoir(sensorDetectionPercent);
        }
        
        bool drainReservoir(float &sensorDetectionPercent) {
            particle::Future<bool> publishFuture; // use to make Publish() asynchronous so it doesn't block (and the pump can't get hung stuck on). details: https://docs.particle.io/firmware/low-power/stop-sleep-cellular/#the-future

            lcd->display("Draining", "reservoir...", 0);
            
            bool drainSuccessful = false;
            this->digitalWriteOrReset(MixingStationIO::Components::DRAIN_PUMP, HIGH);
            unsigned long start = millis();

            delay(5*1000); // wait a bit for pump to prime itself (and for water to reach the drain sensor)
            
bool disconnected = false;            
            do {
// temp debugging
// sanity check for connection, in case we have to literally pull the plug while in progress
if (!isConnected()) {
    // TODO: particle::Future<bool> publishFuture; // use to make Publish() asynchronous so it doesn't block (and the pump can't get hung stuck on). details: https://docs.particle.io/firmware/low-power/stop-sleep-cellular/#the-future
    disconnected = true;
    Particle.publish("debug: drainReservoir()", "Draining error: disconnected");
    break;
}

                if (this->digitalReadOrReset(MixingStationIO::Components::DRAIN_PUMP_SENSOR) != ACTIVE_LOW) { // if no water is detected...
                    sensorDetectionPercent = getSensorPercentInTargetState(
                        MixingStationIO::Components::DRAIN_PUMP_SENSOR, //MixingStationIO::Components componentPin, 
                        ACTIVE_LOW,
                        5000, //unsigned long durationMs, 
                        250 //unsigned long delayBetweenReadingsMs=100
                    );
                    if (sensorDetectionPercent < 0.10) {
                        drainSuccessful = true;
                        publishFuture = Particle.publish("debug: drainReservoir()", "Draining complete. sensorDetectionPercent: " + String(sensorDetectionPercent));
                        break;
                    } else {
                        // testing only
                        publishFuture = Particle.publish("debug: drainReservoir()", "Draining not complete. sensorDetectionPercent: " + String(sensorDetectionPercent));
                    }
                }
                //delay(100);
                Particle.process(); // important to call this to keep the SW watchdog and (hopefully) HW watchdog software timer running
            } while (millis() - start < MAX_DRAIN_TIME_SECONDS*1000);
            this->digitalWriteOrReset(MixingStationIO::Components::DRAIN_PUMP, LOW);

// temp debugging
unsigned long elapsedTimeMs = millis() - start;
if (disconnected) {
    Particle.publish("drainReservoir()", String::format("temp debugging: error: IO disconnect; elapsedTimeMs: %u", elapsedTimeMs));
    delay(1000);
}
else            if (drainSuccessful) {
                lcd->display("... success!", "", 2);
// temp debugging
Particle.publish("drainReservoir()", String::format("temp debugging: Success; elapsedTimeMs: %u", elapsedTimeMs));
delay(1000);
            } else {
                lcd->display("Unsuccessful :-(", String::format("Pct: %.2f", sensorDetectionPercent), 3);
// temp debugging
Particle.publish("drainReservoir()", String::format("temp debugging: fail: Pct: %.2f, elapsedTimeMs: %u", sensorDetectionPercent, elapsedTimeMs), 3);
delay(1000);
            }

            return drainSuccessful;
        }
        
        // old version which used two liquid level sensors (vs flow meters)
        /*
        bool fill(MixingStationIO::WaterTypes waterDispenser, MixingStationIO::WaterTypes waterSensor, float &sensorDetectionPercent) {
            sensorDetectionPercent = -1;
            particle::Future<bool> publishFuture; // use to make Publish() asynchronous so it doesn't block (and the pump can't get hung stuck on). details: https://docs.particle.io/firmware/low-power/stop-sleep-cellular/#the-future
            const int FILL_TIMEOUT_MS = (waterDispenser == MixingStationIO::WaterTypes::TAP) ? TAP_WATER_FILL_TIMEOUT_SEC*1000 : RODI_WATER_FILL_TIMEOUT_SEC*1000; 
            const MixingStationIO::Components waterSolenoidComponent = (waterDispenser == MixingStationIO::WaterTypes::TAP) ? TAP_WATER_SOLENOID : RODI_SOLENOID;
            const MixingStationIO::Components waterSensorComponent = (waterSensor == MixingStationIO::WaterTypes::TAP) ? TAP_WATER_SENSOR : RODI_WATER_SENSOR;

            lcd->display("Filling with", (waterDispenser == MixingStationIO::WaterTypes::TAP) ? "tap..." : "RODI...", 0);
            unsigned long fillStart = millis();
            this->digitalWriteOrReset(waterSolenoidComponent, HIGH);

            // if water is detected, make sure the reading holds for long enough that we're sure it isn't a splash or noise
            bool fillSuccessful = false;
            do {
                if (this->digitalReadOrReset(waterSensorComponent) == ACTIVE_LOW) {
                    sensorDetectionPercent = getSensorPercentInTargetState(
                        waterSensorComponent,
                        ACTIVE_LOW,
                        500, //unsigned long durationMs, 
                        50 //delayBetweenReadingsMs=100
                    );
                    if (sensorDetectionPercent >= 0.80) {
                        fillSuccessful = true;
                        break;
                    } else {
                        // testing only
                        publishFuture = Particle.publish("debug: fill()", "sensorDetectionPercent did not meet threshold: " + String(sensorDetectionPercent));
                    }
                }
                //delay(100);
                Particle.process(); // important to call this to keep the SW watchdog and (hopefully) HW watchdog software timer running
            } while (millis() - fillStart < FILL_TIMEOUT_MS);
            this->digitalWriteOrReset(waterSolenoidComponent, LOW);

            if (fillSuccessful) {
                lcd->display("... success!", "", 2);
            } else {
                lcd->display("Unsuccessful :-(", String::format("Pct: %.2f", sensorDetectionPercent), 3);
            }

            return fillSuccessful;
        }
        */

        // Usage: Once a sensor detects a desired state, use the function below to make sure it holds at that state (at a reasonable duty cycle) for a given period of time
        // Returns an integer percentage from 0..100
        float getSensorPercentInTargetState(MixingStationIO::Components componentPin, bool targetState, unsigned long durationMs, unsigned long delayBetweenReadingsMs=100) {
            // WARNING: be mindful of tripping watchdog timers if loop delays are too short or zero below
            unsigned long start = millis();
            long countInTargetState = 0;
            long countNotInTargetState = 0;

            do {
                unsigned long loopStart = millis();
                
                bool sensorInTargetState = (this->digitalReadOrReset(componentPin) == targetState);
                if (sensorInTargetState) {
                    countInTargetState++;
                } else {
                    countNotInTargetState++;
                }

                unsigned long loopDurationMs = millis() - loopStart;
                if (delayBetweenReadingsMs > loopDurationMs) {
                    delay(delayBetweenReadingsMs - loopDurationMs);
                }
                
            } while (millis() - start <= durationMs);
            
            //Particle.publish("debug", String::format("countInTargetState: %2f, countNotInTargetState: %2f", countInTargetState, countNotInTargetState));
            //return (float)countInTargetState / ((float)countInTargetState + (float)countNotInTargetState);
            long total = countInTargetState + countNotInTargetState; // hacking around issue with float conversion
            return (float)countInTargetState/(float)total;
        }
  
        /*
        // Usage: Once a sensor detects a desired state, use the function below to make sure it holds at that state (at a reasonable duty cycle) for a given period of time
        // Returns an integer percentage from 0..100
        float getSensorDutyCycle(MixingStationIO::Components componentPin, unsigned long durationMs, bool activeState, unsigned long delayBetweenReadingsMs=100) {
            // WARNING: be mindful of tripping watchdog timers if loop delays are too short or zero below
            unsigned long start = millis();
            bool sensorActiveLastState = (this->digitalReadOrReset(componentPin) == activeState);
            unsigned long msInActiveState, msInInactiveState = 0;
            unsigned long lastStateChangeMillis = millis();
            
            do {
                unsigned long loopStart = millis();
                bool sensorActive = (this->digitalReadOrReset(componentPin) == activeState);
                if (sensorActive && !sensorActiveLastState) { // if sensor was just activated, log the time spent in the inactive state
                        unsigned long timeSinceLastStateChangeMs = millis() - lastStateChangeMillis;
                        msInInactiveState += timeSinceLastStateChangeMs;
                    lastStateChangeMillis = millis();
                } else if(!sensorActive && sensorActiveLastState) { // if sensor was just deactivated, log the time spent in the active state
                        unsigned long timeSinceLastStateChangeMs = millis() - lastStateChangeMillis;
                        msInActiveState += timeSinceLastStateChangeMs;
                    lastStateChangeMillis = millis();
                }

                unsigned long loopDurationMs = millis() - loopStart;
                delay(max(delayBetweenReadingsMs - loopDurationMs, 0));
                
            } while (millis() - start < durationMs);
            
            if (lastStateChangeMillis == 0) { // if there were no changes...
                return sensorActiveLastState ? 100.0 : 0.0;
            } else { // if there were changes...
                return (float)msInActiveState/(float)(msInActiveState + msInInactiveState);
            }
        }
        */
        
        /*
        void test_getSensorPercentInTargetState() {
            const unsigned long MAX_DRAIN_TIME_SECONDS = 5; // max time for this test
            
            this->digitalWriteOrReset(MixingStationIO::Components::DRAIN_PUMP, HIGH);
            unsigned long start = millis();

            // TODO: pet WD timers?
            do {
                //if (this->digitalReadOrReset(MixingStationIO::Components::DOSING_PUMP_SENSOR) != ACTIVE_LOW) { // if no water is detected...
                    float airDetectedDutyCycle = getSensorPercentInTargetState(
                        MixingStationIO::Components::DRAIN_PUMP_SENSOR, //MixingStationIO::Components componentPin, 
                        ACTIVE_LOW, //INACTIVE_HIGH, //bool targetState, 
                        1000, //unsigned long durationMs, 
                        50 //100 //unsigned long delayBetweenReadingsMs=100
                    );
                    Particle.publish("Detected duty cycle", String(airDetectedDutyCycle));
                    delay(1000);
                    if (airDetectedDutyCycle > 80) {
                        Particle.publish("airDetectedDutyCycle", "threshold met");
                        this->digitalWriteOrReset(MixingStationIO::Components::DRAIN_PUMP, LOW);
                        return;
                    }
                //}
                delay(100);
            } while (millis() - start < MAX_DRAIN_TIME_SECONDS*1000);
            this->digitalWriteOrReset(MixingStationIO::Components::DRAIN_PUMP, LOW);
            Particle.publish("airDetectedDutyCycle", "threshold not met before timeout");
        }
        */
        

        /*
        void test_sequence1() {
            
            this->digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, HIGH);
            do {
                delay(500);
            } while (this->digitalReadOrReset(MixingStationIO::Components::LOWER_WATER_SENSOR) != ACTIVE_LOW);
            this->digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, LOW);
            
            delay(5*1000);
            
            this->digitalWriteOrReset(MixingStationIO::Components::DRAIN_PUMP, HIGH);
            delay(5*1000);
            do {
                delay(500);
            } while (this->digitalReadOrReset(MixingStationIO::Components::DRAIN_PUMP_SENSOR) == ACTIVE_LOW);
            this->digitalWriteOrReset(MixingStationIO::Components::DRAIN_PUMP, LOW);
        }
        */
    
};