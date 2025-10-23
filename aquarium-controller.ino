#include <RunningAverage.h>

#include <DS18B20.h>

#define TARGET_DEVICE__ID_ENDING "58dda032f"

#define PIN__CO2_SENSOR_IN      D2 // 3.3v
#define PIN__CO2_SOLENOID       A4 // active high
#define PIN__DOSING_PUMP_1      A5 // active high


#define PIN__TEMP_PROBE_IN      D4
#define PIN__TEMP_PROBE_OUT     D5
#define PIN__HEATER_RELAY       D6
#define PIN__FLOW_SENSOR        D8 // note: 5V data pin routed through voltage divider

//#define PIN__PIEZO_BUZZER       A4
#define PIN__PUMP_CONTROL       A0
#define PIN__PUMP_V_SENSE       A1
#define PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE  A2
#define PIN__LIQUID_LEVEL_SENSOR_OPTICAL  A3

#define LED                     D7

#define HEATER_ENABLE           HIGH
#define HEATER_DISABLE          LOW // relay is active low to disconnect heater AC plug

#include "SharedUtilities.h"
#include "FlowRateSensor.h"
#include "CO2BubbleSensor.h""
#include "Waveshare_LCD1602_RGB.h"

// note: 255 seems way too bright
#define LCD_RGB_NORMAL 16, 16, 16
#define LCD_RGB_WARNING 32, 16, 16
#define LCD_RGB_PREWATERING 16, 16, 32
#define LCD_RGB_WATERING 16, 16, 32
#define LCD_RGB_SUCCESS 0, 16, 0





// TODO: consider moving into more elegant classes
bool waterDetectedOpticalSensor() {
    //return false; // TEMPORARY: This sensor has algae or something and seems to falsely trigger (or, the capacitive sensor is positioned too high up)
    return (digitalRead(PIN__LIQUID_LEVEL_SENSOR_OPTICAL) == false); // active low
}






volatile bool flowHeaterShutoffDisabled = true; // for now while flow rate sensor is stuck









volatile bool autoTopoffEnabled = true;
volatile bool isSimulationMode = false;
volatile bool simulation_capacitiveSensorTriggered = false;

 
SYSTEM_THREAD(ENABLED);
STARTUP(startup());

ApplicationWatchdog *wd;

void watchdogHandler() {
    // Do as little as possible in this function, preferably just
    // calling System.reset().
    // Do not attempt to Particle.publish(), use Cellular.command()
    // or similar functions. You can save data to a retained variable
    // here safetly so you know the watchdog triggered when you 
    // restart.
    // In 2.0.0 and later, RESET_NO_WAIT prevents notifying the cloud of a pending reset
    
    digitalWrite(PIN__PUMP_CONTROL, LOW);
    System.reset(RESET_NO_WAIT); // RESET_REASON_USER_APPLICATION_WATCHDOG
}

Waveshare_LCD1602_RGB lcd(16,2);  //16 characters and 2 lines of show
int r,g,b,t=0;


//rgb_lcd lcd;

// set DS18B20 data pin and flag as the only sensor on bus
DS18B20 tempSensorIn(PIN__TEMP_PROBE_IN, true);
DS18B20 tempSensorOut(PIN__TEMP_PROBE_OUT, true);

FlowRateSensor flowRateSensor;
CO2BubbleSensor co2BubbleSensor(PIN__CO2_SENSOR_IN);

float getTemp(DS18B20 *ds18b20){
    #define MAXRETRY 3
  float _temp;
  float fahrenheit = -9999;
  int   i = 0;

  do {
    _temp = ds18b20->getTemperature();
  } while (!ds18b20->crcCheck() && MAXRETRY > i++);

  if (i < MAXRETRY) {
    //celsius = _temp;
    fahrenheit = ds18b20->convertToFahrenheit(_temp);
  }
  else {
    fahrenheit = NAN;
    //Serial.println("Invalid reading");
  }
  return fahrenheit;
}


void startup() {
    // ensure that pump is always disabled on startup
    pinMode(PIN__PUMP_CONTROL, OUTPUT);
    digitalWrite(PIN__PUMP_CONTROL, LOW);
    pinMode(PIN__CO2_SENSOR_IN, INPUT_PULLDOWN);
    pinMode(PIN__CO2_SOLENOID, OUTPUT);
    digitalWrite(PIN__CO2_SOLENOID, LOW);
    pinMode(PIN__DOSING_PUMP_1, OUTPUT);
    digitalWrite(PIN__DOSING_PUMP_1, LOW);
    
    /*
    pinMode(PIN__PIEZO_BUZZER, OUTPUT); 
    pinSetDriveStrength(PIN__PIEZO_BUZZER, DriveStrength::HIGH); // TODO: consider checking for returned success/error code
    tone(PIN__PIEZO_BUZZER, 262, 5); // test tone
    */
}

void setup() {
    SharedUtilities::checkDeviceId(TARGET_DEVICE__ID_ENDING);

    wd = new ApplicationWatchdog(1000*60*5, watchdogHandler, 1536);
    
    System.on(firmware_update, reset_handler);
    
    ///pinMode(D6, INPUT_PULLDOWN);

    pinMode(PIN__HEATER_RELAY, OUTPUT);
    //digitalWrite(PIN__HEATER_RELAY, HEATER_DISABLE);
    //delay(1000);
    digitalWrite(PIN__HEATER_RELAY, HEATER_ENABLE);
    
    pinMode(PIN__LIQUID_LEVEL_SENSOR_OPTICAL, INPUT);
    
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    //flowRateSensor = new FlowRateSensor();
    
    flowRateSensor.init();
    co2BubbleSensor.init(); // TODO: init() as part of constructor?
    
    lcd.setRGB(LCD_RGB_NORMAL);
    lcd.init();
    lcd.setCursor(0,0);
    lcd.send_string("Connecting...");
    //lcd.setCursor(0,1);
    //lcd.send_string("Hello, World!");


    /*
    const int colorR = 0;
    const int colorG = 255;
    const int colorB = 0;
    lcd.begin(16, 2);
    lcd.setRGB(colorR, colorG, colorB);
    lcd.print("Connecting to wifi...");
    */
    
    // wifi takes forever to connect -- could it because of our use of this below?
    /*
    if (!waitFor(Particle.connected, 50*1000)) {
        System.reset(101);
    }
    */
    unsigned long start = millis();
    do {
        Particle.process(); // ??
        delay(250);
    } while (millis() - start < 50*1000 && !Particle.connected());
    if (!Particle.connected()) {
        System.reset(101);
    }
    
    Particle.function("toggleTopoff", cloud_toggleTopoff);
    Particle.function("getTempIn", cloud_getTempIn);
    Particle.function("addWaterManually", cloud_addWaterManually);
    Particle.function("simulationModeToggle", cloud_simulationModeToggle);
    Particle.function("sim_CapSensorToggle", cloud_sim_CapSensorToggle);
    Particle.function("samplePumpData", cloud_samplePumpReadings);
    Particle.function("toggleFlowShutoff", cloud_toggleFlowShutoff);
    Particle.function("testPWMFreq", cloud_testPWMFreq);
    Particle.function("tempTest1", cloud_tempTest1);
    //Particle.function("opticalTest", cloud_opticalTest);
    
    lcd.clear();
    
    RGB.control(true);
    RGB.brightness(12);
    RGB.control(false);
}

char bufferLine1[16];

#define LOG_TO_SPREADSHEET_TIME_MS  60*60*1000
unsigned long timeSinceLogToSpreadsheet = 0;

#define LCD_UPDATE_TIME_MS  1000
unsigned long timeSinceLcdUpdate = 0;

#define WATER_LEVEL_CHECK_TIME_MS  5000
unsigned long timeSinceWaterLevelCheck = 0;

#define HEATER_DISABLE_COOLDOWN_MS  1000*60*5
unsigned long heaterLastDisabled = 0;

#define WATER_SENSOR_WAIT_TIME_MS   1000*10 // how long the water sensor must show a consistent reading
#define MAX_WATERING_TIME_MS    1000*60
#define PUMP_EXTRA_TIME_MS      1000*5
#define WATER_TOPOFF_COOLDOWN_TIME_MS   1000*60*60
unsigned long waterLastToppedOff = 0;


unsigned long heaterEnableTimestamp = 0; // set to a value of millis() that will reenable the heater
String heaterDisableReason;


void doWaterLevelCheck() {
    doWaterLevelCheck(false);
}

void rampPump(bool startVsStop, int optPWMFreq=20000) { // note: 1KHz results in audiable tone when pump is running at low speeds
    if (startVsStop) {
        for (int i=100; i<=255; i++) {
            analogWrite(PIN__PUMP_CONTROL, i, optPWMFreq); // TODO: test to find best PWM frequency
            delay(5);
        }
    } else {
        for (int i=255; i>=100; i--) {
            analogWrite(PIN__PUMP_CONTROL, i, optPWMFreq); // TODO: test to find best PWM frequency
            delay(4);
        }
        digitalWrite(PIN__PUMP_CONTROL, LOW);
    }
}

void doWaterLevelCheck(bool optSkipSensorTest) {
    if (optSkipSensorTest || (!waterDetectedCapacitiveSensor() && digitalRead(PIN__HEATER_RELAY) != HEATER_DISABLE && (waterLastToppedOff == 0 || millis() - waterLastToppedOff > WATER_TOPOFF_COOLDOWN_TIME_MS))) {
        RunningAverage runningPumpSpikes(5);
        runningPumpSpikes.fillValue(0, 5);

        lcd.setRGB(LCD_RGB_PREWATERING);
        
        if (!optSkipSensorTest && waterDetectedOpticalSensor()) {
            // this should never happen (edit: sometimes it does, e.g. from agae or if capacitive sensor is above the optical sensor)
            //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Optical sensor detects water but capacitive sensor does not. Skipping topoff.""\" }", PRIVATE);
            waterLastToppedOff = millis();
        } else {

            bool waterDetected = false;
            
            if (!optSkipSensorTest) {
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.send_string("Confirming water");
                lcd.setCursor(0,1);
                lcd.send_string("level...");
                delay(1000);
    
                // wait for x seconds to make sure water level sensor remains constant
                unsigned long senseStart = millis();
                unsigned long senseElapsedTime;
                do {
                    senseElapsedTime = millis() - senseStart;
                    if (waterDetectedCapacitiveSensor()) {
                        waterDetected = true;
                        break;
                    }
                    //if (senseElapsedTime % 1000 == 0) { // doesn't actually work
                        lcd.setCursor(13, 1);
                        lcd.send_string(String((int)floor(senseElapsedTime/1000)));
                    //}
                    delay(500); // TODO: ***necessary to prevent noisy spikes in sensor from triggering false readings?***
                } while (senseElapsedTime < WATER_SENSOR_WAIT_TIME_MS);
            }
            
            if (!waterDetected) { // TODO: consider incorporating waterDetectedCapacitiveSensorFluctuating()
                // we still haven't detected water, so proceed
                
                lcd.setRGB(LCD_RGB_WATERING);

                lcd.clear();
                lcd.setCursor(0,0);
                lcd.send_string("Checking flow...");

                delay(1000); // current issue w/ sensor reading returning -1
                float lastFlowRate = flowRateSensor.readFlowRateGPH();
                if (lastFlowRate == -1) {
                    lcd.setRGB(LCD_RGB_WARNING);
                    Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: About to top of tank, but measured flow rate of -1""\" }", PRIVATE);
                    delay(1000);
                    lcd.setRGB(LCD_RGB_NORMAL);
                    return; // TODO: stop returning -1
                }

                lcd.setCursor(0,0);
                lcd.send_string("Adding water...");

                waterLastToppedOff = millis();
                
                // turn on pump
                //analogWrite(PIN__PUMP_CONTROL, 128);
                unsigned long start = millis();
                rampPump(true);
                /*
                for (int i=100; i<=255; i++) {
                    analogWrite(PIN__PUMP_CONTROL, i, 1000); // TODO: test to find best PWM frequency
                    delay(5);
                }
                */
                
                delay(150); // needed to avoid start-up spikes?
                
            
                bool outOfWater = false;
                bool lowFlowRateFlag = false;
                unsigned long elapsedTime;
                unsigned long lastFlowRateReadTime = 0;
                float minAvgFlowRate = 9999;
                RunningAverage avgFlowRate(3);
                float currentFlowRate = flowRateSensor.readFlowRateGPH();
                
                do {
                    elapsedTime = millis() - start;

                    // measure once per second to avoid avg == instantaneous value once you fix the issue where flow rate can read as -1                    
                    if (millis() - lastFlowRateReadTime > 1000) {
                        currentFlowRate = flowRateSensor.readFlowRateGPH();
                        lastFlowRateReadTime = millis();
                    }


                    //if (elapsedTime % 1000 == 0) {
//                        lcd.setCursor(0,1);
//                        lcd.send_string(String((int)floor(elapsedTime/1000)) + "  " + String(waterDetectedCapacitiveSensorFluctuating() ? "~~~" : "---") + "  " + String(currentFlowRate) + " GPH");
                    //}
                    
                    if (millis() - elapsedTime > 1000 && currentFlowRate != -1) { // don't measure too soon
                        float flowRateDelta = currentFlowRate - lastFlowRate;
                        avgFlowRate.addValue(flowRateDelta);
                        float avg = avgFlowRate.getAverage();
                        lcd.setCursor(0,1);
                        //lcd.send_string("+" + String(flowRateDelta) + " GPH          ");
                        lcd.send_string("+" + String((int)flowRateDelta) + " GPH / +" + String((int)avg) + " avg");
                        
                        // TODO: stop if flow is too slow; skip adding extra  //if (flowRateDelta)
                        if (avgFlowRate.getCount() == avgFlowRate.getSize()) {
                            if (avg < minAvgFlowRate) {
                                minAvgFlowRate = avg;
                            }
                            if (avg < 7.0) { // note: looks like delta can spike from +8 to +200 (!!)
                                lowFlowRateFlag = true;
                                break;
                            }
                        }
                    }

                    
                    if (waterDetectedCapacitiveSensor()) { // waterDetectedCapacitiveSensorFluctuating()?
                        lcd.clear();
                        lcd.setCursor(0,0);
                        lcd.send_string("Adding a bit");
                        lcd.setCursor(0,1);
                        lcd.send_string("extra...");
                        delay(PUMP_EXTRA_TIME_MS);

                        rampPump(false);
                        /*
                        for (int i=255; i>=100; i--) {
                            analogWrite(PIN__PUMP_CONTROL, i, 1000); // TODO: test to find best PWM frequency
                            delay(4);
                        }
                        digitalWrite(PIN__PUMP_CONTROL, LOW);
                        */
                        elapsedTime = millis() - start; // update slapsedTime
        
                        lcd.setRGB(LCD_RGB_SUCCESS);
                        lcd.clear();
                        lcd.setCursor(0,0);
                        lcd.send_string("Complete!");
                        
                        //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Water added to tank (" + String(elapsedTime/1000.0) + " seconds). minAvgFlowRate: +" + String(minAvgFlowRate) + " GPH. Last avgFlowRate: +" + String(avgFlowRate.getAverage()) + " GPH""\" }", PRIVATE);
                        //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Water added to tank (" + String(elapsedTime/1000.0) + " seconds)""\" }", PRIVATE);
                        Particle.publish("google-sheets", String::format(
                            "{"
                                "\"sheet\": \"Aquarium-topoff-data\", "
                                "\"Elapsed time (seconds)\": %.2f, "
                            "}",
                            elapsedTime/1000.0
                        ));
                        
                        delay(2000);
                        break;
                    }

// TODO: remove !optSkipSensorTest ?                    
                    if (waterDetectedOpticalSensor() && !optSkipSensorTest) { //(!optSkipSensorTest || elapsedTime > 5000)) {

                        digitalWrite(PIN__PUMP_CONTROL, LOW);

                        lcd.setRGB(LCD_RGB_WARNING);
                        lcd.clear();
                        lcd.setCursor(0,0);
                        lcd.send_string("Optical sensor");
                        lcd.setCursor(0,1);
                        lcd.send_string("was tripped");
                        
                        Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Water top-off stopped after optical sensor tripped. Elapsed time: " + String(elapsedTime/1000) + " seconds)""\" }", PRIVATE);
                        delay(2000);

                        break;
                    }
                    Particle.process();
                    




/*
                    //delayAndMotorSense(500);
                    for (int i=0; i<=5; i++) {
                        int numSpikes = getPumpSpikes100ms();
                        runningPumpSpikes.addValue(numSpikes);
                        digitalWrite(LED, numSpikes > 0);
                    }

    
    
    
    

                    digitalWrite(LED, LOW);

                    // TODO: see if we have enough spikes to indicate we're out of water~~~
                    //outOfWater = true;
                    int totalSpikes = 0;
                    for (int j=0; j<5; j++) {
                        totalSpikes += runningPumpSpikes.getElement(j);
                        //outOfWater &= (runningPumpSpikes.getElement(j) > 0);
                    }
                    outOfWater = false; /////(totalSpikes > 9); // not sure
                    
                    if (outOfWater) {
                        break;
                    }
                    
                    //delayAndMotorSense(500); // testing -- does pump cause sensor output to spike falsely?
*/                    
                 
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                } while (elapsedTime < MAX_WATERING_TIME_MS && !outOfWater && !lowFlowRateFlag);
                
                // turn off pump (possibly duped, but that's okay)
                digitalWrite(PIN__PUMP_CONTROL, LOW);

                if (lowFlowRateFlag || outOfWater) {
                    lcd.setRGB(LCD_RGB_WARNING);
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.send_string("Out of water");
                    
                    Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Reservoir is out of water. Initial flow rate: +" + String(lastFlowRate) + " GPH. Last average flow rate: +" + String(avgFlowRate.getAverage()) + " GPH\" }", PRIVATE);
                    
                    // TODO: publish to sheets before returning
                    
                    delay(4000);
                    
                    
                    
                } else if (elapsedTime >= MAX_WATERING_TIME_MS) {
                    lcd.setRGB(LCD_RGB_WARNING);
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.send_string("Optical sensor");
                    lcd.setCursor(0,1);
                    lcd.send_string("not tripped");
                    
                    Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Water top-off stopped after maximum amount of time without sensor tripped. Elapsed time: " + String(elapsedTime/1000) + " seconds)""\" }", PRIVATE);
                    delay(2000);
                }
                
                
            } else {
                // water actually was detected, so abort
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.send_string("Inconsistent");
                lcd.setCursor(0,1);
                lcd.send_string("reading");
                delay(3000);
            }
        }
        
        lcd.setRGB(LCD_RGB_NORMAL);
    }    
}

volatile bool cloud_samplePump_flag = false;
volatile bool flag_doWaterLevelCheck = false;

void loop() {
    
    // temp
    digitalWrite(LED, digitalRead(PIN__CO2_SENSOR_IN));
    
    
    
    

    
    // testing
    Particle.process();
    
    if (flag_doWaterLevelCheck) {
        flag_doWaterLevelCheck = false;
        doWaterLevelCheck(true);
    }
    
    //digitalWrite(LED, digitalRead(PIN__FLOW_SENSOR));
    if (cloud_samplePump_flag) {
        cloud_samplePump_flag = false;
        samplePumpReadings();
    }
    
    if (autoTopoffEnabled && (millis() - timeSinceWaterLevelCheck > WATER_LEVEL_CHECK_TIME_MS || isSimulationMode)) {
        timeSinceWaterLevelCheck = millis();
        doWaterLevelCheck();
    }

    

    
    
    
    
    float tempInF = getTemp(&tempSensorIn);
    float tempOutF = getTemp(&tempSensorOut);
    float flowRate = flowRateSensor.readFlowRateGPH();


    if (millis() - timeSinceLcdUpdate > LCD_UPDATE_TIME_MS) {
        timeSinceLcdUpdate = millis();
        
        
        
        
/*        

~~~~~~        
unsigned long heaterEnableTimestamp = 0; // set to a value of millis() that will reenable the heater
String heaterDisableReason;

        
        
        
        
        // second stab below
        // if heater is not disabled...
        if (digitalRead(PIN__HEATER_RELAY) != HEATER_DISABLE) {
            // check for excessive temperature
            if (tempInF > 83 || tempOutF > 90) {
                // turn off heater; it will turn on when temp returns to a normal range
                digitalWrite(PIN__HEATER_RELAY, HEATER_DISABLE);
            }
            
            // check for no flow
            // TODO: turn off heater for up to 1 hour, then return to normal operation for TBD hours
            // > need: timestamp to ignore flow rate errors, and timestamp to reset this state
        }
        
        // if heater is disabled...
        else {
            
            // see if temperature is okay
            if (tempInF <= 83 && tempOutF <= 90) {
                digitalWrite(PIN__HEATER_RELAY, HEATER_ENABLE);
            }
            
            // see if flow is okay
            
        }
        
        // separately, check for low temperature (and alert only)
        if (tempInF < 73 || tempOutF < 73) {
            
        }
*/        
        
        
        
        
        
        
        // TODO: refactor where core checks go
        // TODO: clean up w/ states, etc
        // TODO: break out separately for flow rate, e.g. in cooldown time
        if (tempInF > 83 || tempOutF > 90) {
            if (millis() - heaterLastDisabled > HEATER_DISABLE_COOLDOWN_MS) { // TODO: fix this -- it just keeps repeating
                digitalWrite(PIN__HEATER_RELAY, HEATER_DISABLE);
                heaterLastDisabled = millis();
                lcd.setRGB(LCD_RGB_WARNING);
                Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Heater disabled due to flow rate and/or temperature""\" }", PRIVATE);
            }
        } 

        if (tempInF < 75 || tempOutF < 75) {
            
            for (int i=0; i<10; i++) {
                // jiggle the relay in case it's stuck
                digitalWrite(PIN__HEATER_RELAY, HEATER_DISABLE);
                delay(125);
                digitalWrite(PIN__HEATER_RELAY, !HEATER_DISABLE);
                delay(125);
            }
            
            Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Temperature is too low: " + String(tempInF).substring(0, 5) + """\" }", PRIVATE);
            
            lcd.setRGB(LCD_RGB_WARNING);
            unsigned long start = millis();
            while (millis() - start < 1000*60*60 && tempInF < 75) {
                tempInF = getTemp(&tempSensorIn);

                lcd.clear();
                lcd.setCursor(0,0);
                lcd.send_string("Temp too low: ");
                lcd.setCursor(0,1);
                lcd.send_string(String(tempInF).substring(0, 5) + (char)223 + "   ");

                delay(1000 * 10);
                Particle.process();
            };
                
        }

        
        if (!flowHeaterShutoffDisabled && (flowRate < 1.0 && digitalRead(PIN__HEATER_RELAY) != HEATER_DISABLE)) {
// TODO: fix !flowHeaterShutoffDisabled &&  above... probably all messed up

            // temp
            
            digitalWrite(PIN__HEATER_RELAY, HEATER_DISABLE);

            lcd.setRGB(LCD_RGB_WARNING);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string("Low flow rate.");
            lcd.setCursor(0,1);
            lcd.send_string("Heater disabled.");
            
            Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Low flow rate; disabling heat temporarily""\" }", PRIVATE);
            
            unsigned long start = millis();
            float lastFlowRate;
            do {
                lastFlowRate =  flowRateSensor.readFlowRateGPH();
                wd->checkin(); // make sure the watchdog doesn't reset us during this potentially long loop
                delay(500);
                Particle.process(); // note: this was needed to prevent cloud functions from not working in this loop
            } while (!flowHeaterShutoffDisabled && (millis() - start < 1000*60*60*2 && lastFlowRate < 20)); // break out if flowHeaterShutoffDisabled flag gets set while in this loop
            
            digitalWrite(PIN__HEATER_RELAY, HEATER_ENABLE);
            if (lastFlowRate >= 20) {
                lcd.setRGB(LCD_RGB_SUCCESS);
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.send_string("Flow rate");
                lcd.setCursor(0,1);
                lcd.send_string("restored!");
                
                Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Flow rate restored; heat re-enabled""\" }", PRIVATE);
                delay(3000);
            } else {
                lcd.setRGB(LCD_RGB_WARNING);
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.send_string("Safety timeout.");
                lcd.setCursor(0,1);
                lcd.send_string("Heat re-enabled.");
                
                Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: No flow, but safety timed out. Re-enabling heat.""\" }", PRIVATE);
                delay(3000);
            }
            lcd.setRGB(LCD_RGB_NORMAL);


            
            
            
        } // .....
        
        // TODO: add alert if temp is too cold
        
        else {
            digitalWrite(PIN__HEATER_RELAY, HEATER_ENABLE);
            lcd.setRGB(LCD_RGB_NORMAL);
        }
        


        //lcd.clear(); // ????????????
        lcd.setCursor(0,0);
        lcd.send_string(String(tempInF).substring(0, 5) + (char)223 + "   "); //.concat("      "));
        lcd.setCursor(9,0);
        lcd.send_string(String(tempOutF).substring(0, 5) + (char)223 + "  "); //.concat("      "));
        lcd.setCursor(0,1);
        lcd.send_string(String(flowRate).substring(0, 5) + " GPH  ");
        lcd.setCursor(9,1);
        //lcd.send_string(tempInF > tempOutF ? "   Idle" : "   Heat"); // TODO: add warning states too
        lcd.send_string("  T" + String(waterDetectedOpticalSensor() ? "+" : "-") + "B" + String(waterDetectedCapacitiveSensorFluctuating() ? "~" : String(waterDetectedCapacitiveSensor() ? "+" : "-"))); // eg " T+, B~"
        /*lcd.send_string(String::format("  T%sB%s", 
            String(waterDetectedOpticalSensor() ? "+" : "-"),
            String(waterDetectedCapacitiveSensorFluctuating() ? "~" : String(waterDetectedCapacitiveSensor() ? "+" : "-"))
        ));*/
        //lcd.send_string("   " + String(analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE)) + "    ");
        
    }
    
    if (millis() - timeSinceLogToSpreadsheet > LOG_TO_SPREADSHEET_TIME_MS) {
        
// TODO: make sure we're not pumping liquid        
        
        timeSinceLogToSpreadsheet = millis();
        Particle.publish("google-sheets", String::format(
            "{"
                "\"sheet\": \"Aquarium-sensor-data\", "
                "\"Temperature (tank)\": %.5f, "
                "\"Temperature (heater)\": %.5f, "
                "\"Flow rate\": %.5f, "
            "}",
            tempInF, tempOutF, flowRate
        ));
    }

}




// TODO: consider moving into more elegant classes
bool waterDetectedCapacitiveSensor() {
    if (isSimulationMode) {
        return simulation_capacitiveSensorTriggered;
    } else {
        return (analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE) < 2900); // normal value seems to be ~3100 when no contact with wqter
    }
}

bool waterDetectedCapacitiveSensorFluctuating() {
    int reading = analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE);
    return (reading > 1000 && reading < 2900);
}

int cloud_toggleTopoff(String extra) {
    autoTopoffEnabled = !autoTopoffEnabled;
    return autoTopoffEnabled;
}

int cloud_getTempIn(String extra) {
    return getTemp(&tempSensorIn)*100;    
}

int cloud_simulationModeToggle(String extra) {
    isSimulationMode = !isSimulationMode;
    simulation_capacitiveSensorTriggered = waterDetectedCapacitiveSensor(); // set to existing value
    return isSimulationMode;
}

int cloud_toggleFlowShutoff(String extra) {
    flowHeaterShutoffDisabled = !flowHeaterShutoffDisabled;
    return flowHeaterShutoffDisabled;
}

int cloud_sim_CapSensorToggle(String extra) {
    simulation_capacitiveSensorTriggered = !simulation_capacitiveSensorTriggered;
    return simulation_capacitiveSensorTriggered;
}

int cloud_addWaterManually(String extra) {
    flag_doWaterLevelCheck = true;
    return 1;
}

int cloud_testPWMFreq(String extra) {
    int freq = extra.toInt();
    if (freq <= 0) {
        return -1;
    }

    rampPump(true, freq);
    delay(1000);
    rampPump(false, freq);

    return freq;
}

int cloud_tempTest1(String extra) {
    digitalWrite(PIN__CO2_SOLENOID, !digitalRead(PIN__CO2_SOLENOID));
    digitalWrite(PIN__DOSING_PUMP_1, !digitalRead(PIN__DOSING_PUMP_1));
    return 123;
}

/*
void pumpOnGradual() {
    unsigned long start = millis();
    for (int i=0; i<=192; i++) {
        analogWrite(PIN__PUMP_CONTROL, i, 1000); // TODO: test to find best PWM frequency
        delay(10);
    }
}
*/

int getPumpSpikes100ms() {
    int zeros = 0;
    int spikes = 0;
    unsigned long start = millis();
    do {
        float v = readMotorVoltageDrop();
        if (v == 0) {
            zeros++;
        }
        /*
        if (v > 0.01) {
            tmp_max++;
        }
        */
        if (v > 0.05) { // this one looks even better. add up the counts per 100ms and (4 vs 15 after 10 repetitions in your test). every 100ms set had 1 (some more) when no water
            spikes++;
        }
        /*
        if (v > 0.1) { // also looks promising
            tmp_max3++;
        }
        if (v > 0.2) {
            tmp_max4++;
        }
        */
    } while (millis() - start <= 100);    
    
    //return (float)spikes/zeros;
    return spikes;
}


float tmp_min;
float tmp_max;
float tmp_max2;
float tmp_max3;
float tmp_max4;
int wait100MsAndCheckIfEmpty() {
    tmp_min = 0;
    tmp_max = 0;
    tmp_max2 = 0;
    tmp_max3 = 0;
    tmp_max4 = 0;

    unsigned long start = millis();
    int numSpikes = 0;
    
    
    do {
        float v = readMotorVoltageDrop();
        digitalWrite(LED, v <= 0.15);
        if (v == 0) {
            tmp_min++;
        }
        if (v > 0.01) {
            tmp_max++;
        }
        if (v > 0.05) { // this one looks even better. add up the counts per 100ms and (4 vs 15 after 10 repetitions in your test). every 100ms set had 1 (some more) when no water
            tmp_max2++;
        }
        if (v > 0.1) { // also looks promising
            tmp_max3++;
        }
        if (v > 0.2) {
            tmp_max4++;
        }
        /*if (v < 0.10) {
            numSpikes++;
        }*/
        //delay(1);
        //runningAvg.addValue(voltageDrop);
        //delayAndMotorSense(500); // testing -- does pump cause sensor output to spike falsely?
    } while (millis() - start <= 100);
    
    digitalWrite(LED, LOW);
    return numSpikes;
}


int cloud_opticalTest(String extra) {
    return -1;
    
    
    
    
    if (waterDetectedOpticalSensor()) {
        return 0;
    }
    
    rampPump(true);
    /*
    for (int i=0; i<=255; i++) {
        analogWrite(PIN__PUMP_CONTROL, i, 
            5000); // TODO: test to find best PWM frequency
        delay(4);
    }
    */

    while (!waterDetectedOpticalSensor()) {
        delay(100);
        Particle.process();
    }
    
    digitalWrite(PIN__PUMP_CONTROL, LOW);
    
    return 123;
}

int cloud_testPump_old(String extra) {
    //RunningAverage runningAvg(100);
    //runningAvg.clear();
    
    //analogWrite(PIN__PUMP_CONTROL, 192);
    
    
    unsigned long start = millis();
    for (int i=0; i<=255; i++) {
        analogWrite(PIN__PUMP_CONTROL, i, 1000); // TODO: test to find best PWM frequency
        delay(4);
    }
    digitalWrite(PIN__PUMP_CONTROL, HIGH);
    delay(500);
    //pumpOnGradual();
    
    int results[10][5];
    float spikes[10];
    for (int i=0; i<10; i++) {
        //wait100MsAndCheckIfEmpty();
        spikes[i] = getPumpSpikes100ms();
        /*
        results[i][0] = tmp_min;
        results[i][1] = tmp_max;
        results[i][2] = tmp_max2;
        results[i][3] = tmp_max3;
        results[i][4] = tmp_max4;
        */
    }
    
    digitalWrite(PIN__PUMP_CONTROL, LOW);
    
    
    for (int i=0; i<10; i++) {
        //Particle.publish("debug: test value", String(results[i][0]) + ", " + String(results[i][1]) + ", " + String(results[i][2]) + ", " + String(results[i][3]) + ", " + String(results[i][4]));
        Particle.publish("debug: pctSpikes", String(spikes[i]*100));
        delay(1001);
    }
    
    
    //Particle.publish("debug: pctSpikes", String(pctSpikes*100));
    Particle.publish("debug: test value", "---");
    
    return 123;
    
    
    /*
    unsigned long start = millis();
    
    do {
        float voltageDrop = readMotorVoltageDrop();
        delay(30);
        runningAvg.addValue(voltageDrop);
        //delayAndMotorSense(500); // testing -- does pump cause sensor output to spike falsely?
    } while (millis() - start < 3*1000);
    
    digitalWrite(PIN__PUMP_CONTROL, LOW);
    
    Particle.publish("debug: average value", String(runningAvg.getAverage()));
    return runningAvg.getAverage() * 1000;
    */
}



float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readMotorVoltageDrop() {
    
    // 12v -- motor pin -- motor pin -- MOSFET -- GND
    //                       /\
    //                        | 10x voltage divider -- sense pin
    float volts = analogRead(PIN__PUMP_V_SENSE);
    volts = mapFloat(volts, 0, 4095, 0, 3.3);

    return volts;

}

bool isMotorUnderStrain() {
    
    float v = readMotorVoltageDrop();
    
    //return (v <= 0.014);
    return (v <= 0.05);
    
}

void delayAndMotorSense(unsigned long delayMs) {
    if (delayMs < 10) {
        delay(delayMs);
        return;
    }
    
    unsigned long start = millis();
    do {
        digitalWrite(LED, isMotorUnderStrain());
        delay(10);
        
    } while (millis() - start < delayMs);
    
    digitalWrite(LED, LOW);
}


void reset_handler(system_event_t event, int param) {
    if (param == firmware_update_begin) {
        digitalWrite(PIN__PUMP_CONTROL, LOW);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.send_string("Updating");
        lcd.setCursor(0,1);
        lcd.send_string("firmware...");
        // note: loop() will continue executing after this (or just in the background?)
    }
    
}


void testPump1() {
    const unsigned int TEST_RUNTIME_MS = 10*1000; //6*1000
    unsigned int runningAvg100s[TEST_RUNTIME_MS/10]; // 100 averages, 10ms each
    const int threshold = 18; //25; // see charts
    RunningAverage runningAvg10s(10); // 10 averages, 1ms each
    RunningAverage averageOfAverages(200);//TEST_RUNTIME_MS/10);
    RunningAverage avgOfOutOfWaterFlag(3*10); // edit: trying longer than 1 second // targeting one second of readings -- want about 1 second, which should be 10 samples (1/10 of the loop which takes 1ms per iteration)
    
    rampPump(true);
    /*
    for (int i=0; i<=255; i++) {
        analogWrite(PIN__PUMP_CONTROL, i, 5000); // TODO: test to find best PWM frequency
        delay(2);
    }
    */
    delay(500);

    unsigned long start = millis();
    int count = 0;
    int i = 0;
    //digitalWrite(LED, HIGH);
    do {
        
        float vDrop = readMotorVoltageDrop();
        runningAvg10s.addValue(vDrop);
        if (count++ % 10 == 0) {
            float avg = runningAvg10s.getAverage() * 100000; // this is the average of the last 10 samples (~1ms each)
            runningAvg100s[i++] = avg;
            averageOfAverages.addValue(avg);
            float avgOfAvg = averageOfAverages.getAverage();
            
            bool mayBeOutOfWater = (averageOfAverages.getCount() == averageOfAverages.getSize()) && (avgOfAvg < threshold); 
            avgOfOutOfWaterFlag.addValue(mayBeOutOfWater ? 1.0 : 0); // do we need a float??? left off here. 3/10 is not avg of > 0.95
            digitalWrite(LED, mayBeOutOfWater);
            
            /*
            if ((avgOfOutOfWaterFlag.getCount() == avgOfOutOfWaterFlag.getSize()) && (avgOfOutOfWaterFlag.getAverage() > 0.95)) { // see if 80% or more of samples have had the flag, after we have 10 samples
                RGB.control(true);
                RGB.color(255, 255, 0);
            }
            */
            if (avgOfOutOfWaterFlag.getCount() == avgOfOutOfWaterFlag.getSize()) { // see if 80% or more of samples have had the flag, after we have 10 samples
                bool allOnes = true;
                for (int index=0; index<avgOfOutOfWaterFlag.getCount(); index++) {
                    allOnes &= (avgOfOutOfWaterFlag.getElement(index) == 1);
                }
                if (allOnes) {
                    RGB.control(true);
                    RGB.color(255, 255, 0);
                }
            }

        }
        
        
        delay(1);
        
    } while (millis() - start <= TEST_RUNTIME_MS);
    digitalWrite(LED, LOW);
    digitalWrite(PIN__PUMP_CONTROL, LOW);
    RGB.control(false);
    
    String valuesStr;
    valuesStr.reserve(1024);
    valuesStr = "";

    for (int index=0; index<i; index++) {
        valuesStr += String(runningAvg100s[index]) + ",";
        if (valuesStr.length() > 900) {
            Particle.publish("Test results: runningAvg100s (partial)", valuesStr);
            valuesStr = "";
            delay(1000);
        }
    }
    valuesStr += "end";
    Particle.publish("Test results: runningAvg100s (final)", valuesStr);
    
        
    valuesStr = "";
    for (int index=0; index<averageOfAverages.getCount(); index++) {
        valuesStr += String(averageOfAverages.getElement(index)) + ",";
        if (valuesStr.length() > 900) {
            Particle.publish("Test results: averageOfAverages (partial)", valuesStr);
            valuesStr = "";
            delay(1000);
        }
    }
    valuesStr += "end";
    Particle.publish("Test results: averageOfAverages (final)", valuesStr);
    
 
    valuesStr = "";
    for (int index=0; index<avgOfOutOfWaterFlag.getCount(); index++) {
        valuesStr += String(avgOfOutOfWaterFlag.getElement(index)) + ",";
        if (valuesStr.length() > 900) {
            Particle.publish("Test results: avgOfOutOfWaterFlag (partial)", valuesStr);
            valuesStr = "";
            delay(1000);
        }
    }
    valuesStr += "end";
    Particle.publish("Test results: avgOfOutOfWaterFlag (final)", valuesStr);
    
   
}

int cloud_samplePumpReadings(String extra) {
    cloud_samplePump_flag = true;
    return 1;
}

void samplePumpReadings() {
    const int TEST_RUNTIME_MS = 30*1000;
    const int NUM_SUBSAMPLES = 10;
    const int AVG_TRAILING_NUM_SAMPLES = 400;//200;
    //const int NUM_SAMPLES = TEST_RUNTIME_MS/NUM_SUBSAMPLES;
    unsigned int voltageSamples[(unsigned int)ceil((float)TEST_RUNTIME_MS/AVG_TRAILING_NUM_SAMPLES)]; //??
    RunningAverage miniSamples(NUM_SUBSAMPLES); // every 10ms, average 10 samples @ 1ms each // TODO: is this necessary?
    RunningAverage trailingSamples(AVG_TRAILING_NUM_SAMPLES);
    RunningAverage avgOfAvg(10); // sync 10
    float voltageSamplesAvg[(unsigned int)ceil((float)AVG_TRAILING_NUM_SAMPLES/10)]; // sync 10

    // start pump
    rampPump(true);
    /*
    for (int i=0; i<=255; i++) {
        analogWrite(PIN__PUMP_CONTROL, i, 5000); // TODO: test to find best PWM frequency
        delay(2);
    }
    */
    delay(500); // wait for any voltage spikes to pass (?)

    digitalWrite(LED, HIGH);
    unsigned long start = millis();
    int count = 0;
    int i = 0;
    int j = 0;
    int j2 = 0;
    //digitalWrite(LED, HIGH);
    lcd.clear();
    do {
        unsigned long startMicros = micros();
        float vDrop = readMotorVoltageDrop();
        miniSamples.addValue(vDrop);
        if (count % NUM_SUBSAMPLES == 0) {
            float avg = miniSamples.getAverage() * 100000; // this is the average of the last 10 samples (~1ms each)
            trailingSamples.addValue(avg);
            //miniSamples.clear();
//            voltageSamples[i++] = avg;
        }
        
        if (count % AVG_TRAILING_NUM_SAMPLES == 0 && i++ != 0) {
            voltageSamples[i] = trailingSamples.getAverage();
            avgOfAvg.addValue(voltageSamples[i]);
            if (j2++ % 10 == 0) { // sync 10
                voltageSamplesAvg[j++] = avgOfAvg.getAverage();
            }
        }
        
        if (count % 1000 == 0) {
            //lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string(String(count/1000) + "   ");
            //lcd.setCursor(0,1);
            //lcd.send_string(String(voltageSamples[max(0, i-1)]));
        }
    
        
        count++;
        wd->checkin();
        ////delayMicroseconds(max(0, 1000 - (micros() - startMicros))); // wait a total of 1ms
        delay(1);
    } while (millis() - start <= TEST_RUNTIME_MS);
    digitalWrite(PIN__PUMP_CONTROL, LOW);
    digitalWrite(LED, LOW);
    RGB.control(false);
    
    // log values
    String valuesStr;
    valuesStr.reserve(1024);

    valuesStr = "";
    for (int index=0; index<i; index++) {
        valuesStr += String(voltageSamples[index]) + ",";
        if (valuesStr.length() > 900) {
            Particle.publish("Test results: voltageSamples (partial)", valuesStr);
            valuesStr = "";
            delay(1000);
        }
    }
    valuesStr += "end";
    Particle.publish("Test results: voltageSamples (final)", valuesStr);
    
    valuesStr = "";
    for (int index=0; index<i; index++) {
        valuesStr += String(voltageSamplesAvg[index]) + ",";
        if (valuesStr.length() > 900) {
            Particle.publish("Test results: voltageSamplesAvg (partial)", valuesStr);
            valuesStr = "";
            delay(1000);
        }
    }
    valuesStr += "end";
    Particle.publish("Test results: voltageSamplesAvg (final)", valuesStr);
    
}














