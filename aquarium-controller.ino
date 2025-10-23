/** Hardware TODOs:
 * [URGENT] Solder cutoff ball valve to be physically pulled high so it's in the correct state if IO exp is disabled -- ACTUALLY URGENT SINCE SAFEMODE LEAVES VALVE IN WRONG STATE
 *    [x] Then update emergencyIODisable() to disable mcp2 again
 *    [] Decide if/how/when to handle CO2 errors galore when IO2 force-disabled
 */








/**
 * EMI TODOs:
 * [] add cap to mixing station RST line -- should expect large spikes/surges potentially
 * [x] add cap (at least) to any/all interrupt pins, and test by unplugging/replugging the canister filter over and over
 *    [x] consider expanded RC filter for water leak sensor so it needs continuous signal to trigger
 * [x] consider moving GPIO used for water topoff solenoid to relay, w/o requiring 10K resistor (?)
 * [] consider ~50-100 ohm termination resistors for ethernet lines
 * [] consider additional caps anywhere else on main board(s)
 * 
 */

// **TODO** Search for "volatile", and ensure that when reading any variable (> 1 byte) that can be updated by an ISR or other thread, you wrap with noInterrupts() and interrupts() -- see https://arduino.stackexchange.com/questions/20994/why-the-need-to-use-the-volatile-keyword-on-global-variables-when-handling-inter

/**
 * I2C troubleshooting checklist
 * [] Wire up pushbuttons to reset I2C for testing the utility room LCD screen
 * [] Try removing other I2C devices (e.g. 2nd IO expander) -- see if too many pull-up resistors
 * [] Check to see if you added pull-up resistors to the Particle board -- consider removing
 * [] Check for pull-up resistors on utility room side
 * [] Try new firmware branch (5.x)
 * 
 * 
 * 
 */


// TODO: Try and repro/fix issue where topoff solenoid closing causes main board IO expander to go nuts, opening the flow cutoff electric ball valve until the HW WDT fires

// ****TODO***** Figure out why button press happened by itself (IO exp issue?)

// TODO: "If you have larger variables or structures that are shared with interrupt handlers, you should disable interrupts when updating them from the loop. "

// **TODO** State machine with centralized logic/etc for PIN__MIXING_STATION_IO_EXP_DISABLE, etc
// *TODO* Thoroughlly test starting up and "safing" all pumps/etc
// **TODO** Consider manual button to force-reset the downstairs IO board
// **TODO** Test HW WDT to make sure it still works. IMPT: Create nightly or weekly check to ensure that WDT reset line actually works (in case the wire gets disconnected)
// TODO: Look into 5.3x release with HW WDT support, and https://docs.particle.io/reference/product-lifecycle/long-term-support-lts-releases/ in general
// **TODO** Make sure mcp2 is wrapped with lock(), and not called from interrupts (e.g. CO2 bubble class, etc)
// *TODO* If mcp2 isn't initializxed, CO2 warning during CO2 after-hours loops forever (and resetting into safe mode doens't seem to work :-()
// *TODO* thread to monitor for overflows and instantly restart in "stop the flooding!" mode
// *TODO* add locks to RTC alarm module and LCD I2C calls
    // update: this went horribly wrong ::facepalm::. consider higher-level custom mutext to manage all I2C access

// TODO: try using filesystem to write detailed log messages, especially just before forcing a system restart. see https://docs.particle.io/reference/device-os/firmware/#file-system
// left off: change "CO2BubbleSensor.h" to  "CO2BubbleSensor_v2.h" and thoroughly test each change; then add in averaging
// TODO: reset MCU if HW watchdog petting fails (see "void pet() {" in that class)
// TODO: should Waveshare_LCD1602_RGB calls be wrapped in WITH_LOCK to prevent I2C issues?
// TODO: figure out why loop() is taking so long (~1000ms) to complete
// TODO: set WDT to long, but non-inactive value when firware flashing starts



// TODO: Pushbutton toggles "tank cleaning" mode, where topoff and injection tasks(?) are disabled
// *TODO*: after water top-off complete, must reset CO2 stats to avoid false positive on "flow too low" fault. also, try slowing pump down to lower dB level
// TODO: watchdog and firmware update
// TODO: report error if handler can't reset the watchdog
// TODO: optimize HW watchdog logic in rampPump()

#include "buttons.h"
#include <RTClibraryDS3231_DL.h>
#include "CircularBuffer.h"
#include <RunningAverage.h>
#include <DS18B20.h>
#include <Adafruit_MCP23017.h>
#include "RelayModule.h"
#include "ControllerIOExp.h"

// comment out line below to enable piezo buzzer
//#define DISABLE_BEEPS true

// custom reset reasons. NOTE: sync this with resetReasonToString()
#define RESET_REASON_HW_WATCHDOG_PET_FAILED             111
#define RESET_REASON_IO_EXP_WRITE_READ_MISMATCH         112 // TODO: consider storing requested pin in persistentmemory. NOTE: this can keep resetting the device if the mixing station board is just powered off. code with this in mind.
#define RESET_REASON_WATER_LEAK_DETECTED                113
#define RESET_REASON_IO_EXP_READ_MISMATCH               114
#define RESET_REASON_MIXING_STATION_DEVICE_ENERGIZED    115
#define RESET_REASON_CO2_ATTACH_IRQ_FAILED              116
#define RESET_REASON_BALL_VALVE_UNEXPECTED_STATE        117

#define DOSING_PUMP_1_MILLILITERS 1.0 // directions: 1ml per 10 gallons every day or other day
#define DOSING_PUMP_1_START_HOUR 9
#define DOSING_PUMP_1_REPEAT_DAYS 1

#define DOSING_PUMP_2_MILLILITERS 0.8
#define DOSING_PUMP_2_START_HOUR 9
#define DOSING_PUMP_2_REPEAT_DAYS 1

#define DOSING_PUMP_1_SECONDS_PER_ML 0.9 // was .92 but system was crashing??? // approx based on measurement (old pump was 1.15)
#define DOSING_PUMP_2_SECONDS_PER_ML 0.9
#define DOSING_PUMP_3_SECONDS_PER_ML 0.9 // TODO: calibrate with distilled water and no backpressure first, then finally with actual backpressure as a final check

#define TOPOFF_SENSOR_FULL_THRESHOLD_NORMAL 250
#define TOPOFF_SENSOR_FULL_THRESHOLD_FILLING 10
#define TOPOFF_SENSOR_INTERVAL_NORMAL 250 //ms
#define TOPOFF_SENSOR_SAMPLES_NORMAL 20
#define TOPOFF_SENSOR_INTERVAL_FILLING 100 //ms
#define TOPOFF_SENSOR_SAMPLES_FILLING 10

#define ACTIVE_LOW LOW
#define ACTIVE_HIGH HIGH

//#include "Waveshare_LCD1602_RGB.h"
//Waveshare_LCD1602_RGB lcd(16,2);  //16 characters and 2 lines of show
//#include "LiquidCrystal_I2C_Spark.h"
//LiquidCrystal_I2C *lcd2;
#include "LCDUniversal.h"
#include "Utilities.h"
LCDUniversal lcd;
//int r,g,b,t=0;


#include "CoolingPump.h"
CoolingPump coolingPump;

const uint8_t COOLING_TEMP_SENSOR_TANK[8]  = { 0x28, 0x90, 0x42, 0x04, 0x00, 0x00, 0x00, 0xCB };
const uint8_t COOLING_TEMP_SENSOR_INLET[8] = { 0x28, 0xA2, 0xC5, 0x03, 0x00, 0x00, 0x00, 0x54 };
const uint8_t COOLING_TEMP_SENSOR_OUTLET[8]   = { 0x28, 0xFF, 0x57, 0x02, 0x00, 0x00, 0x00, 0x8B };

//rgb_lcd lcd;

// TODO: fix issue where pressing the pushbutton runs the topoff pump (!)


// TODO: "The default drive strength on Gen 3 devices is 2 mA per pin. This can be changed to 9 mA using pinSetDriveStrength()."
// TODO: fix topoff logic: see "if (avg < 0.75) {"
// TODO: log/fix issue where MCU was stalled out when CO2 was clearly off, LCD was showing "normal mode" stats, but bubble rate was shown as 0.62 BPS
// TODO: set bubble count to 0 (or similar) when pausing interrupts so as to avoid false fault

/* 
Important note on Particle software timers (https://docs.particle.io/reference/device-os/api/software-timers/software-timers/)

The timer callback is similar to an interrupt - it shouldn't block. However, it is less restrictive than an interrupt. If the code does block, the system will not crash - the only consequence is that other software timers that should have triggered will be delayed until the blocking timer callback function returns.

- You should not use functions like Particle.publish from a timer callback.
- Do not use Serial.print and its variations from a timer callback as writing to Serial is not thread safe. Use Log.info instead.
- It is best to avoid using long delay() in a timer callback as it will delay other timers from running.
- Avoid using functions that interact with the cellular modem like Cellular.RSSI() and Cellular.command().
- Software timers run with a smaller stack (1024 bytes vs. 6144 bytes). This can limit the functions you use from the callback function.

Timer accounting (so far):
- main code: 0
- Co2BubbleSensor: 3
- HWWatchdog: 1
- Scheduler class: 1 each (x3 eventual instances?)

*/

/*

Latest priorities
=================

Urgent
------
[] Repro and fix issue where topoff pump running (in "adding a bit extra" mode) deadlocks the entire MCU (ignoring the watchdog timer) with LED set to HIGH. 
   I suspect that this is triggered by bubble interrupt and PWM timer conflicting?

Hardware
--------
[] Split to two power supplies, with protection for 5V line via existing regulator
[] Confirm correct capacitor types around MCU
[] Confirm that power MOSFET gate is correctly pulled low
[] Prototype an external watchdog solution
[] Implement external watchdog solution

Software
--------
[] Do not allow topoff pump to run for more than ___ seconds; notify if it does
CO2:
[] Implement averaging for bubble check
[] Implement notifications for bubble check averages (warnings vs. stopage)
[] Implement scheduling for CO2

Watchdog timers:
[] Implement dynamic watchdog timers for all pump control code


*/






// ***TODO*** CONSIDER SETTING WATCHDOG TO DOSING INTERVAL (wd = new ApplicationWatchdog(WATCHDOG_TIMEOUT_MS....)



// TODO: https://learn.sparkfun.com/tutorials/capacitors/application-examples
// https://upperbound.com/projects/555-watchdog-timer/
// TODO: https://techexplorations.com/guides/arduino/common-circuits/bypass-decoupling-capacitor/
    /*
When to use a decoupling capacitor?
In practice, when would you use a decoupling capacitor?
If your circuit contains a microcontroller or something similarly fast switching, then always include a small ceramic decoupling capacitor of around 0.1μF connected very close to that fast switching component's Vcc and GND pins. The capacitor will take care of the noise.
If your circuit contains a component that occasionally draws a big burst of current, like a motor or a transmitter, then add a larger bypass capacitor (say, between 10μF to 50μF). 
This Atmel guide recommends that you include decoupling capacitors to all Vcc-GND pairs and also provides the appropriate values and locations (on the PCB) for these capacitors.
    */
// TODO: https://lowpowerlab.com/shop/product/147 or similar




#define TARGET_DEVICE__ID_ENDING "58dda032f"

#define PIN__CO2_SENSOR_IN      D2 // 3.3v. NOTE: added a 2.2uF ceramic capacitor across signal and ground to address (95% of?) EMI issues caused when canister pump disconnected/connected. next step is to significantly shorten the runtime of the ISR handler, which when coupled with the capacitor completely mitigate the EMI-induced hanging of the MCU.
//#define PIN__CO2_SOLENOID       A4 // active high
#define PIN__DOSING_PUMP_1      A5 // active high
#define PIN__DOSING_PUMP_2      D13 // active high
#define PIN__DOSING_PUMP_3      D11 // active high

#define PIN__TEMP_PROBE_IN      D4
#define PIN__TEMP_PROBE_OUT     D5
#define PIN__HEATER_RELAY       A0 // was D6, but either that GPIO port or the wiring seemed to prevent high-impedience from actually being high-impedience
#define PIN__FLOW_SENSOR        D8 // note: 5V data pin routed through voltage divider

#define PIN__PIEZO_BUZZER       D3

/*#define PIN__PUMP_CONTROL       A0
#define PIN__PUMP_V_SENSE       A1*/
//#define PIN__TOPOFF_FLOW_METER  D6 // was A0 -- see important note about D6 above
#define PIN__COOLER_TEMP_SENSORS  D6 // was A0 -- see important note about D6 above // note: hard-wired 4.7k pullup
#define PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE  A2
#define PIN__LOWER_LIQUID_LEVEL_SENSOR_CAPACITIVE  A3 // new connector pinout (looking top down onto PCB with buzzer to the left): signal, +5V, GND (top to bottom)
#define PIN__WATER_LEAK_SENSOR  A1 // active high (note to test: make sure you bridge contacts e.g. with hands; don't just touch one to make the test LED light up)
#define PIN__PUSHBUTTON         D12 // active low. new connector pinout (looking top down onto PCB with buzzer to the left): GND, signal (top to bottom)
////////#define PIN__PUSHBUTTON_2       MODE//D20 // connected through MD pin; already has pullup resistor built in
#define PIN__DRAIN_BALL_VALVE   A4 // HIGH opens the drain valve
#define PIN__MIXING_STATION_IO_EXP_DISABLE  D9 // HIGH disables the IO expansion board (activates octocoupler which sends an isolated LOW to disables IO expander for mixing station (effectively shuts its peripherals off)
#define PIN__IO_EXP_2_DISABLE   D10 // LOW disables IO expander connected to main board (effectively shuts its peripherals off)

#define LED                     D7

// "For the MCP23017 you specify a pin number from # 0 to 15 for the GPIOA0...GPIOA7, GPIOB0...GPIOB7 pins (i.e. pin 12 is GPIOB4)."
// I.e. Pin number 0..7 corresponds to A0..A7, and 8..15 corresponds to B0..B7
// outputs
#define PIN_IOEXP2__TOPOFF_SOLENOID         9 //B1
#define PIN_IOEXP2__CUTOFF_BALL_VALVE        8 //B0 // LOW closes the cutoff valve
#define PIN_IOEXP2__CANISTER_PUMP_RELAY    15 //B7
#define PIN_IOEXP2__C02_SOLENOID            10 //B2
#define PIN_IOEXP2__PUSHBUTTON_2            2 //A2
#define PIN_IOEXP2__LED_1                   3 //A3
#define PIN_IOEXP2__LED_2                   5 //A5
#define PIN_IOEXP2__LED_3                   7 //A7
//#define PIN_IOEXP2__UNUSED_OUTPUT           11 //B3 -- unused two-pin output header
#define PIN_IOEXP2__COOLER_PUMP           11 //B3 -- unused two-pin output header
#define PIN_IOEXP2__INTERNAL_TEST_GPIO      0 // A0 reserved for internal use

#include "SharedUtilities.h"
#include "PushNotification.h"
#include "SimpleScheduler.h"
#include "FlowRateSensor.h"
#include "CO2BubbleSensor_v3.h"
#include "IOExpansionBoard.h"
#include "MixingStationIO.h"

// note: 255 seems way too bright
#define LCD_RGB_NORMAL 64, 64, 64 // TODO: change brightness multiplier based on time of day // 16, 16, 16 // TODO: dedupe with LCDUniversal
#define LCD_RGB_WARNING 32, 16, 16 // TODO: dedupe with LCDUniversal
#define LCD_RGB_PREWATERING 16, 16, 32
#define LCD_RGB_WATERING 16, 16, 32
#define LCD_RGB_SUCCESS 0, 16, 0
#define LCD_RGB_CLEANING_MODE 64, 40, 0

//#include "WCL_WatchDog.h"
//#define WCL_WATCHDOG_TIMEOUT_MS 5000


//WCL_WatchDog hardwareWatchdog;


// TODO: consider moving into more elegant classes
bool waterDetectedOpticalSensor() {
    return false; // note: no longer using this sensor
    //return false; // TEMPORARY: This sensor has algae or something and seems to falsely trigger (or, the capacitive sensor is positioned too high up)
    //return (digitalRead(PIN__LIQUID_LEVEL_SENSOR_OPTICAL) == false); // active low
}






volatile bool flowHeaterShutoffDisabled = true; // for now while flow rate sensor is stuck








volatile bool isSimulationMode = false;
volatile bool simulation_capacitiveSensorTriggered = false;

 
SYSTEM_THREAD(ENABLED);
STARTUP(startup());

#define HW_WATCHDOG_TIMEOUT_SEC 16 // note: this may not work if >= 60 seconds (may need to use different alarm flag for minutes vs seconds)
#define HW_WATCHDOG_PET_INTERVAL_SEC 5 // note: this may not work if >= 60 seconds (may need to use different alarm flag for minutes vs seconds)
#define SW_WATCHDOG_TIMEOUT_MS 1000*30 //1000*60*2
#define SW_WATCHDOG_STACK_SIZE 3072 //1536 // 1536 recommended as minimum stack size. what's the max?
#include "HWWatchdog.h"
// TODO: retained vaiable to capture last reason we might have reset
ApplicationWatchdog *wd; // TODO: rename
HWWatchdog hwWatchdog(HW_WATCHDOG_TIMEOUT_SEC, HW_WATCHDOG_PET_INTERVAL_SEC);

        
void swWatchdogHandler() {
    // Do as little as possible in this function, preferably just
    // calling System.reset().
    // Do not attempt to Particle.publish(), use Cellular.command()
    // or similar functions. You can save data to a retained variable
    // here safetly so you know the watchdog triggered when you 
    // restart.
    // In 2.0.0 and later, RESET_NO_WAIT prevents notifying the cloud of a pending reset
    
    //digitalWrite(PIN__PUMP_CONTROL, LOW); //co2BubbleSensor.stopCO2();
    //digitalWrite(PIN__CO2_SOLENOID, LOW);
    System.reset(/*currentWatchdogType, */RESET_NO_WAIT);
}

// set DS18B20 data pin and flag as the only sensor on bus
DS18B20 tempSensorIn(PIN__TEMP_PROBE_IN, true);
DS18B20 tempSensorOut(PIN__TEMP_PROBE_OUT, true);
DS18B20 coolerTempSensors(PIN__COOLER_TEMP_SENSORS, false); // two sensors in parallel

FlowRateSensor flowRateSensor;
Adafruit_MCP23017 mcp2;
CO2BubbleSensor_v3 co2BubbleSensor(PIN__CO2_SENSOR_IN, PIN_IOEXP2__C02_SOLENOID, &mcp2);

float getTemp(DS18B20* ds18b20, const uint8_t* addr = nullptr) {
    const int MAX_RETRIES = 3;
    float _temp = NAN;

    if (!ds18b20) return NAN;

    for (int i = 0; i < MAX_RETRIES; i++) {
        if (addr) {
            ds18b20->setAddress((uint8_t*)addr);  // IMPORTANT if using addr
            _temp = ds18b20->getTemperature(addr);
        } else {
            _temp = ds18b20->getTemperature(); // single-drop fallback
        }

        if (ds18b20->crcCheck()) {
            return ds18b20->convertToFahrenheit(_temp);
        }
    }

    return NAN;
}


/*
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
*/

/* ---------- Retained variables ---------- */
// note: for retained variables, the order they're declared here matters -- see https://docs.particle.io/reference/device-os/api/backup-ram-sram/making-changes-to-the-layout-or-types-of-retained-variables/
// also, make sure not to change the variable type which could corrupt the existing data
retained CO2BubbleSensor_v3::Faults lastCO2ControllerFault = CO2BubbleSensor_v3::Faults::NONE;
retained long dosingPump1LastRunEpoch_Retained = -1;
retained bool co2CurrentlyActivated = false;
retained unsigned long temp_milliseconds = 0;
retained bool autoTopoffEnabled = true;
retained long dosingPump2LastRunEpoch_Retained = -1;
retained long dosingPump3LastRunEpoch_Retained = -1;
retained bool uptimeExceededOneMinute = false; // used to enter into safe mode if we reset/hang/etc before one minute of looping
retained long fullWaterChangeLastRunEpoc_Retained = -1;
/* ---------------------------------------- */

// note: now making this a non-retained variable starting off true, which is the value we expect nominally
/*retained*/ bool mixingStationLastConnected = true;

RelayModule relayModule(PIN__HEATER_RELAY, PIN_IOEXP2__CANISTER_PUMP_RELAY, &mcp2);

///MixingStation mixingStation(&lcd);
MixingStationIO mixingStation(PIN__MIXING_STATION_IO_EXP_DISABLE, HIGH, 1 /* A1 */, 0x00, &lcd, &relayModule, &mcp2); // IOExpansionBoard(int disablePin, bool resetPinLogicLevel, int gpioTestPin, byte i2cAddressAddition=0x00) {

void emergencyIODisable(bool turnBackOn=false) {
    if (turnBackOn) {
        //mixingStation.disable();//digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, LOW); 
        digitalWrite(PIN__IO_EXP_2_DISABLE, HIGH);
        //delay(1);
        initializeMcp2();
        //mixingStation.initializeToDisabled();
        mixingStation.enableAndReset();  // TODO: what if returns false?
        delay(1);
    } else {
        mixingStation.disable(); //digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, ACTIVE_HIGH); 
        digitalWrite(PIN__IO_EXP_2_DISABLE, ACTIVE_LOW); 
        delayMicroseconds(5); // force delay in case the system restarts into safe mode right after this. docs say min for reset is 1us, so we use 5 to be safe
    }
}

void startup() {
    // testing: set high drive strength for I2C pins (https://docs.particle.io/assets/images/argon/argon-pinout-v1.0.pdf)
    pinSetDriveStrength(D0, DriveStrength::HIGH);
    pinSetDriveStrength(D1, DriveStrength::HIGH);

    // ensure that pump is always disabled on startup
    pinMode(LED, OUTPUT);
    //pinMode(PIN__PUMP_CONTROL, OUTPUT);
    //digitalWrite(PIN__PUMP_CONTROL, LOW);
    pinMode(PIN__CO2_SENSOR_IN, INPUT_PULLDOWN);
    //pinMode(PIN__CO2_SOLENOID, OUTPUT);
    //digitalWrite(PIN__CO2_SOLENOID, LOW);
    pinMode(PIN__DOSING_PUMP_1, OUTPUT);
    digitalWrite(PIN__DOSING_PUMP_1, LOW);
    pinMode(PIN__DOSING_PUMP_2, OUTPUT);
    digitalWrite(PIN__DOSING_PUMP_2, LOW);
    pinMode(PIN__DOSING_PUMP_3, OUTPUT);
    digitalWrite(PIN__DOSING_PUMP_3, LOW);
    
    pinMode(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE, INPUT);
    pinMode(PIN__LOWER_LIQUID_LEVEL_SENSOR_CAPACITIVE, INPUT);
    pinMode(PIN__WATER_LEAK_SENSOR, INPUT);// now using external pulldown resistor before the newly-added RC low-pass filter
    pinMode(PIN__PUSHBUTTON, INPUT_PULLUP);
    //pinMode(PIN__PUSHBUTTON_2, INPUT);

    pinMode(PIN__DRAIN_BALL_VALVE, OUTPUT);
    digitalWrite(PIN__DRAIN_BALL_VALVE, LOW);
    
    // important: start both IO expansion units in high impedience only mode, for safety
    //mixingStation.initializeToDisabled(); // from docs: "The order of globally constructed objects is unpredictable and you should not rely on global variables being fully initialized."

    /* update: this now set by IO board base class
    pinMode(PIN__MIXING_STATION_IO_EXP_DISABLE, OUTPUT);
    pinSetDriveStrength(PIN__MIXING_STATION_IO_EXP_DISABLE, DriveStrength::HIGH); // to ensure optocoupler LED has enough current
    digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, ACTIVE_HIGH);
    */
    //digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, LOW);
    pinMode(PIN__IO_EXP_2_DISABLE, OUTPUT);
    digitalWrite(PIN__IO_EXP_2_DISABLE, ACTIVE_LOW);
    
    //digitalWrite(PIN__IO_EXP_2_DISABLE, HIGH);

    pinMode(PIN__PIEZO_BUZZER, OUTPUT); 
    pinSetDriveStrength(PIN__PIEZO_BUZZER, DriveStrength::HIGH); // TODO: consider checking for returned success/error code
    
    //pinMode(PIN__TOPOFF_FLOW_METER, INPUT_PULLUP);

    if (digitalRead(PIN__PUSHBUTTON) == ACTIVE_LOW) { // if button is held during power-up, enter into safe mode
        delay(10); // debounce
        if (digitalRead(PIN__PUSHBUTTON) == ACTIVE_LOW) {
            emergencyIODisable();
            System.enterSafeMode(RESET_NO_WAIT);
        }
    }
    if (System.resetReason() == RESET_REASON_PANIC) {
       emergencyIODisable();
       System.enterSafeMode(RESET_NO_WAIT);
    }
}




/*
#define WATCHDOG_TIMEOUT_MS 10*1000
#define WDT_RREN_REG 0x40010508
#define WDT_CRV_REG 0x40010504
#define WDT_REG 0x40010000
#define WDT_RR0_REG 0x40010600
#define WDT_RELOAD 0x6E524635
void WatchDoginitialize() { // https://youtu.be/Xb6dkEHLASU
    *(uint32_t *) WDT_RREN_REG = 0x00000001;
    *(uint32_t *) WDT_CRV_REG = (uint32_t) (WATCHDOG_TIMEOUT_MS * 32.768);
    *(uint32_t *) WDT_REG = 0x00000001;
}
void WatchDogpet() { *(uint32_t *) WDT_RR0_REG = WDT_RELOAD; }

void s etup() {
    
    WatchDoginitialize();
    WatchDogpet();
    RGB.control(true); RGB.color(255,255,0);
    delay(15000); // should trigger watchdog
    RGB.control(false);
*/

void beepBuzzer(int count, int optOnDurationMs=35, int optOffDurationMs=50, int optFrequency=1048) {
    #ifdef DISABLE_BEEPS
        return;
    #endif
    for (int i=0; i<count; i++) {
        tone(PIN__PIEZO_BUZZER, optFrequency); // test tone
        delay(optOnDurationMs);
        noTone(PIN__PIEZO_BUZZER);
        delay(optOffDurationMs);
    }
}

void errorBeep() {
    beepBuzzer(4, 100, 50);
}

#define NOTE_C5  262*2
#define NOTE_D5  294*2
#define NOTE_E5  330*2
#define NOTE_F5  349*2
#define NOTE_G5  392*2
#define NOTE_A5  440*2
#define NOTE_B5  493*2
#define NOTE_C6  NOTE_C5*2

void arpeggioBeep(bool reverse=false) {
    if (!reverse) {
        beepBuzzer(1, 200, 50, NOTE_C5);
        beepBuzzer(1, 200, 50, NOTE_E5);
        beepBuzzer(1, 200, 50, NOTE_G5);
        beepBuzzer(1, 200, 50, NOTE_C6);
    } else {
        beepBuzzer(1, 200, 50, NOTE_C6);
        beepBuzzer(1, 200, 50, NOTE_G5);
        beepBuzzer(1, 200, 50, NOTE_E5);
        beepBuzzer(1, 200, 50, NOTE_C5);
    }
    delay(500);
}

bool lcdDisplayWaitPressButton(const char *line1="", const char *line2="", int countdownSeconds=-1) { // returns true if button was pressed
    bool buttonWasPressed = false;
    //**TODO** Fix and add support for downstairs upper pushbutton
//***TODO*** Wait for all buttons to be released first
    if (line1 != NULL && line1 != "") {
        lcd.setCursor(0,0);
        lcd.send_string("                "); // clear line
        lcd.setCursor(0,0);
        lcd.send_string(String(line1).c_str());
        //lcd.setCursor(strlen(line1),0); 
    } else {
        // leave line blank
    }

    lcd.setCursor(0,1);
    if (line2 != NULL && line2 != "") {
        lcd.send_string("                "); // clear line
        lcd.setCursor(0,1);
        lcd.send_string(String(line2).c_str());
    } else {
        if (countdownSeconds <= 0) {
            lcd.send_string("Press button... ");
        } else {
            lcd.send_string("Press btn...    "); // leave room for digits
        }
    }

    // make sure all buttons are released
    while (!allButtonsReleased()) {
        Particle.process();
        delay(10);
    };

    if (countdownSeconds <= 0) {
        while (!anyButtonPressed()) {
            Particle.process();
            delay(10);
        }
        buttonWasPressed = true;
    } else {
        unsigned long start = millis();
        unsigned long lastLcdUpdate = 0;
        do {
            if (lastLcdUpdate == 0 || millis() - lastLcdUpdate >= 1000) {
                lcd.setCursor(13,1);
                lcd.send_string(String::format("%d  ", countdownSeconds - (int)((millis() - start)/1000.0)));
                lastLcdUpdate = millis();
            }
            Particle.process();
            delay(10);
            
            if (anyButtonPressed()) {
                buttonWasPressed = true;
            }
        } while (!buttonWasPressed && millis() - start <= (countdownSeconds + 1)*1000); // add a bit of extra time to get to zero
    }
    if (line1 != NULL && line1 != "") { // testing
        lcd.setCursor(0,0);
        lcd.send_string("                ");
    }
    lcd.setCursor(0,1);
    lcd.send_string("                ");
    delay(100);
    
    // testing: if button was pressed, wait for it to be released
    if (buttonWasPressed) {
        while (anyButtonPressed()) {
            Particle.process();
            delay(10);
        }
    }
    
    return buttonWasPressed;
}

void lcdDisplayWaitPressButton(int countdownSeconds) {
    lcdDisplayWaitPressButton(NULL, NULL, countdownSeconds);
}

enum WaterChangeAutomationLevel {
    NOT_AUTOMATED = 0, // requires button press at each step
    SEMI_AUTOMATED, // requires button press at each major step
    FULLY_AUTOMATED // runs start to finish automatically; adds more conservative checks as well
};

enum WaterChangeMode {
    FULL_WATER_CHANGE = 0,
    SKIP_MIXING_TANK_FILL, // used for ??
    SIPHON_CLEANING_MODE
    //~~~~SKIP_DRAINING_AQUARIUM // used for siphon cleaning mode
    // note: if adding any other changes, make sure to re-check how other enums are being tested for equality or inequality
};

bool runFullWaterChangeAutomatic(WaterChangeMode, WaterChangeAutomationLevel);
bool runFullWaterChangeAutomatic(WaterChangeMode, WaterChangeAutomationLevel, bool);
bool mixingStation_drainAquarium(int, bool);

volatile bool tmp_enable_petHWWatchdog = true;

SimpleScheduler startCO2Task(
    "Start CO2", // task name
    CO2BubbleSensor_v3::getStartHour(), // start hour
    0,  // start minute
    24,  // repeat interval (hours)
    SimpleScheduler::DayOfWeek::ANY, // day of week to limit to
    startCO2Task_handler, // function to execute 
    0 // EEPROM ID
);
void startCO2Task_handler() {
    co2BubbleSensor.startCO2();
    //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] CO2 start task activated""\" }", PRIVATE);
}

SimpleScheduler stopCO2Task(
    "Stop CO2", // task name
    CO2BubbleSensor_v3::getStopHour(), // start hour
    0,  // start minute
    24,  // repeat interval (hours)
    SimpleScheduler::DayOfWeek::ANY, // day of week to limit to
    stopCO2Task_handler, // function to execute 
    1 // EEPROM ID
);
void stopCO2Task_handler() {
    co2BubbleSensor.stopCO2();
    //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] CO2 stop task activated""\" }", PRIVATE);
}

SimpleScheduler dosingPump1(
    "Dosing pump 1 (Flourish Excel)", // task name
    DOSING_PUMP_1_START_HOUR, // start hour
    0,  // start minute
    DOSING_PUMP_1_REPEAT_DAYS*24,  // repeat interval (hours)
    SimpleScheduler::DayOfWeek::ANY, // day of week to limit to
    dosingPump1Task_handler, // function to execute 
    6, // EEPROM ID
    true // enabled
);
void dosingPump1Task_handler() {
    //bool startDosingPumpSafe(int dosingPumpNum, float doseMilliliters, long &lastRunEpoch_retained)
    startDosingPumpSafe(1, DOSING_PUMP_1_MILLILITERS, dosingPump1LastRunEpoch_Retained);
    //Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Dosing pump #1 activated for %.1f mL""\" }", DOSING_PUMP_1_MILLILITERS), PRIVATE);
}

SimpleScheduler dosingPump2(
    "Dosing pump 2 (fertalizer)", // task name
    DOSING_PUMP_2_START_HOUR, // start hour
    0,  // start minute
    DOSING_PUMP_2_REPEAT_DAYS*24,  // repeat interval (hours)
    SimpleScheduler::DayOfWeek::ANY, // day of week to limit to
    dosingPump2Task_handler, // function to execute 
    7, // EEPROM ID
    true // enabled
);
void dosingPump2Task_handler() {

    //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] About to run dosing pump #2...""\" }", PRIVATE);
    //lcdDisplayWaitPressButton("Ready to dose...", "", 40);

    //bool startDosingPumpSafe(int dosingPumpNum, float doseMilliliters, long &lastRunEpoch_retained)
    startDosingPumpSafe(2, DOSING_PUMP_2_MILLILITERS, dosingPump2LastRunEpoch_Retained);
    //Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Dosing pump #2 activated for %.1f mL""\" }", DOSING_PUMP_2_MILLILITERS), PRIVATE);
}

SimpleScheduler automaticWaterChange(
    "Automated water change", // task name
    17, // start hour
    15, //0,  // start minute
    24,  // repeat interval (hours)
    SimpleScheduler::DayOfWeek::SUNDAY, // day of week to limit to
    automaticWaterChange_handler, // function to execute 
    8, // EEPROM ID
    true // enabled
);

void automaticWaterChange_handler() {
    //automaticWaterChange.resetLastRunEEPROM();
    runFullWaterChangeAutomatic(WaterChangeMode::FULL_WATER_CHANGE, WaterChangeAutomationLevel::FULLY_AUTOMATED, true); // will send push notification
    
    //PushNotification::send("Testing automatic water change scheduler handler. Resetting lastRun EEPROM value.");
    // TODO: skip if water changed too recently?
}

/*SimpleScheduler testTask1(
    "Test task 1 (hourly / 11am)", // task name
    11, // start hour
    0,  // start minute
    1,  // repeat interval (hours)
    testTask1_handler, // function to execute 
    2 // EEPROM ID
);
SimpleScheduler testTask2(
    "Test task 2 (every 3h / 9:07am)", // task name
    9, // start hour
    07,  // start minute
    3,  // repeat interval (hours)
    testTask2_handler, // function to execute 
    3 // EEPROM ID
);
SimpleScheduler testTask3(
    "Test task 3 (every 24h / 8:05am)", // task name
    8, // start hour
    05,  // start minute
    24,  // repeat interval (hours)
    testTask3_handler, // function to execute 
    4 // EEPROM ID
);
SimpleScheduler testTask4(
    "Test task 3 (every 48h / 8:10am)", // task name
    8, // start hour
    10,  // start minute
    48,  // repeat interval (hours)
    testTask4_handler, // function to execute 
    5 // EEPROM ID
);*/

/*void testTask1_handler() {
    Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Test task 1 activated""\" }", PRIVATE);
};
void testTask2_handler() {
    Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Test task 2 activated""\" }", PRIVATE);
};
void testTask3_handler() {
    Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Test task 3 activated""\" }", PRIVATE);
};
void testTask4_handler() {
    Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Test task 4 activated""\" }", PRIVATE);
};*/

int tmp_lastPerfElapsedTimeMs = 0; // note: cloud variables don't seem to support unsigned ints

char mixingStationStrBuff[64]; // https://forum.arduino.cc/t/load-new-string-into-char-array/44473/8, https://docs.particle.io/firmware/best-practices/code-size-tips/#statically-allocated-ram


bool tmp__ioTestEnable = false;
int tmp_numChecks = 0;
int tmp_numChecksFailed = 0;

unsigned long firstConnectedMillis = 0;

void waterLeakHandler() {
    // testing
    //delayMicroseconds(10);
    if (pinReadFast(PIN__WATER_LEAK_SENSOR)) {
        pinSetFast(PIN__MIXING_STATION_IO_EXP_DISABLE); //digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, ACTIVE_HIGH); 
        pinResetFast(PIN__IO_EXP_2_DISABLE); //digitalWrite(PIN__IO_EXP_2_DISABLE, ACTIVE_LOW);
        //delayMicros(1); // update: don't want to take any chances delaying in an ISR which could have any chance of system hangs in such a critical piece of code // old note:  IO expander spec sheet says reset requires a 1us delay, which I assume is safe enough for an ISR
        System.reset(RESET_REASON_WATER_LEAK_DETECTED, RESET_NO_WAIT);
        // for testing
        /*
        if (pinReadFast(PIN__WATER_LEAK_SENSOR)) {
            RGB.control(true); 
            RGB.color(255,0,0);
        } else {
            RGB.control(false); 
        }
        */
    }
}

void initializeMcp2() {
    WITH_LOCK(Wire) { 
        mcp2.begin(0x01);
        mcp2.pinMode(PIN_IOEXP2__TOPOFF_SOLENOID, OUTPUT);
        mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, LOW);
        mcp2.pinMode(PIN_IOEXP2__CUTOFF_BALL_VALVE, OUTPUT);
        mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, HIGH); // keep open by default
        mcp2.pinMode(PIN_IOEXP2__C02_SOLENOID, OUTPUT); // TODO: should this happen both here and in the CO2 bubble controller?
        mcp2.digitalWrite(PIN_IOEXP2__C02_SOLENOID, LOW);
        mcp2.pinMode(PIN_IOEXP2__COOLER_PUMP, OUTPUT); // currently unused
        mcp2.digitalWrite(PIN_IOEXP2__COOLER_PUMP, LOW);
        mcp2.pinMode(PIN_IOEXP2__INTERNAL_TEST_GPIO, OUTPUT); 
        mcp2.digitalWrite(PIN_IOEXP2__INTERNAL_TEST_GPIO, LOW); // impt: always keep low for internal validation checks
        mcp2.pinMode(PIN_IOEXP2__LED_1, OUTPUT); 
        mcp2.digitalWrite(PIN_IOEXP2__LED_1, HIGH);
        mcp2.pinMode(PIN_IOEXP2__LED_2, OUTPUT); 
        mcp2.digitalWrite(PIN_IOEXP2__LED_2, LOW);
        
        // inputs
        mcp2.pinMode(PIN_IOEXP2__PUSHBUTTON_2, INPUT);
        mcp2.pullUp(PIN_IOEXP2__PUSHBUTTON_2, HIGH);
    }
}

const char* resetReasonToString() {
    
    int resetReason = System.resetReason();
    uint32_t resetData = System.resetReasonData();
    
    if (resetReason == RESET_REASON_USER) { // int?
        switch (resetData) { // uint32_t
            case RESET_REASON_HW_WATCHDOG_PET_FAILED:
                return "RESET_REASON_HW_WATCHDOG_PET_FAILED";
            case RESET_REASON_IO_EXP_WRITE_READ_MISMATCH:
                return "RESET_REASON_IO_EXP_WRITE_READ_MISMATCH";
            case RESET_REASON_WATER_LEAK_DETECTED:
                return "RESET_REASON_WATER_LEAK_DETECTED";
            case RESET_REASON_IO_EXP_READ_MISMATCH:
                return "RESET_REASON_IO_EXP_READ_MISMATCH";
            case RESET_REASON_MIXING_STATION_DEVICE_ENERGIZED:
                return "RESET_REASON_MIXING_STATION_DEVICE_ENERGIZED";
            case RESET_REASON_CO2_ATTACH_IRQ_FAILED:
                return "RESET_REASON_CO2_ATTACH_IRQ_FAILED";
            case RESET_REASON_BALL_VALVE_UNEXPECTED_STATE:
                return "RESET_REASON_BALL_VALVE_UNEXPECTED_STATE";
        }
    } else {
        switch (resetReason) {
            case RESET_REASON_PIN_RESET:
                return "RESET_REASON_PIN_RESET";
            case RESET_REASON_POWER_MANAGEMENT:
                return "RESET_REASON_POWER_MANAGEMENT";
            case RESET_REASON_POWER_DOWN:
                return "RESET_REASON_POWER_DOWN";
            case RESET_REASON_POWER_BROWNOUT:
                return "RESET_REASON_POWER_BROWNOUT";
            case RESET_REASON_WATCHDOG:
                return "RESET_REASON_WATCHDOG";
            case RESET_REASON_UPDATE:
                return "RESET_REASON_UPDATE";
            case RESET_REASON_UPDATE_TIMEOUT:
                return "RESET_REASON_UPDATE_TIMEOUT";
            case RESET_REASON_FACTORY_RESET:
                return "RESET_REASON_FACTORY_RESET";
            case RESET_REASON_SAFE_MODE:
                return "RESET_REASON_SAFE_MODE";
            case RESET_REASON_DFU_MODE:
                return "RESET_REASON_DFU_MODE";
            case RESET_REASON_PANIC:
                return "RESET_REASON_PANIC";
            case RESET_REASON_USER:
                return "RESET_REASON_USER";
            case RESET_REASON_UNKNOWN:
                return "RESET_REASON_UNKNOWN";
            case RESET_REASON_NONE:
                return "RESET_REASON_NONE";
        }
    }
    
    return String::format("Reset reason: %d. Reset data: %zu", resetReason, resetData).c_str();
}

bool mixingStationConnected;
uint8_t warningBits = 0x00;

Timer dstTimer(1000*60*60, SharedUtilities::checkAndAdjustDST);

volatile bool setting__disableFlowRateSensor = false;

double getCoolingInF()  { return static_cast<double>(coolingPump.getInletTempF()); }
double getCoolingOutF() { return static_cast<double>(coolingPump.getOutletTempF()); }
double getCoolingTankF() { return static_cast<double>(coolingPump.getTankTempF()); }
 
double tempInF = -1;
double tempOutF = -1;
double flowRate = -1;

void detectAndPublishCoolerTempSensors() {
    const int NUM_COOLER_SENSORS = 3;
    uint8_t sensorAddresses[NUM_COOLER_SENSORS][8]; // testing

    coolerTempSensors.resetsearch();

    char msg[512];  // Safe and fixed buffer
    size_t offset = 0;

    for (int i = 0; i < NUM_COOLER_SENSORS; i++) {
        uint8_t addr[8];
        
        if (coolerTempSensors.search(addr)) {
            double temp = coolerTempSensors.getTemperature(addr);
            double tempF = coolerTempSensors.convertToFahrenheit(temp);

            // Format address as string
            char addrStr[17]; // 8 bytes * 2 chars + null
            snprintf(addrStr, sizeof(addrStr),
                     "%02X%02X%02X%02X%02X%02X%02X%02X",
                     addr[0], addr[1], addr[2], addr[3],
                     addr[4], addr[5], addr[6], addr[7]);

            offset += snprintf(msg + offset, sizeof(msg) - offset,
                               "Sensor %d (0x%s): %.4fF  ", i, addrStr, tempF);
        } else {
            offset += snprintf(msg + offset, sizeof(msg) - offset,
                               "Sensor %d: Not found  ", i);
        }
    }

    msg[sizeof(msg) - 1] = '\0';  // defensive null terminator
    Particle.publish("cooler temp sensors", msg, PRIVATE);
}


void setup() {
    
    SharedUtilities::checkDeviceId(TARGET_DEVICE__ID_ENDING);

    System.enableFeature(FEATURE_RESET_INFO);
    System.enableFeature(FEATURE_RETAINED_MEMORY);

    // start off SW watchdog with extra long duration as a precaution
    wd = new ApplicationWatchdog(SW_WATCHDOG_TIMEOUT_MS*10, swWatchdogHandler, SW_WATCHDOG_STACK_SIZE);
    
    
    mixingStation.initializeToDisabled(); // immediately initialize and set to disabled state. note that we can't do this in startup() b/c globally constructed objects are not always initialized by then (resulting in system hang)
    
    bool waterLeakWasDetected = (System.resetReason() == RESET_REASON_USER && System.resetReasonData() == RESET_REASON_WATER_LEAK_DETECTED);

/* update: disabling as this was making dev/debugging difficult
    // testing: in case we keep looping from setup() to a system reset without calling loop
    // note: do this before starting HW watchdog
    if (System.resetReason() != RESET_REASON_POWER_DOWN && System.resetReason() != RESET_REASON_UPDATE && !uptimeExceededOneMinute && !waterLeakWasDetected) {
        // reset IO expansion boards (although pins will reset in safe mode)
        emergencyIODisable();
        // wait up to xxx seconds to send push notification, then enter safe mode
        RGB.control(true); 
        RGB.color(RGB_COLOR_ORANGE);
        wd->dispose(); // disable SW WDT since waitFor() won't pet it
        if (waitFor(Particle.connected, 2*60*1000)) { // *NOTE* This does not pet the SW WDT (not sure about HW WDT)
            Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Reset too early. About to enter safe mode.""\" }"), WITH_ACK);
        }
        System.enterSafeMode(RESET_NO_WAIT);
    } else {
        uptimeExceededOneMinute = false; // reset flag
    }
*/

    System.on(firmware_update, reset_handler);
    // TODO: System.on(out_of_memory, outOfMemoryHandler); // see https://docs.particle.io/firmware/best-practices/code-size-tips/#out-of-memory-handler

    // set up HW RTC watchdog
    // note: RTC seems to reset early (randomly?) if this is called after waiting to connect to the particle cloud
    hwWatchdog.initialize(); // sets hwWatchdog::rtcInitialized depending on success

    // init CO2 controller ASAP to ensure solenoid always defaults to "off"
    co2BubbleSensor.initialize(); // NOTE: does not work if called in startup() vs setup() TODO: init() as part of constructor?

    flowRateSensor.init(); // TODO: init() as part of constructor?



// enable IO expansion boards
//***TODO*** dedupe with inidivual calls to enable boards (along with error checking if they don't start up)
//TODO: mixingStation.enableAndReset();
    emergencyIODisable(true);
    // emergencyIODisable(bool turnBackOn=false) {

    relayModule.initalize(); // turns on heater and canister filter by default. note: for now, must be called after emergencyIODisable()



    // testing
    // NOTE: the topoff pump causes noise which triggers this interrupt when running
    //attachInterrupt(PIN__PUSHBUTTON, pushbuttonHandler, FALLING); // update: pushing the button sounds a tone and, for some reason, also runs the topoff pump at low speed

    // temp: IO expansion board 2
    // note: initialize the board asap to start in "safe" state
//**TODO** check for success; otherwise CO2 code currently goes nuts with perceived errors    
///    initializeMcp2(); // note: already done via emergencyIODisable(true); above

    lcd.init();
    lcd.clear();
    lcd.setCursor(0,0);

    bool sendWaterLeakFalseAlarmAlert = false;
    float waterLeakDetectionPercent = -1;
    
    if (waterLeakWasDetected) {
        emergencyIODisable(); // disable both IO expansion boards
        LEDStatus blinkOrange(RGB_COLOR_ORANGE, LED_PATTERN_BLINK, LED_SPEED_FAST, LED_PRIORITY_IMPORTANT);
        LEDStatus blinkRed(RGB_COLOR_RED, LED_PATTERN_BLINK, LED_SPEED_FAST, LED_PRIORITY_IMPORTANT);
        blinkOrange.setActive(true);
        lcd.setRGB(LCD_RGB_WARNING);
        lcd.send_string("LEAK DETECTED");
        errorBeep();
        errorBeep();
        lcd.setCursor(0,1);
        lcd.send_string("Confirming...");

        // will this auto-send once connected and online?
        // note: sending a critical push alert here (but "generic" below, in case the code path for "critical" isn't working)
        //particle::Future<bool> publishFuture = Particle.publish("push-notification", String::format("{ \"type\": \"send-message-critical\", \"message\": \"[Aquarium controller] ALERT: Water leak detected. Confirming now...""\" }"), PRIVATE);                
        particle::Future<bool> publishFuture = PushNotification::send("ALERT: Water leak detected. Confirming now...", true); //send(const char* message, bool criticalAlert=false) // testing
        
        // see if we're getting a consisten reading over the course of about 1 second
        int numDetections = 0;
        int numNonDetections = 0;
        for (int i=0; i<100; i++) {
            if (pinReadFast(PIN__WATER_LEAK_SENSOR)) {
                numDetections++;
            } else {
                numNonDetections++;
            }
            delay(10);
        }
        waterLeakDetectionPercent = (float)numDetections / ((float)numDetections + numNonDetections);
        if (waterLeakDetectionPercent < 0.25) { // update: saw instance where legit water triggering wire sensors on tank only registered as 0.52
            blinkOrange.setActive(false);
            lcd.setRGB(LCD_RGB_NORMAL);
            lcd.setCursor(0,1);
            lcd.send_string(String::format("No leak: %.2f ", waterLeakDetectionPercent));
            sendWaterLeakFalseAlarmAlert = true; // we won't be online at this point, so our attempt to publish() using particle::Future just times out
            delay(2000);
            // turn IO boards back on
            emergencyIODisable(true);
            // will resume the rest of setup() from here...
        } else {
            lcd.setCursor(0,1);
            //lcd.send_string(String::format("Confirmed: %.2f ", waterLeakDetectionPercent));
            lcd.send_string(String::format("Conf: %.2f/ ", waterLeakDetectionPercent));
            blinkOrange.setActive(false);
            blinkRed.setActive(true);
// NOTE: this never seems to go out, but the (2) version below does            
            /*particle::Future<bool> publishFuture = Particle.publish("push-notification", // https://docs.particle.io/firmware/low-power/stop-sleep-cellular/#the-future
                String::format("{ \"type\": \"send-message-critical\", \"message\": \"[Aquarium controller] ALERT: Water leak detected. waterLeakDetectionPercent: %.2f""\" }", waterLeakDetectionPercent), 
                WITH_ACK);*/
            particle::Future<bool> publishFuture = PushNotification::send(String::format("[Aquarium controller] ALERT: Water leak detected. waterLeakDetectionPercent: %.2f", waterLeakDetectionPercent), true); //send(const char* message, bool criticalAlert=false) // testing
    
            // new
            RunningAverage runningPercent(20);
            runningPercent.clear();
    
            // drain the water whenever it's detected at the top sensor
            unsigned long pctStartUpdated = millis();
            do {
                runningPercent.addValue(pinReadFast(PIN__WATER_LEAK_SENSOR) ? 1 : 0);
                if (millis() - pctStartUpdated > 500) {
                    lcd.setCursor(12,1);
                    //lcd.send_string(String::format("Confirmed: %.2f ", waterLeakDetectionPercent));
                    lcd.send_string(String::format("%.2f ", runningPercent.getAverage()));
                    pctStartUpdated = millis();
                }
                
                if (isLiquidDetectedTop()) {
                    digitalWrite(PIN__DRAIN_BALL_VALVE, HIGH);
                    digitalWrite(LED, HIGH);
                } else {
                    digitalWrite(PIN__DRAIN_BALL_VALVE, LOW);
                    digitalWrite(LED, LOW);
                }
                if (Particle.connected() && !publishFuture.isSucceeded()) { // TODO: need to test
                    // try again
                    /*
                    publishFuture = Particle.publish("push-notification", // https://docs.particle.io/firmware/low-power/stop-sleep-cellular/#the-future
                        String::format("{ \"type\": \"send-message-critical\", \"message\": \"[Aquarium controller] ALERT: Water leak detected (2). waterLeakDetectionPercent: %.2f""\" }", waterLeakDetectionPercent), 
                        WITH_ACK);
                    */
                    publishFuture = PushNotification::send(String::format("[Aquarium controller] ALERT: Water leak detected. waterLeakDetectionPercent: %.2f", waterLeakDetectionPercent), true); //send(const char* message, bool criticalAlert=false) // testing
                    delay(1000); // TODO: test to make sure we don't get stuck in some kind of infinite loop
                }
                delay(50); //500);
                Particle.process();
            } while(1);
        }
        
    }
    
    bool waterLeakInterruptSuccessful = attachInterrupt(PIN__WATER_LEAK_SENSOR, waterLeakHandler, CHANGE, 0); // setting to highest priority external interrupt (Q: could this affectI2C and make matters worse?guessing at a high (a low number, actually) priority

    ////mixingStation = new MixingStation(lcd);
    
    if (hwWatchdog.rtcInitialized) {
        lcd.setRGB(LCD_RGB_NORMAL);
    } else {
        lcd.setRGB(LCD_RGB_WARNING);
        lcd.send_string("HW WDT error");
        errorBeep();
        delay(3*1000);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.setRGB(LCD_RGB_NORMAL);
    }
    
    // testing: LCD display #2
    /*
    lcd2 = new LiquidCrystal_I2C(0x27, 16, 2);
    lcd2->init();
    lcd2->backlight();
    lcd2->clear();
    lcd2->print("***Spark Time***");
    */

    // testing: I/O expansion board
    //mixingStation.initializeToDisabled(); // now moved to startup()
    bool mixingStationInitSuccess = mixingStation.enableAndReset();
    if (mixingStationInitSuccess) {
        mixingStation.digitalWrite(MixingStationIO::Components::TEST_LED, HIGH);
    }
    

    
    
    
    
    
    
    
    
    
    
    
    

    beepBuzzer(2);
    
    lcd.send_string("Connecting...");

    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);

    /*
    `const int colorR = 0;
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
    const int TIMEOUT_SEC = 120;
    do {
        //Particle.process(); // ??
        lcd.setCursor(0,1);
        lcd.send_string(String::format("%d   ", (int)(TIMEOUT_SEC - (millis() - start)/1000.0)));
        delay(1000);
        
        if (digitalRead(PIN__PUSHBUTTON) == LOW) { // if button is pressed while connecting, reset and enter into safe mode
            emergencyIODisable();
            System.enterSafeMode(RESET_NO_WAIT);
        }
    } while ((millis() - start < TIMEOUT_SEC*1000) && !Particle.connected()); // make sure time is valid before setting up RTC below
    if (!Particle.connected()) {
        System.reset(101);
    } else {
        firstConnectedMillis = millis();
    }
    
    lcd.setCursor(0,1);
    lcd.send_string("                ");

    // TODO: only wait if RTC not set?
    lcd.setCursor(0,0);
    lcd.send_string("Syncing clock...    ");
    start = millis();
    do {
        Particle.process(); // ??
        delay(250);
    } while (millis() - start < 60*1000 && !Time.isValid()); // make sure time is valid before setting up RTC below
    if (!Particle.connected()) {
        System.reset(102);
    }
    Time.zone(-6); 
    Time.setDSTOffset(1);
    dstTimer.start();
    SharedUtilities::checkAndAdjustDST();
    Particle.publish("Current date/time", Time.format(Time.now(), "%Y-%m-%d %H:%M:%S"));

    lcd.setCursor(0,0);
    lcd.send_string("Connected!       ");

    // TODO: see RESET_REASON_HW_WATCHDOG_PET_FAILED, etc
    // see enum of reset reasons ("System_Reset_Reason") here: https://github.com/particle-iot/device-os/blob/73422c305f483d97ac0a2929e00d0053b30cd783/services/inc/system_defs.h
    if (///System.resetReason() != RESET_REASON_USER && 
        ///System.resetReason() != RESET_REASON_PIN_RESET && 
//        System.resetReason() != RESET_REASON_POWER_DOWN && 
        System.resetReason() != RESET_REASON_UPDATE
    ) { // https://docs.particle.io/reference/device-os/api/system-calls/reset-reason/
        //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"Intercom controller reset""\" }", PRIVATE);
    //} else { //if (System.resetReason() == RESET_REASON_PANIC || System.resetReason() == RESET_REASON_POWER_BROWNOUT || System.resetReason() == RESET_REASON_WATCHDOG || System.resetReason() == RESET_REASON_SAFE_MODE || System.resetReason() == RESET_REASON_DFU_MODE) {
        // reason codes: https://github.com/redbear/STM32-Arduino/blob/master/arduino/cores/RedBear_Duo/firmware/hal/inc/core_hal.h
        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Controller reset: %s""\" }", resetReasonToString()), PRIVATE);
        //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Controller reset. Reason code: " + (System.resetReason() == RESET_REASON_USER ? "RESET_REASON_USER" : (System.resetReason() == RESET_REASON_PIN_RESET ? "RESET_REASON_PIN_RESET" : String(System.resetReason()))) + ". Reason data: " + String((uint32_t)System.resetReasonData()) + "\" }", PRIVATE);
    } else {
         //Particle.publish("debug: reset reason", String(System.resetReason()));
         Particle.publish("debug: reset reason", resetReasonToString());
    }
    
    // temporary: if we just restarted after a firmware update, and CO2 was running previously, resume CO2
    if (CO2BubbleSensor_v3::isActiveTimeOfDay()) { // || co2CurrentlyActivated) {
        //if (System.resetReason() == RESET_REASON_UPDATE || System.resetReason() != RESET_REASON_PIN_RESET) {
        if (lastCO2ControllerFault != CO2BubbleSensor_v3::Faults::BPS_TOO_HIGH && lastCO2ControllerFault != CO2BubbleSensor_v3::Faults::BUBBLE_DETECTION_STUCK) {
            co2BubbleSensor.startCO2();
            //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Restarting CO2 after restart""\" }", PRIVATE);
        } else {
            Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Warning: Skipping scheduled CO2 after unexpected system restart: %s. lastCO2ControllerFault: %d""\" }", resetReasonToString(), lastCO2ControllerFault), PRIVATE);
        }
    }
    
    if (sendWaterLeakFalseAlarmAlert) {
        PushNotification::send(String::format("[Aquarium controller] WARNING: Water leak signal was too weak; disregarding. waterLeakDetectionPercent: %.2f", waterLeakDetectionPercent), true);
        /*Particle.publish("push-notification", 
            String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Water leak signal was too weak; disregarding. waterLeakDetectionPercent: %.2f""\" }", waterLeakDetectionPercent), 
            WITH_ACK);*/
        delay(1000);
    }
    
    // old test code
/*
    if (rtcInitialized) {
        if (rtcModule.lostPower()) {
////////            rtcModule.adjust(DateTime(Time.now())); // testing: does this line cause the HW RTC to reset the argon right away for some strange reason?
            Particle.publish("debug", "debug: set RTC time");
        }
        
        // set the hardware watchdog timer and start the timer to pet it continuously
        rtcInitialized &= rtcModule.setAlarm1(rtcModule.now() + TimeSpan(HW_WATCHDOG_TIMEOUT_SEC), DS3231_A1_Second);

// left off here: commenting out the above, then using cloud funciton to "arm", seems to work. why?
*/
/*
    // schedule an alarm 10 seconds in the future
    rtcModule.setAlarm1(
            rtcModule.now() + TimeSpan(10),
            DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
    );
*/    
/*    digitalWrite(LED, HIGH);
    delay(8*1000);
    rtcModule.setAlarm1(
            rtcModule.now() + TimeSpan(10),
            DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
    );
    delay(9999*1000);

    }
*/
    
    if (!hwWatchdog.rtcInitialized) {
        //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *ERROR* Could not connect to HW RTC watchdog. TODO: Implement error handling.\" }", PRIVATE);    
        PushNotification::send("*ERROR* Could not connect to HW RTC watchdog. TODO: Implement error handling.", true); //send(const char* message, bool criticalAlert=false) // testing
        errorBeep();
    }
    
    if (!waterLeakInterruptSuccessful) {
        //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *ERROR* Could not attach water leak detection interrupt. TODO: Implement error handling.\" }", PRIVATE);    
        PushNotification::send("*ERROR* Could not attach water leak detection interrupt. TODO: Implement error handling.", true); //send(const char* message, bool criticalAlert=false) // testing
        errorBeep();
    }
    //Particle.publish("debug", "debug: RTC test: " + String(rtcModule.now().unixtime()) + " <> " + String(Time.now()));

    Particle.publishVitals(60*60); // publish vitals every hour
    
    Particle.variable("tempInF", tempInF);
    Particle.variable("tempOutF", tempOutF);
    Particle.variable("flowRate", flowRate);
    Particle.variable("cooling_inF",  getCoolingInF);
    Particle.variable("cooling_outF", getCoolingOutF);
    Particle.variable("cooling_tankF", getCoolingTankF);

    //Particle.variable("lastPerfMs", tmp_lastPerfElapsedTimeMs);
    //Particle.variable("tmp_numChecksFailed", tmp_numChecksFailed);
    
    Particle.function("bypassFlowSensorTgl", cloud_bypassFlowSensorTgl);
    Particle.function("toggleTopoff", cloud_toggleTopoff);
    Particle.function("getTempIn", cloud_getTempIn);
    Particle.function("forceTopoffCheck", cloud_addWaterManually);
    //Particle.function("simulationModeToggle", cloud_simulationModeToggle);
    //Particle.function("sim_CapSensorToggle", cloud_sim_CapSensorToggle);
    //Particle.function("toggleFlowShutoff", cloud_toggleFlowShutoff); // flowHeaterShutoffDisabled
    //Particle.function("testPWMFreq", cloud_testPWMFreq);
    Particle.function("toggleCO2", cloud_toggleCO2);
    //Particle.function("IOTest1", cloud_ioTest1);
    Particle.function("sensorTestToggle", cloud_sensorTestToggle);
    Particle.function("dosingPump2Ctrl", cloud_dosingPump2Ctrl);
    Particle.function("drainReserv", cloud_drainReserv);
    Particle.function("dosePrime", cloud_dosePrime);
    Particle.function("fillMixingTank_PWD", cloud_fillMixingTank_pwd);
    Particle.function("fullWaterChange_semi_PWD", cloud_fullWaterChange_pwd);
    Particle.function("pumpToAqu_PWD", cloud_mainPumpTest_pwd);
    Particle.function("drainTank_PWD", cloud_drainTank_pwd);
    Particle.function("safeMode_PWD", cloud_safeMode_pwd);
    Particle.function("restart_PWD", cloud_restart_pwd);
    //Particle.function("IOTest2", cloud_ioTest2);
    Particle.function("ioTestToggle", cloud_ioTestToggle);
    Particle.function("tempTest", cloud_tempTest);
    Particle.function("coolingTest", cloud_coolingTest);
    /***********************************************************************************************/
    /** NOTE: These functions can be dangerous and instantly overdose the tank. Use with Caution. **/
    //Particle.function("dosePumpNum", cloud_dosePumpNum);
    //Particle.function("doseForSec", cloud_injectForSec);
    /***********************************************************************************************/
    
    //Particle.function("wdTest", cloud_watchdogTest);
    Particle.function("getCO2BPSx100", cloud_getCO2BPSx100);
    //Particle.function("opticalTest", cloud_opticalTest);
    //Particle.function("deadlockTest", cloud_deadlockTest);
    //Particle.function("HW_WD_toggle", HW_WD_toggle);
    Particle.function("HW_WDT_test", cloud_HW_WD_test);
    //Particle.function("dosing1Test", cloud_dosingPump1Test);
    //Particle.function("schedulerTest1", cloud_schedulerTest1);
    //Particle.function("schedulerTest2", cloud_schedulerTest2);
    //Particle.function("readEEPROMLong", cloud_readEEPROMLong);
    
    //co2BubbleSensor.startMonitoring(); // start CO2 monitoring, so we can pick up any fault flags in the main loop
    

    
    lcd.clear();
    
    RGB.control(true);
    RGB.brightness(12);
    RGB.control(false);

    /*
    // test case: why register not updating?
    Particle.publish("register test 1a", String::format("0x%x", (*(volatile uint32_t *) (WDT_CONFIG_REG)))); // volatile: ????
    ATOMIC_BLOCK() { // ???
        *(volatile uint32_t *) WDT_CONFIG_REG |= 0x08; // why isn't this write being reflected in the subsequent read?
    }
    delay(150);
    Particle.publish("register test 1b", String::format("0x%x", (*(volatile uint32_t *) (WDT_CONFIG_REG))));
    */
    /*
    Particle.publish("register test 1b", String::format("0x%x", (*(uint32_t *) 0x40010404)));
    Particle.publish("debug: 1", String(hardwareWatchdog.isEnabled() ? "yes" : "no"));
    ATOMIC_BLOCK() { // ???
        *(uint32_t *) WDT_RR0_REG = WDT_RELOAD; // pet
        delay(10);
        //*(uint32_t *) WDT_RREN_REG = 0x00000001;
        //delay(1);
        *(uint32_t *) WDT_REG = 0x00000000; // disable
    }
    delay(100);
    Particle.publish("debug: 2", String(hardwareWatchdog.isEnabled() ? "yes" : "no"));
    hardwareWatchdog.initialize(1000);
    delay(100);
    Particle.publish("debug: 3", String(hardwareWatchdog.isEnabled() ? "yes" : "no"));
    delay(3000);
    */
//RoReg
/*
    Particle.publish("register test 1a", String::format("0x%x", (*(uint32_t *) 0x10000000)));
    
    Particle.publish("register test 1b", String::format("0x%x", (*(volatile uint32_t *) 0x40010504)));
    *(volatile uint32_t *) WDT_RREN_REG = 0x00000001;
    *(volatile uint32_t *) WDT_CRV_REG = (uint32_t) 32768;
    delay(150);
    Particle.publish("register test 1b", String::format("0x%x", (*(volatile uint32_t *) 0x40010504)));

    hardwareWatchdog.initialize(1000);
    delay(150);
    Particle.publish("register test 1b", String::format("0x%x", (*(uint32_t *) 0x40010504)));
    //Particle.publish("register test 1", String::format("0x%x", (*(uint32_t *) 0x40010100)));
    //Particle.publish("register test 2", String::format("0x%x", (*(uint32_t *) 0x40010000)));
    //*(uint32_t *) WDT_CONFIG_REG |= 0x08; // why isn't this write being reflected in the subsequent read?
    */
    

    
    //hardwareWatchdog.initialize(1000);
    //delay(1000);
    ////Particle.publish("register test 2a", String::format("0x%x", (*(uint32_t *) 0x4001050C)));
//    Particle.publish("register test 1", String::format("0x%x", (*(uint32_t *) 0x40010504)));
  //  Particle.publish("register test 1", String::format("0x%x", (*(uint32_t *) 0x4001050C)));
    //Particle.publish("register test 3", String::format("0x%x", (*(uint32_t *) 0x40010000)));

    /*
    delay(1000);
    //Particle.publish("WDT_CRV_REG", String::format("%x", (*(RwReg *) WDT_CRV_REG)));
    //Particle.publish("WDT_RREN_REG", String::format("%x", (*(RwReg *) WDT_RREN_REG)));
    //Particle.publish("WDT_CONFIG_REG", String::format("%x", (*(RwReg *) WDT_CONFIG_REG)));
    //Particle.publish("WDT_REG", String::format("%x", (*(RwReg *) WDT_REG)));
    // "WoReg"??
    Particle.publish("WDT_REG", String::format("0x%x", (*(RwReg *) 0x40010000U)));
    Particle.publish("WDT_REG", String::format("%lx", (*(RwReg *) 0x40010000U)));
    //Particle.publish("WDT_REG", String::format("%lx", (*(WoReg *) 0x40010000U)));
    delay(1000);

    Particle.publish("debug: wdt enabled?", String(hardwareWatchdog.isEnabled() ? "yes" : "no"));
    hardwareWatchdog.initialize(1000);
    delay(100);
    Particle.publish("debug: wdt enabled?", String(hardwareWatchdog.isEnabled() ? "yes" : "no"));

    delay(2000);
    Particle.publish("WDT_REG", String::format("0x%x", (*(RwReg *) 0x40010000U)));
    Particle.publish("WDT_REG", String::format("%lx", (*(RwReg *) 0x40010000U)));
    delay(1000);
    */
    
    /*
    Particle.publish("WDT_CRV_REG", String::format("%x", (*(uint32_t *) WDT_CRV_REG)));
    delay(1000);
    Particle.publish("WDT_RREN_REG", String::format("%x", (*(uint32_t *) WDT_RREN_REG)));
    delay(1000);
    Particle.publish("WDT_CONFIG_REG", String::format("%x", (*(uint32_t *) WDT_CONFIG_REG)));
    delay(1000);
    Particle.publish("WDT_REG", String::format("%x", (*(uint32_t *) WDT_REG)));
    */
    
    // reset SW watchdog to normal interval
    wd->dispose();
    wd = new ApplicationWatchdog(SW_WATCHDOG_TIMEOUT_MS, swWatchdogHandler, SW_WATCHDOG_STACK_SIZE);

    if (mixingStationInitSuccess) {
        // send quick signal
        mixingStation.digitalWrite(MixingStationIO::Components::TEST_LED, LOW);
        delay(100);
        mixingStation.digitalWrite(MixingStationIO::Components::TEST_LED, HIGH);
        delay(100);
        mixingStation.digitalWrite(MixingStationIO::Components::TEST_LED, LOW);
        delay(100);
        beepBuzzer(1);
    } else {
        // TODO: go into an error state?
        //digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, ACTIVE_HIGH); // TODO: centralize
        mixingStation.disable();
        errorBeep();
        //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Unable to connect to mixing station. DISABLING IO UNIT.""\" }", PRIVATE);
    }

    
    // string testing
    /*
    strncpy(mixingStationStrBuff, "1234567890", sizeof(mixingStationStrBuff)-1);
    Particle.publish("str test 1", mixingStationStrBuff);
    delay(1500);
    strncpy(mixingStationStrBuff, "abcdefghijklmnopqrstuvwxyz", sizeof(mixingStationStrBuff)-1);
    Particle.publish("str test 2", mixingStationStrBuff);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.send_string("test: " + String(mixingStationStrBuff)); // works
    lcd.setCursor(0,1);
    //lcd.send_string(String::format("test 2: %s", String(mixingStationStrBuff))); // doesn't work
    lcd.send_string(String::format("test 2: %s", String(mixingStationStrBuff).c_str())); // works
    delay(20*1000);
    */
    
    // testing
    /*
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.send_string("test 1");
    lcdDisplayWaitPressButton();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.send_string("testing 2");
    lcdDisplayWaitPressButton(10);
    */
    
    mixingStationConnected = mixingStation.isConnected();
    
    detectAndPublishCoolerTempSensors();
    coolingPump.begin(&coolerTempSensors,
                      COOLING_TEMP_SENSOR_INLET,
                      COOLING_TEMP_SENSOR_OUTLET,
                      COOLING_TEMP_SENSOR_TANK,
                      &mcp2,
                      PIN_IOEXP2__COOLER_PUMP,
                      PIN_IOEXP2__LED_3,
                      temp_isButton1Pressed,   // << existing helpers
                      temp_isButton2Pressed,
                      &lcd);
}

void pushbuttonHandler() {
    tone(PIN__PIEZO_BUZZER, 1048, 20); // test tone
}

char bufferLine1[16];

#define LOG_TO_SPREADSHEET_TIME_MS  60*60*1000
unsigned long timeSinceLogToSpreadsheet = 0;

#define LCD_UPDATE_TIME_MS  1000

#define WATER_LEVEL_CHECK_TIME_MS  30*1000
system_tick_t timeSinceWaterLevelCheck = 0;

#define HEATER_DISABLE_COOLDOWN_MS  1000*60*5
unsigned long heaterLastDisabled = 0;

#define MAX_WATERING_TIME_MS    1000*30 // note: be careful here in case a sensor were to fail
#define PUMP_EXTRA_TIME_MS      1000*2
#define WATER_TOPOFF_COOLDOWN_TIME_MS   1000*60*60
unsigned long waterLastToppedOff = 0;


unsigned long heaterEnableTimestamp = 0; // set to a value of millis() that will reenable the heater
String heaterDisableReason;


void rampPump(bool startVsStop, int optPWMFreq=20000, bool optTest1=false) { // note: 1KHz results in audiable tone when pump is running at low speeds
    /*
    if (startVsStop) {
        for (int i=100; i<=255; i++) {
            analogWrite(PIN__PUMP_CONTROL, i, optPWMFreq); // TODO: test to find best PWM frequency
//RGB.color(i % 2 == 0 ? 255 : 0, 255, 0);
            delay(5);
        }
    } else {
        for (int i=255; i>=100; i--) {
            analogWrite(PIN__PUMP_CONTROL, i, optPWMFreq); // TODO: test to find best PWM frequency
//RGB.color(i % 2 == 0 ? 255 : 0, 255, 0);
            delay(4);
        }
        digitalWrite(PIN__PUMP_CONTROL, LOW);
    }
    */
    
    WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, startVsStop); }
}

RunningAverage topoffSensorSamples_normal(TOPOFF_SENSOR_SAMPLES_NORMAL);
unsigned long topoffSensorLastSampled_normal = 0;

RunningAverage topoffSensorSamples_filling(TOPOFF_SENSOR_SAMPLES_FILLING);
unsigned long topoffSensorLastSampled_filling = 0;

enum TopoffSensorModes {
    NORMAL,
    FILLING
};

void sampleTopoffSensorTimed(TopoffSensorModes mode) {
    if (mode == TopoffSensorModes::NORMAL) {
        if (millis() - topoffSensorLastSampled_normal >= TOPOFF_SENSOR_INTERVAL_NORMAL) {
            topoffSensorLastSampled_normal = millis();
            topoffSensorSamples_normal.addValue(analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE)); // normal value seems to be ~3100 when no contact with wqter    
        }
    } else if (mode == TopoffSensorModes::FILLING) {
        if (millis() - topoffSensorLastSampled_filling >= TOPOFF_SENSOR_INTERVAL_FILLING) {
            topoffSensorLastSampled_filling = millis();
            topoffSensorSamples_filling.addValue(analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE)); // normal value seems to be ~3100 when no contact with wqter    
        }
    }
}

void resetTopoffSensorAverage(TopoffSensorModes mode) {
    if (mode == TopoffSensorModes::NORMAL) {
        topoffSensorSamples_normal.clear();
    } else if (mode == TopoffSensorModes::FILLING) {
        topoffSensorSamples_filling.clear();
    }
}

bool topoffSensorWaterDetected(TopoffSensorModes mode) {
    if (isSimulationMode) {
        return simulation_capacitiveSensorTriggered;
    } else {
        if (mode == TopoffSensorModes::NORMAL) {
            return (topoffSensorSamples_normal.getCount() > 0 && topoffSensorSamples_normal.getCount() == topoffSensorSamples_normal.getSize() && topoffSensorSamples_normal.getAverage() <= TOPOFF_SENSOR_FULL_THRESHOLD_NORMAL);
        } else if (mode == TopoffSensorModes::FILLING) {
            return topoffSensorSamples_filling.getCount() > 0 && (topoffSensorSamples_filling.getCount() == topoffSensorSamples_filling.getSize() &&  topoffSensorSamples_filling.getAverage() <= TOPOFF_SENSOR_FULL_THRESHOLD_FILLING);
        } else {
            return false;
        }
        //return (topoffSensorSamples_normal.getAverage() < 1000);
        //return (analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE) < 2900); // normal value seems to be ~3100 when no contact with wqter
    }
}

bool doWaterLevelCheck(bool optSkipSensorTest=false) {
    const int LCD_UPDATE_INTERVAL_MS = 500;
    unsigned long lcdLastUpdated = 0;
    unsigned long lastWaterLastToppedOff;

    // see if water is below fill line
    if (optSkipSensorTest || (!topoffSensorWaterDetected(TopoffSensorModes::NORMAL) && relayModule.isHeaterOn() && (waterLastToppedOff == 0 || millis() - waterLastToppedOff > WATER_TOPOFF_COOLDOWN_TIME_MS))) {
        // note: for some reason the system keeps filling up as soon as it restarts
        lcd.setRGB(LCD_RGB_PREWATERING);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.send_string("Checking water...");
        Particle.publish("doWaterLevelCheck() debug", String::format("topoffSensorSamples_normal.getAverage(): %.0f, topoffSensorSamples_filling.getAverage(): %.0f", topoffSensorSamples_normal.getAverage(), topoffSensorSamples_filling.getAverage()));
        
        // TODO: clean up if you keep this hack
        for (int i=0; i<10; i++) {
            sampleTopoffSensorTimed(TopoffSensorModes::NORMAL);
            sampleTopoffSensorTimed(TopoffSensorModes::FILLING);
            delay(500);
        }
        Particle.publish("doWaterLevelCheck() debug 2", String::format("topoffSensorSamples_normal.getAverage(): %.0f, topoffSensorSamples_filling.getAverage(): %.0f", topoffSensorSamples_normal.getAverage(), topoffSensorSamples_filling.getAverage()));
        
        if (topoffSensorWaterDetected(TopoffSensorModes::NORMAL) || topoffSensorWaterDetected(TopoffSensorModes::FILLING)) {
            lcd.setCursor(0,1);
            lcd.send_string("Water detected");
            delay(2000);
            lcd.setRGB(LCD_RGB_NORMAL);
            return true;
        }

        // temp
        /*
        Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] About to top off water in aquarium...""\" }", PRIVATE);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.send_string("Ready to add H2O");
        lcdDisplayWaitPressButton(60);
        */

        lcd.setRGB(LCD_RGB_WATERING);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.send_string("Adding water...");
        
        // turn on pump
        unsigned long start = millis();
        lastWaterLastToppedOff = waterLastToppedOff;
        waterLastToppedOff = millis();

// TODO: any watchdog changes to make here?
        //co2BubbleSensor.stopInterrupts(); // required to avoid system deadlock of some kind when rampPump() is called
        rampPump(true);
    
        bool outOfWater = false;
        unsigned long elapsedTime;
        unsigned long fillStartTime = millis();
        unsigned long fillStopTime;
        resetTopoffSensorAverage(TopoffSensorModes::FILLING);

        do {
// TODO: check that reservoir isn't out of water at any point        
            elapsedTime = millis() - start;
            
            // sample topoff sensor
            sampleTopoffSensorTimed(TopoffSensorModes::FILLING);

            // update LCD
            if (millis() - lcdLastUpdated > LCD_UPDATE_INTERVAL_MS) {
                lcd.setCursor(0,1);
                lcd.send_string(String::format("Raw avg: %.0f", topoffSensorSamples_filling.getAverage()) + "     ");
                lcdLastUpdated = millis();
            }
            
            if (topoffSensorWaterDetected(TopoffSensorModes::FILLING)) {
                fillStopTime = millis();
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.send_string("Adding a bit");
                lcd.setCursor(0,1);
                lcd.send_string("extra...");
                delay(PUMP_EXTRA_TIME_MS); // note: seeing a hang here or (more likely) in the rampPump() call below

                rampPump(false);

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
                    (fillStopTime-fillStartTime)/1000.0
                ));
                
                delay(2000);
                break;
            }

            wd->checkin(); // pet the SW watchdog
            //hwWatchdog.pet(); // this should continue via the background timer
            Particle.process();

        } while (elapsedTime < MAX_WATERING_TIME_MS && !outOfWater);

//co2BubbleSensor.resetMonitoringStats(); // important to ensure that gaps in data readings don't cause false positive fault detections
//co2BubbleSensor.startInterrupts();
        
        // turn off pump (possibly duped, but that's okay)
        //digitalWrite(PIN__PUMP_CONTROL, LOW);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, LOW); }

        if (outOfWater) {
            lcd.setRGB(LCD_RGB_WARNING);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string("Out of water");
            
            Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Topoff reservoir is out of water""\" }", PRIVATE);
            // TODO: publish to sheets before returning
            
            delay(4000);
            return false;
        } else if (elapsedTime >= MAX_WATERING_TIME_MS) {
            lcd.setRGB(LCD_RGB_WARNING);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string("Water level sen");
            lcd.setCursor(0,1);
            lcd.send_string("sor not tripped");
            
            Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Water top-off stopped after maximum amount of time without sensor tripped. Elapsed time: " + String(elapsedTime/1000) + " seconds)""\" }", PRIVATE);
            //delay(2000);
            return false;
        } else {
            Particle.publish("info", String::format("Topoff completed in %.1f seconds. Last topoff was %.1f hours ago. debug: %d, %d", (fillStopTime-fillStartTime)/1000.0, (fillStopTime-lastWaterLastToppedOff)/1000.0/60.0/60.0, fillStopTime, lastWaterLastToppedOff), PRIVATE);
            //Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Topoff completed in %.1f seconds. Last topoff was %.1f hours ago. debug: %d, %d""\" }", (fillStopTime-fillStartTime)/1000.0, (fillStopTime-lastWaterLastToppedOff)/1000.0/60.0/60.0, fillStopTime, lastWaterLastToppedOff), PRIVATE);
        }
        
        lcd.setRGB(LCD_RGB_NORMAL);
        resetTopoffSensorAverage(TopoffSensorModes::NORMAL);
        return true;
    }    
    
    return false;
}

volatile bool flag__coolingPumpTest = false;

volatile bool cloud_samplePump_flag = false;
volatile bool flag_doWaterLevelCheck = false;

volatile int flag_injectForSecPumpNum = -1; // pump num to dose
volatile float flag_injectForSec = 0; // dosing rate measured at 1.25 seconds per mL for old pump


bool firmwareUpdateBegun = false;

float getSecondsPerMl(int pumpNum) {
    switch (pumpNum) {
        case 1:
            return DOSING_PUMP_1_SECONDS_PER_ML;
        case 2:
            return DOSING_PUMP_2_SECONDS_PER_ML;
        case 3:
            return DOSING_PUMP_3_SECONDS_PER_ML;
        default:
            return -1;
    };    
}

float getDosingPumpPin(int pumpNum) {
    switch (pumpNum) {
        case 1:
            return PIN__DOSING_PUMP_1;
        case 2:
            return PIN__DOSING_PUMP_2;
        case 3:
            return PIN__DOSING_PUMP_3;
        default:
            return -1;
    };    
}

/*
 * Uses retained variable to ensure that the pump has not run for at least 24 hours since the MCU was last powered up (vs persistent storage)
 */
bool startDosingPumpSafe(int dosingPumpNum, float doseMilliliters, long &lastRunEpoch_retained) {
    if (!Time.isValid()) {
        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *Not* running dosing pump %d. Invalid time.""\" }", dosingPumpNum), PRIVATE);                
        return false;
    }

    long now = Time.now();
    
    // handle any data corruption issues (with alert in case this happens more than once)
    if (lastRunEpoch_retained <= 0 && lastRunEpoch_retained != -1) {
        Particle.publish("debug", String::format("Dosing pump %d debug: Setting lastRunEpoch_retained from %ld to -1", dosingPumpNum, lastRunEpoch_retained), PRIVATE);                
        lastRunEpoch_retained = -1;
    }
    float hoursSinceLastRun_Retained = (lastRunEpoch_retained == -1) ? -1 : (now - lastRunEpoch_retained)/60.0/60.0;
    Particle.publish("debug", String::format("Dosing pump %d debug: now: %ld, lastRunEpoch_retained: %ld, hoursSinceLastRun_Retained: %f", dosingPumpNum, now, lastRunEpoch_retained, hoursSinceLastRun_Retained), PRIVATE);

    const float ONE_MINUTE_AS_HOURS = 1/60.0; // bug fix: we need to allow for up to 60 sec of flex since tasks can be called w/in a 60 second window
    if (hoursSinceLastRun_Retained == -1 || (hoursSinceLastRun_Retained >= 24 - ONE_MINUTE_AS_HOURS)) {
        lastRunEpoch_retained = (long)now;

        // get calibrated pump seconds per mL
        float secondsPerMl = getSecondsPerMl(dosingPumpNum);
        if (secondsPerMl <= 0) {
            Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *Not* running dosing pump %d: Unknown pump number""\" }", dosingPumpNum), PRIVATE);                
            return false;
        }

        // TODO: set watchdog, run pump // PIN__DOSING_PUMP_1
        unsigned int dosingPumpDurationMs = secondsPerMl*1000*doseMilliliters;
        //Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Dosing pump %d injecting %.1fmL (%ldms). (hoursSinceLastRun_Retained: %f, new lastRunEpoch_retained: %ld)""\" }", dosingPumpNum, doseMilliliters, dosingPumpDurationMs, hoursSinceLastRun_Retained, lastRunEpoch_retained), PRIVATE);                
        wd->checkin();
        if (hwWatchdog.setOneOffWatchdogAlarmThenDisable__see_notes(dosingPumpDurationMs + 100)) { // include a bit of buffer time
            // get pump pin
            int pumpPin = getDosingPumpPin(dosingPumpNum);
            if (pumpPin != -1) {
                // run pump here // note: disable SW wd for other long-running motor runs?
                digitalWrite(pumpPin, HIGH);
                delay(dosingPumpDurationMs);
                digitalWrite(pumpPin, LOW);
                
                if (!hwWatchdog.enable()) {
                    delay(1000);
                    Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *ERROR* Failed to re-enable watchdog after running dosing pump %d.""\" }", dosingPumpNum), PRIVATE);                
                    System.reset(1234567);
                    return false;
                }
            } else {
                Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *Not* running dosing pump %d. Unknown pin number""\" }", dosingPumpNum), PRIVATE);                
            }
        } else {
            Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *ERROR* Failed to set one-off watchdog timer for dosing pump %d.""\" }", dosingPumpNum), PRIVATE);                
            System.reset(12345678);
            return false;
        }
        
        return true;
    } else {
        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *Not* running dosing pump %d. Hours since last run: %f""\" }", dosingPumpNum, hoursSinceLastRun_Retained), PRIVATE);                
        return false;
    }
}

enum class ControllerModes {
    NORMAL,
    TANK_CLEANING_MODE,
    // TODO: other mode for firmware update?
};
ControllerModes controllerMode = ControllerModes::NORMAL;
//ControllerModes lastControllerMode = ControllerModes::NORMAL;

void updateLCD(float tempInF, float tempOutF, float flowRate, ControllerModes controllerMode = ControllerModes::NORMAL) {
    
    // original working version:
    /*
    lcd.setCursor(0,0);
    lcd.send_string(String(tempInF).substring(0, 5) + (char)223 + "   "); //.concat("      "));
    lcd.setCursor(9,0);
    lcd.send_string(String(tempOutF).substring(0, 5) + (char)223 + "  "); //.concat("      "));
    lcd.setCursor(0,1);
    lcd.send_string(String(flowRate).substring(0, 5) + " GPH  ");
    lcd.setCursor(9,1);
    lcd.send_string("  T" + String(waterDetectedOpticalSensor() ? "+" : "-") + "B" + String(waterDetectedCapacitiveSensorFluctuating() ? "~" : String(waterDetectedCapacitiveSensor() ? "+" : "-"))); // eg " T+, B~"
    */

    // CO2 test version 
    /*
    lcd.setCursor(0,0);
    lcd.send_string("Count: " + String(co2BubbleSensor.bubbleCount) + "   ");
    lcd.setCursor(14,0);
    lcd.send_string("E" + String(co2BubbleSensor.getFaultCode()) + " ");
    lcd.setCursor(0,1);
    //lcd.send_string("Last: " + String(co2BubbleSensor.lastBubbleElapsedTimeMs) + "ms   ");
    lcd.send_string("BpS: " + String(co2BubbleSensor.getBubblesPerSecond()).substring(0, 4) + "    ");
    lcd.setCursor(13,1);
    lcd.send_string(co2BubbleSensor.isCO2Started() ? " On" : "Off");
    //lcd.setCursor(8,1);
    //lcd.send_string("CO2: " + (co2BubbleSensor.getFaultCode() == CO2BubbleSensor_v3::Faults::NONE ? (String(co2BubbleSensor.bubbleCount) + "   ") : ("*E" + String(co2BubbleSensor.getFaultCode()) + "*")));
    */
    
    
    if (warningBits == 0x00) {
///    if (controllerMode == ControllerModes::NORMAL) {
        // combined version
        lcd.setCursor(0,0);
        lcd.send_string(String(tempInF).substring(0, 5) + (char)223 + "  "); //.concat("      "));
    
        lcd.setCursor(8,0);
        if (setting__disableFlowRateSensor) {
            lcd.send_string("*N/A GPH  ");
        } else {
            lcd.send_string(String(flowRate).substring(0, 4) + " GPH  ");
        }

    } else {
        lcd.setCursor(0,0);
        lcd.send_string(String::format("Flags:0b%s          ", String(warningBits, BIN).c_str()));
    }
        
                
        lcd.setCursor(0,1);
        //lcd.send_string(String(tempOutF).substring(0, 5) + (char)223 + "  "); //.concat("      "));
        float topoffAvg = topoffSensorSamples_normal.getAverage();
//////////        lcd.send_string(String::format(topoffAvg < 1000 ? "%.2f" : "%.0f", topoffAvg) + "    ");
//////////lcd.send_string(mixingStation.isConnected() ? "IO conn " : "IO disco");
///lcd.send_string(String::format("IO%s;%d", mixingStation.isConnected() ? "c": "d", tmp_numChecksFailed));

if (mixingStation.debug_digitalReadSafeRetries + mixingStation.debug_digitalWriteSafeRetries + mixingStation.debug_autoRepairSuccesses1 + mixingStation.debug_autoRepairSuccesses2 > 0) {
    lcd.send_string(String::format("%d,%d,%d,%d~", 
        mixingStation.debug_digitalReadSafeRetries, 
        mixingStation.debug_digitalWriteSafeRetries, 
        mixingStation.debug_autoRepairSuccesses1, 
        mixingStation.debug_autoRepairSuccesses2));
} else {
    ////lcd.send_string(String::format(topoffAvg < 1000 ? "%.2f" : "%.0f", topoffAvg) + "    ");
    lcd.send_string(String(tempOutF).substring(0, 5) + (char)223 + "  "); //.concat("      "));
    
}

        lcd.setCursor(8,1);
        lcd.send_string(
            String(co2BubbleSensor.getBubblesPerSecondCombined()).substring(0, 4) + (
                (co2BubbleSensor.getFaultCode() == CO2BubbleSensor_v3::Faults::NONE) ?
                    " BPS  " :
                    (" !E" + String(co2BubbleSensor.getFaultCode()) + "  ")
            )
        );
        /*lcd.send_string(
            (co2BubbleSensor.getFaultCode() == CO2BubbleSensor_v3::Faults::NONE) ?
                (String(co2BubbleSensor.getBubblesPerSecondCombined()).substring(0, 4) + " BPS  ") :
                ("*E" + String(co2BubbleSensor.getFaultCode()) + "*    ")
        );*/
    
        
        
        
        
        // old versions
        //lcd.send_string(tempInF > tempOutF ? "   Idle" : "   Heat"); // TODO: add warning states too
        /*lcd.send_string(String::format("  T%sB%s", 
            String(waterDetectedOpticalSensor() ? "+" : "-"),
            String(waterDetectedCapacitiveSensorFluctuating() ? "~" : String(waterDetectedCapacitiveSensor() ? "+" : "-"))
        ));*/
        //lcd.send_string("   " + String(analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE)) + "    ");    
/*    }
    
    else if (controllerMode == ControllerModes::TANK_CLEANING_MODE) {
        lcd.setCursor(0,1);
        lcd.send_string(String(tempInF).substring(0, 5) + (char)223 + "  "); //.concat("      "));
    
        lcd.setCursor(8,1);
        lcd.send_string(String(flowRate).substring(0, 4) + " GPH  ");
    }
    */
}

/*
#define TOPOFF_SENSOR_INTERVAL_NORMAL = 500; //ms
RunningAverage topoffSensorSamples_normal(10);
unsigned long topoffSensorLastSampled_normal = 0;

#define TOPOFF_SENSOR_INTERVAL_NORMAL = 75; //ms
RunningAverage topoffSensorSamples_filling(5);
unsigned long topoffSensorLastSampled_filling = 0;
*/


/*
bool lowTempAlertNewlyTriggered = false;
void doTempTempChecks_old(float &tempInF, float &tempOutF, float &flowRate) {
    // TODO: refactor where core checks go
    // TODO: clean up w/ states, etc
    // TODO: break out separately for flow rate, e.g. in cooldown time

//****TODO**** make sure we turn heater back on after tripping below. also sanity check all other logic
    if (tempInF > 83 || tempOutF > 90) {
        if (millis() - heaterLastDisabled > HEATER_DISABLE_COOLDOWN_MS) { // TODO: fix this -- it just keeps repeating
            relayModule.setHeater(false);
            heaterLastDisabled = millis();
            lcd.setRGB(LCD_RGB_WARNING);
            beepBuzzer(5, 60, 60); // void beepBuzzer(int count, int optOnDurationMs=35, int optOffDurationMs=50, int optFrequency=1048)
            Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Heater disabled due to high temperature""\" }", PRIVATE);
            delay(2000);
        }
    } 

    if (tempInF < 74 || tempOutF < 74) {
        if (!lowTempAlertNewlyTriggered) {
            lowTempAlertNewlyTriggered = true;
            for (int i=0; i<3; i++) {
                // jiggle the relay in case it's stuck
                relayModule.setHeater(false);
                delay(200);
                relayModule.setHeater(true);
                delay(200);
            }

            // send alert
            static system_tick_t lastLowTempWarning = 0;
            if (lastLowTempWarning == 0 || (millis() - lastLowTempWarning > 5*60*1000)) {
                //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Temperature is too low: " + String(tempInF).substring(0, 5) + """\" }", PRIVATE);
                static PushNotification notification_tempTooLow(15min);
                notification_tempTooLow.sendWithCooldown(String::format("WARNING: Temperature is too low: %.2f", tempInF), true);
                lastLowTempWarning = millis();
            }
                        
            lcd.setRGB(LCD_RGB_WARNING);
            beepBuzzer(5, 60, 60); // void beepBuzzer(int count, int optOnDurationMs=35, int optOffDurationMs=50, int optFrequency=1048)

                tempInF = getTemp(&tempSensorIn);
    
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.send_string("Temp too low: ");
                lcd.setCursor(0,1);
                lcd.send_string(String(tempInF).substring(0, 5) + (char)223 + "   ");
                delay(3000);

        }
    } else {
        lowTempAlertNewlyTriggered = false;
    }

    
    if (!flowHeaterShutoffDisabled && (flowRate < 1.0 && relayModule.isHeaterOn())) {
// TODO: fix !flowHeaterShutoffDisabled &&  above... probably all messed up

        // temp
        
        relayModule.setHeater(false);

        lcd.setRGB(LCD_RGB_WARNING);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.send_string("Low flow rate.");
        lcd.setCursor(0,1);
        lcd.send_string("Heater disabled.");
        
        beepBuzzer(5, 60, 60); // void beepBuzzer(int count, int optOnDurationMs=35, int optOffDurationMs=50, int optFrequency=1048)
        Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Low flow rate; disabling heat temporarily""\" }", PRIVATE);
        
        unsigned long start = millis();
        float lastFlowRate;
        do {
            lastFlowRate =  flowRateSensor.readFlowRateGPH();
            wd->checkin(); // make sure the watchdog doesn't reset us during this potentially long loop
            delay(500);
            Particle.process(); // note: this was needed to prevent cloud functions from not working in this loop
        } while (!flowHeaterShutoffDisabled && (millis() - start < 1000*60*60*2 && lastFlowRate < 20)); // break out if flowHeaterShutoffDisabled flag gets set while in this loop
        
        relayModule.setHeater(true);
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
        relayModule.setHeater(true);
        lcd.setRGB(LCD_RGB_NORMAL);
    }
    
    //updateLCD(tempInF, flowRate);
        
}
*/

unsigned long mixingStationLastBlink = 0;
//bool tmp_ioExpanderTestLastState = LOW;

bool flag__cloud_toggleCO2 = false;
bool flag__cloud_fillMixingTank_pwd = false;
bool flag__cloud_drainReserv = false;
bool flag__cloud_dosePrime = false;
float flag__cloud_dosePrime_optOverrideTimeSeconds = 0;
int flag__cloud_fullWaterChange_pwd = 0;
bool flag__cloud_mainPumpTest_pwd = false;
bool flag__cloud_sensorTestToggle = false;
bool flag__cloud_dosingPump2Ctrl = false;
bool flag__cloud_drainTank_pwd = false;
bool flag__cloud_tempTest = false;

bool flag__flowRateTooLowAfterEnablingCanisterFilter = false;
bool flag__tempTooHigh = false;
bool flag__tempTooLow = false;
bool flag__flowRateTooLow = false;

system_tick_t timeSinceLcdUpdate = 0;

// temp
int displayStrings(int, const char*, ...);

void canisterCleaningMode() {
    const float GALLONS = 1.75;//2.5;
    int itemIndex;
    
    itemIndex = displayStrings(
        2,
        "Start now?",
        "Exit"
    );
    if (itemIndex == 1) {
        return;
    }
            
    static bool wasCO2Active = co2BubbleSensor.isCO2Started();
    lcd.setRGB(LCD_RGB_CLEANING_MODE);
    //lcd.display("Filter cleaning", "mode", 0);
    if (!disableFilterAndHeatWithAlert()) {
        // if we get here, the display will show an error message, a push alert will have been sent, and a global fault flag will have been set
        errorBeep();
        return;
    }
    if (wasCO2Active) {
        co2BubbleSensor.stopCO2();
    }

    // temp: close cutoff valve to help prevent air bubbles when pump is primed
    lcd.display("Closing cutoff", "valve...", 0);
    WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, LOW); } // LOW closes the cutoff valve
    delay(4*1000);

    //lcd.display("Start cleaning", "filter...", 2);
    //lcd.display("while mix tank", "starts filling.", 1);

    bool skipFilling = true;
    itemIndex = displayStrings(
        2,
        "Pre-fill mixing tank",
        "Skip filling"
    );

    bool error = false;
    if (itemIndex == 0) {
        skipFilling = false;
        bool tryAgain = false;
        do {
            bool success = false;
            bool cancelledViaButtonPress = false;
            unsigned long debug_numInvalidReadings = 0;
            
            // drain reservoir
            mixingStation.drainReservoir();

            // ******TODO: dose prime w/ right gallon value****** --> consider adding retry option
            success = mixingStation.dosePrime(GALLONS);
            if (!success) {
                // TODO: implement
                lcd.display("Dosing error", "", 3);
                /*displayStrings(
                    1,
                    ">Dosing error. TODO. Press."
                );*/          
            } else {
                success = mixingStation.dispenseWater(GALLONS, cancelledViaButtonPress, debug_numInvalidReadings); //bool dispenseWater(float totalMixedGallons, bool &cancelledViaButtonPress, unsigned long &debug_numInvalidReadings) {
            }
            
            if (!success || cancelledViaButtonPress || debug_numInvalidReadings > 0) {
                errorBeep();
                
                // debug
                if (cancelledViaButtonPress || debug_numInvalidReadings > 0) {
                    errorBeep();
                    displayStrings(
                        1,
                        String::format("Debug: Error: %d, %d", cancelledViaButtonPress, debug_numInvalidReadings)
                    );
                }
                
                itemIndex = displayStrings(
                    2,
                    "Error. Try again?",
                    "Error. Exit?"
                );
                if (itemIndex == 0) {
                    tryAgain = true;
                } else {
                    tryAgain = false;
                    error = true;
                }
            } else if (success) {
                tryAgain = false;
            }
            
        } while (tryAgain);
    }

    lcd.setRGB(LCD_RGB_NORMAL);
    
    displayStrings(
        1,
      //"                                " // 32 chars
        "Open cutoff valve to prime pump"
    );
    //lcd.display("Open canister", "filter valve...", 3);
    lcd.display("Opening...", "", 0);
    WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, HIGH); } // LOW closes the cutoff valve
    delay(7*1000);

    displayStrings(
        1,
      //"                                " // 32 chars
        "Close cutoff valve to add water"
    );
    //lcd.display("Open canister", "filter valve...", 3);
    lcd.display("Closing...", "", 0);
    WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, LOW); } // LOW closes the cutoff valve
    delay(7*1000);
    
    if (!error) {
        if (skipFilling) {
            displayStrings(
                1,
                "Open cutoff valve & resume"
            );
        } else {
            itemIndex = displayStrings(
                2,
                "Pump water to aquarium",
                "Skip"
            );
            if (itemIndex == 0) {
                bool success = mixingStation.pumpWaterToAquarium(123, isLiquidDetectedTop);
                if (!success) {
                    errorBeep();
                    PushNotification::send(String::format("Warning: Error pumping water to aquarium during canister filter change"));
                }
            }
        }
    }

    /** go back to "normal" mode **/
    
    if (mcp2.digitalRead(PIN_IOEXP2__CUTOFF_BALL_VALVE) == LOW) { // LOW means cutoff valve is closed
        lcd.display("Opening valve...", "", 0);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, HIGH); } // HIGH opens the cutoff valve
        delay(7*1000);
    }
    
    lcd.setRGB(LCD_RGB_NORMAL);
    lcd.display("Resuming...", "", 1);

    if (!enableFilterAndHeatWithAlert()) {
        // if we get here, the display will show an error message, a push alert will have been sent, and a global fault flag will have been set
        errorBeep();
        return;
    }
    if (wasCO2Active && CO2BubbleSensor_v3::isActiveTimeOfDay()) {
        co2BubbleSensor.startCO2();
    }
    
    itemIndex = displayStrings(
        2,
        "Drain reservoir",
        "Skip"
    );
    if (itemIndex == 0) {
        bool success = mixingStation.drainReservoir();
        if (!success) {
            errorBeep();
            PushNotification::send(String::format("Warning: Error draining reservoir"));
        }
    }

    timeSinceLcdUpdate = millis() - LCD_UPDATE_TIME_MS - 1; // force LCD refresh next cycle
    lcd.clear();    
}

ButtonPress temp_isButton1Pressed() {
    
    // check for upstairs button #1
    if (digitalRead(PIN__PUSHBUTTON) == ACTIVE_LOW) {
        return ButtonPress::PRESSED;
    }

    // check for downstairs button #1
    if ((mixingStation.digitalRead(MixingStationIO::Components::PUSHBUTTON_UPPER) == ACTIVE_LOW) && !mixingStation.lastReadErrorFlag && mixingStation.isConnected(false)) {
        // temp sanity check since we had data corruption issues earlier
        delay(10);
        if ((mixingStation.digitalRead(MixingStationIO::Components::PUSHBUTTON_UPPER) == ACTIVE_LOW) && !mixingStation.lastReadErrorFlag && mixingStation.isConnected(false)) {
            return ButtonPress::PRESSED;
        }
    }
    
    return ButtonPress::NOT_PRESSED;
}

ButtonPress temp_isButton2Pressed() {
    // check for upstairs button #2
    
    uint16_t gpioValues;
    WITH_LOCK(Wire) { gpioValues = mcp2.readGPIOAB(); }
    
    if (gpioValues == 0xffff) { // IO board disconnected
        return ButtonPress::READ_ERROR;
    }

    if (bitRead(gpioValues, PIN_IOEXP2__INTERNAL_TEST_GPIO) != LOW) { // unexpected value
        return ButtonPress::READ_ERROR;
    }

    if ((bitRead(gpioValues, PIN_IOEXP2__PUSHBUTTON_2) == ACTIVE_LOW)) {
        return ButtonPress::PRESSED;
    }
    
    // check for downstairs button #2
    if ((mixingStation.digitalRead(MixingStationIO::Components::PUSHBUTTON_LOWER) == ACTIVE_LOW) && !mixingStation.lastReadErrorFlag && mixingStation.isConnected(false)) {
        // temp sanity check since we had data corruption issues earlier
        delay(10);
        if ((mixingStation.digitalRead(MixingStationIO::Components::PUSHBUTTON_LOWER) == ACTIVE_LOW) && !mixingStation.lastReadErrorFlag && mixingStation.isConnected(false)) {
            return ButtonPress::PRESSED;
        }
    }
    
    return ButtonPress::NOT_PRESSED;
}


/*bool anyButtonPressed_debounced() {
    const unsigned long THRESHOLD_MS = 50;
    unsigned long start = millis();
    do {
        
        
    } while (millis() - start < THRESHOLD_MS);
}*/

bool anyButtonPressed() { // TODO: ~debounce
    return (temp_isButton1Pressed() == ButtonPress::PRESSED || temp_isButton2Pressed() == ButtonPress::PRESSED);
    
    
    
    
    
    
//    if (digitalRead(PIN__PUSHBUTTON) == ACTIVE_LOW) {
//        return true;
//    } else {
        /*bool mainButton2Value;
//TODO: mixingStation.lastReadErrorFlag...
        WITH_LOCK(Wire) { mainButton2Value = mcp2.digitalRead(PIN_IOEXP2__PUSHBUTTON_2); }
        if ((mainButton2Value == ACTIVE_LOW || mixingStation.digitalReadSafe(MixingStationIO::Components::PUSHBUTTON_LOWER) == ACTIVE_LOW) && mixingStation.isConnected()) {
            return true;
        }*/
//    }
//    return false;
}

bool allButtonsReleased() { // TODO: ~debounce
    return (temp_isButton1Pressed() == ButtonPress::NOT_PRESSED && temp_isButton2Pressed() == ButtonPress::NOT_PRESSED);

    /*
    bool mainButton2Value;
    WITH_LOCK(Wire) { mainButton2Value = mcp2.digitalRead(PIN_IOEXP2__PUSHBUTTON_2); }
    return (digitalRead(PIN__PUSHBUTTON) == HIGH);/////&& mainButton2Value == HIGH && mixingStation.digitalReadSafe(MixingStationIO::Components::PUSHBUTTON_LOWER) == HIGH);
    */
}

void loop() {
    ////coolingPump.tick();   // <— replaces the flag__coolingPumpTest block
    if (flag__coolingPumpTest) {
        flag__coolingPumpTest = false;
        coolingPump.runPumpCycleWithLogging();
    }
    
    /*
    if (flag__coolingPumpTest) {
        // wait for button release
        while (temp_isButton2Pressed() == ButtonPress::PRESSED) {
            Particle.process();
        }

        static system_tick_t lastTempUpdate = 0;        
        if (lastTempUpdate == 0 || millis() - lastTempUpdate > 3000) {
            lcd.setCursor(0,0);
            lcd.send_string("Btn: Cooling pmp");
            
            mcp2.digitalWrite(PIN_IOEXP2__LED_3, HIGH);
            float coolerTempIn  = getTemp(&coolerTempSensors, sensorAddresses[0]); //getTempFrom64(&coolerTempSensors, COOLER_TEMP_IN_ADDR);
            float coolerTempOut = getTemp(&coolerTempSensors, sensorAddresses[1]);
            mcp2.digitalWrite(PIN_IOEXP2__LED_3, LOW);

            lcd.setCursor(0,1);
            lcd.send_string(
                isnan(coolerTempIn) ? "--" :
                String(coolerTempIn).substring(0, 5) + (char)223 + "  -> ");
            lcd.setCursor(8,1);
            lcd.send_string(
                isnan(coolerTempOut) ? "--" :
                String(coolerTempOut).substring(0, 5) + (char)223 + "     ");
        }
        
        if (temp_isButton2Pressed() == ButtonPress::PRESSED) {
            mcp2.digitalWrite(PIN_IOEXP2__COOLER_PUMP, LOW);
            beepBuzzer(1);
            flag__coolingPumpTest = false;
			// Wait for button release
			while (temp_isButton2Pressed() == ButtonPress::PRESSED) {
    			Particle.process();
			}
            delay(100);
            return;
        }

        bool isButton1Pressed = (temp_isButton1Pressed() == ButtonPress::PRESSED);
        mcp2.digitalWrite(PIN_IOEXP2__COOLER_PUMP, isButton1Pressed); //mcp2.digitalWrite(PIN_IOEXP2__LED_3, LOW);

        Particle.process();
        return;
    }
    */
    
    // **TODO**: safety in case data corruption leads to phantom I2C button presses?
    //**TODO** ensure that heater/filter are on unless we're in a special fault mode where they should stay off

    // update core sensor readings
    static system_tick_t lastSensorUpdate = 0;
    if (lastSensorUpdate == 0 || millis() - lastSensorUpdate > 1000) { // TODO: can we update every 500ms without messing up flow rate data collection?
        lastSensorUpdate = millis();
        tempInF = getTemp(&tempSensorIn);
        tempOutF = getTemp(&tempSensorOut);
        flowRate = flowRateSensor.readFlowRateGPH();
    }    
    
    // update LCD display
    if (millis() - timeSinceLcdUpdate > LCD_UPDATE_TIME_MS) {
        timeSinceLcdUpdate = millis();
        updateLCD(tempInF, tempOutF, flowRate, controllerMode);
    }
    
    // display menu on button press
    // TODO: refactor any/all use of ControllerModes
    if ((temp_isButton1Pressed() == ButtonPress::PRESSED) && (controllerMode == ControllerModes::NORMAL || controllerMode == ControllerModes::TANK_CLEANING_MODE)) {
        // beep and wait for button release
        beepBuzzer(1);
        while (temp_isButton1Pressed() == ButtonPress::PRESSED) {
            Particle.process();
        }

        int itemIndex = displayStrings(
            11,
          //"                                " // 32 chars
            "Gravel siphon  cleaning mode",
            "Canister filter cleaning mode",
            "Full water change (full auto)",
            "Full water change (semi-auto)",
            "Full water change (manual)",
            "Toggle flowrate sensor bypass",
            "Set flag__cloud_tempTest", 
            "Run flow calibration (tap)",
            "Run flow calibration (rodi)",

            "Cooling pump test",

            "Exit"
        );
        
        // TODO: refactor and consider switching to state machine approach
        switch (itemIndex) {
            case 0: 
                siphonCleaningMode();
                break;
            case 1: 
                canisterCleaningMode();
                break;
            case 2: 
                runFullWaterChangeAutomatic(WaterChangeMode::FULL_WATER_CHANGE, WaterChangeAutomationLevel::FULLY_AUTOMATED);
                break;
            case 3: 
                runFullWaterChangeAutomatic(WaterChangeMode::FULL_WATER_CHANGE, WaterChangeAutomationLevel::SEMI_AUTOMATED);
                break;
            case 4: 
                runFullWaterChangeAutomatic(WaterChangeMode::FULL_WATER_CHANGE, WaterChangeAutomationLevel::NOT_AUTOMATED);
                break;
            case 5:
                setting__disableFlowRateSensor = !setting__disableFlowRateSensor;
                lcd.display("Flowrate sensor:", setting__disableFlowRateSensor ? "Disabled" : "Enabled", 3);
                timeSinceLcdUpdate = millis() - LCD_UPDATE_TIME_MS - 1; // force LCD refresh next cycle
                break;
            case 6:
                flag__cloud_tempTest = true;
                break;
            case 7: {
                    // TODO: clean up!
                    bool success = mixingStation.dispenseWaterCalibrationMode(true); // bool useTapInsteadOfRodi
                    if (!success) {
                        success = mixingStation.dispenseWaterCalibrationMode(true); // bool useTapInsteadOfRodi                    
                    } if (!success) {
                        PushNotification::send("Debug: Tap water caliberation failed twice");
                        break;
                    }
                    break;
                }
            case 8: {
                    // TODO: clean up!
                    /*
                    bool success = mixingStation.dispenseWaterCalibrationMode(true); // bool useTapInsteadOfRodi
                    if (!success) {
                        success = mixingStation.dispenseWaterCalibrationMode(true); // bool useTapInsteadOfRodi                    
                    } if (!success) {
                        PushNotification::send("Debug: First caliberation failed twice. Skipping second calibration.");
                        break;
                    }*/
                    
                    bool success = mixingStation.dispenseWaterCalibrationMode(false);
                    if (!success) {
                        success = mixingStation.dispenseWaterCalibrationMode(false); // bool useTapInsteadOfRodi                    
                    } if (!success) {
                        PushNotification::send("Debug: Second calibration failed twice");
                        break;
                    }
                    break;
                }
            case 9:
                lcd.clear(); // reset display
                flag__coolingPumpTest = !flag__coolingPumpTest;
                delay(100); // extra button release debounce
                return; // go right back to looo()
            default:
                return;
        }
    }
    
    // handle special case where canister filter may not be pushing water
    if (flag__flowRateTooLowAfterEnablingCanisterFilter) {
        // TODO: implement and decide if a special mode is warrented
        static PushNotification notification_flowRateTooAfterEnablingCanisterFilter(60min);
        notification_flowRateTooAfterEnablingCanisterFilter.sendWithCooldown("ERROR: Flow rate at or near zero after canister filter re-enabled. Check tank manually ASAP.", true);
        lcd.setRGB(LCD_RGB_WARNING);

        static system_tick_t canisterFilterLastRetry = 0;
        if (canisterFilterLastRetry == 0 || (millis() - canisterFilterLastRetry > 10*60*1000)) { // try every 5 minutes to get filter / flow meter working again
            if (enableFilterAndHeatWithAlert()) { // try again
                relayModule.setHeater(true);
                beepBuzzer(3);
                lcd.setRGB(LCD_RGB_NORMAL);
                lcd.display("Flow rate + heat", "restored", 1);
                flowRate = flowRateSensor.readFlowRateGPH(true);
                PushNotification::send(String::format("Update: Flow rate is now higher (%.2f GPH). Leaving filter on and re-enabling heat.", flowRate), true);
                flag__flowRateTooLowAfterEnablingCanisterFilter = false;
            }/* else {
                flowRate = 0; // set to zero to prevent looping back here until another reading happens
                PushNotification::send(String::format("Update: Flow rate turned on briefly (%.2f GPH), but did not pass threshold to leave canister filter on.", flowRate));
                delay(1000);
            }*/
            canisterFilterLastRetry = millis();
        }
    }
    
    
    
    
    
    
            
        
    // *TODO* Ensure loop runs at least once a minute -- constrain topoff/etc to run faster than that    
    startCO2Task.loop(true, controllerMode == ControllerModes::TANK_CLEANING_MODE);
    stopCO2Task.loop(true, controllerMode == ControllerModes::TANK_CLEANING_MODE);
    dosingPump1.loop(true, controllerMode == ControllerModes::TANK_CLEANING_MODE);
    dosingPump2.loop(true, controllerMode == ControllerModes::TANK_CLEANING_MODE);
    automaticWaterChange.loop(true, controllerMode == ControllerModes::TANK_CLEANING_MODE);

    
    
        
        
    /************* TODO: system state update block *************/

    // set warning bits for important pieces of system state we want to know about
    // TODO: do this every second or so, not every loop()
    warningBits = 0x00;
    if (!mixingStationConnected) { 
        warningBits |= 0b00000001; 
    }
    if (!mixingStation.isEnabled()) { 
        warningBits |= 0b00000010; 
    }
    if (digitalRead(PIN__IO_EXP_2_DISABLE) == ACTIVE_LOW) { // TODO: update
        warningBits |= 0b00000100; 
    }
    if (co2BubbleSensor.getFaultCode() != CO2BubbleSensor_v3::Faults::NONE) {
        warningBits |= 0b00001000; 
    }

    /****************************************************/


    /************* TODO: critical monitoring block *************/

    // TODO: ensure water is flowing
    /*flowRateSensor.readFlowRateGPH() // issue: calling this while the LCD update does the same will sometimes return -1
    static PushNotification notification_co2BackupAlert(10min);
            notification_co2BackupAlert.sendWithCooldown("WARNING: C02 stopped via backup. You should never see this!", true);
    */

    // run temperature checks    
    static system_tick_t lastTempCheck = 0;
    if (millis() - lastTempCheck > 30*1000) {
        // TODO: clean up controller mode stuff    
        if (controllerMode == ControllerModes::NORMAL) {
            doTempTempChecks(tempInF, tempOutF, flowRate);
        }
        lastTempCheck = millis();
    }
    
    // monitor for CO2 faults
    static system_tick_t lastCO2Check = 0;
    if (millis() - lastCO2Check > 30*1000) {
        co2BubbleSensor.updateFaultCodeAndSendAlerts(); // will stop CO2 and send alerts if fault conditions are detected
        lastCO2Check = millis();
        
        // sync a retained bool flag with the state of CO2 activation to ensure continuity between MCU resets (specifically firmware updates)
        if (co2CurrentlyActivated != co2BubbleSensor.isCO2Started()) {
            co2CurrentlyActivated = co2BubbleSensor.isCO2Started();
        }
    }
    
    // warn if flow rate is too low
    if (flag__flowRateTooLow) {
        if (flowRate > 20) {
            // wait and check again to be safe
            lcd.display("Checking flow", "rate...", 0);
            delay(1000);
            float flowRate2 = flowRateSensor.readFlowRateGPH(true);
            if (flowRate2 > 20) {
                flag__flowRateTooLow = false;
                lcd.setRGB(LCD_RGB_NORMAL);
                lcd.display("Flow rate", "restored", 1);
                PushNotification::send(String::format("Update: Flow rate has been restored: %.2f GPH", flowRate2), false); //~~~~~true); // stopping false critical alarms for now
                lcd.clear();
            } else {
                lcd.display(String::format("Too low: %.2f GPH", flowRate2), "", 1);
            }
        }
    } else {
        static system_tick_t lastFlowRateCheck = 0;
        if (millis() - lastFlowRateCheck > 30*1000) {
            if (flowRate < 20) {
                // wait and check again to be safe
                lcd.display("Checking flow", "rate...", 0);
delay(1000); // TODO: consider makig this much longer to reduce false positive alerts
                float flowRate2 = flowRateSensor.readFlowRateGPH(true);
                if (flowRate2 < 20 && !setting__disableFlowRateSensor) {
                    flag__flowRateTooLow = true;
                    static PushNotification notification_flowRateTooAfterEnablingCanisterFilter(60min);
                    notification_flowRateTooAfterEnablingCanisterFilter.sendWithCooldown(String::format("WARNING: Flow rate too low: %.2f GPH.", flowRate2), false); //~~~~~true); // stopping false critical alarms for now
                    lcd.setRGB(LCD_RGB_WARNING);
                    errorBeep();
                    lcd.display(String::format("Too low: %.2f GPH", flowRate2), "", 1);
                }
                lcd.clear();
            }
            lastFlowRateCheck = millis();
        }
    }

// TODO: refactor this section (e.g. use pointer to retained variable, add new IO exp class implementation, etc.)
// end TODO    
    
    static system_tick_t lastMonitoringCheck = 0;
    if (millis() - lastMonitoringCheck > 1000) { // note: don't go under 1 second, as some of these checks are critical

        // ensure that electric ball valves are never in unexpected states in loop()
        if (digitalRead(PIN__DRAIN_BALL_VALVE) == HIGH) {
            // should never be HIGH in loop()
            particle::Future<bool> publishFuture = PushNotification::send(String::format("ERROR: Drain ball valve set to open state in loop(). Restarting..."));
            delay(1000); // wait up to 1 second to send // TODO: log this instead
            System.reset(RESET_REASON_BALL_VALVE_UNEXPECTED_STATE, RESET_NO_WAIT);
    
        }
        bool cutoffValveClosed;
        WITH_LOCK(Wire) { 
            cutoffValveClosed = (mcp2.digitalRead(PIN_IOEXP2__CUTOFF_BALL_VALVE) == LOW);
        }
        if (cutoffValveClosed) {
            // should never be closed in loop()
            particle::Future<bool> publishFuture = PushNotification::send(String::format("ERROR: Cutoff ball valve set to closed state in loop(). Restarting..."));
            delay(1000); // wait up to 1 second to send // TODO: log this instead
            System.reset(RESET_REASON_BALL_VALVE_UNEXPECTED_STATE, RESET_NO_WAIT);
        }

        if (mixingStation.isConnected()) { // note: don't rely on variable storing connection state; we want a realtime indicator of whether we're connected or not
            // ensure no pumps/solenoids/ets are energized -- that should only happen inside of other functions outside of loop()
            const int criticalOutputsToCheck[] = { // each int is the bit position of the output
                MixingStationIO::Components::DOSING_PUMP,
                MixingStationIO::Components::DRAIN_PUMP,
                MixingStationIO::Components::RODI_SOLENOID,
                MixingStationIO::Components::TAP_WATER_SOLENOID,
                MixingStationIO::Components::MAIN_PUMP
            };
            uint16_t gpioValues = mixingStation.readGPIOAB();
            uint16_t gpioValues_original = gpioValues;
            if (gpioValues == 0xffff) { // try again if it looks like we may have a bogus result
                delay(10);
                gpioValues = mixingStation.readGPIOAB();
            }
            for (int i=0; i<sizeof(criticalOutputsToCheck)/sizeof(criticalOutputsToCheck[0]); i++) { // note hack for getting number of elements in the array
                bool value = bitRead(gpioValues, criticalOutputsToCheck[i]);
                if (value == HIGH) {
                    mixingStation.disable();
                    //particle::Future<bool> publishFuture = Particle.publish("push-notification", String::format("{ \"type\": \"send-message-critical\", \"message\": \"[Aquarium controller] ALERT: Mixing station component energized while in loop(). Component: %d, gpioValues: 0x%x, gpioValues_original: 0x%x""\" }", criticalOutputsToCheck[i], gpioValues, gpioValues_original), PRIVATE);                
                    particle::Future<bool> publishFuture = PushNotification::send(String::format("[Aquarium controller] ALERT: Mixing station component energized while in loop(). Component: %d, gpioValues: 0x%x, gpioValues_original: 0x%x""\" }", criticalOutputsToCheck[i], gpioValues, gpioValues_original), true); //send(const char* message, bool criticalAlert=false) // testing
                    delay(1000); // wait up to 1 second to send // TODO: log this instead
                    System.reset(RESET_REASON_MIXING_STATION_DEVICE_ENERGIZED, RESET_NO_WAIT);
                }
            }
            
            // ensure that we don't detect water in our mixing tank liquid level sensor (e.g. in the case of an potential overflow situation)
            if (mixingStation.digitalReadOrReset(MixingStationIO::Components::CALIBRATION_WATER_SENSOR) == ACTIVE_LOW) {
                // wait a sec and try again to verify
                delay(1000);
                if (mixingStation.digitalReadOrReset(MixingStationIO::Components::CALIBRATION_WATER_SENSOR) == ACTIVE_LOW) {
                    static unsigned long timeSinceLastAlert = 0;
                    if (millis() - timeSinceLastAlert > 10*60*1000) { // note: this won't start alerting until 10 min (or whatever value is) after MCU (re)start
                        mixingStation.disable();
                        emergencyIODisable();
                        RGB.control(true); 
                        RGB.color(255,0,0);
                        lcd.display("Tmp water alert", "IO exp disabled", 0);
                        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: Water detected at calibration senor in mixing tank. DISABLING BOTH IO BOARDS. TODO: Improve monitering/alerting""\" }"), PRIVATE);
                        //errorBeep();
                        delay(1000);
                        timeSinceLastAlert = millis();
                    }
                }
            }
        }
        
        
        // TODO: add additional checks here, e.g. temperature
        

        lastMonitoringCheck = millis();
    }
    
    /****************************************************/
    


    
// periodically check for changes in mixing station connectivity
    static unsigned long lastMixingStationConnectionCheck = 0;
    if (millis() - lastMixingStationConnectionCheck > 1000) {
        // see if the mixing station is connected
        // note that if the unit is disabled, we re-enable it to check for connectivity (but disable again below if a connection isn't established)
        lastMixingStationConnectionCheck = millis();
        if (!mixingStation.isEnabled()) {
            mixingStation.enableAndReset(); // important: this should be disabled farther down if no connection was established
        }
        mixingStationConnected = mixingStation.isConnected(true);

        if (mixingStationConnected) {
            // notify if mixing station just connected or disconnected
            if (!mixingStationLastConnected) { // if we were disconnected last time, but are now connected...
                mixingStationLastConnected = true;
                RGB.control(false);
                Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] I/O expansion board reconnected""\" }"), PRIVATE);
                lcd.init(); // testing reconnecting lcd2, which gets corrupted if we don't initialize it after the IO expansion board is disconnected
            }
        } else { 
            mixingStation.disable(); // disable in case there's data corruption going on
            if (mixingStationLastConnected) { // if we were connected but something is now disconnected...
                mixingStationLastConnected = false;
                RGB.control(true); 
                RGB.color(255,0,0);
                Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: I/O expansion board disconnected""\" }"), PRIVATE);
            }
        }
    }
    
    // update mixing station status LED and check for button presses
    if (mixingStationConnected) {
        // blink mixing station LED
        if (millis() - mixingStationLastBlink >= 2000) {
            mixingStationLastBlink = millis();
            mixingStation.digitalWrite(MixingStationIO::Components::TEST_LED, HIGH);
            delay(150);
            mixingStation.digitalWrite(MixingStationIO::Components::TEST_LED, LOW);
        }
        // check for button press (to reset the utility room LCD display)
        // TODO: decide whether to keep this constant polling in place if data corruption is a liability
        if (mixingStation.digitalRead(MixingStationIO::Components::PUSHBUTTON_UPPER) == ACTIVE_LOW && !mixingStation.lastReadErrorFlag) {
            mixingStation.digitalWrite(MixingStationIO::Components::TEST_LED, HIGH);
            lcd.init();
            lcd.display("LCD displays", "reset", 1);
            mixingStation.digitalWrite(MixingStationIO::Components::TEST_LED, LOW);
            delay(100);
        }        
    }
    
    
    // new: debugging why this seems to trigger right after MCU start for some reason
    if (autoTopoffEnabled && ((millis() - timeSinceWaterLevelCheck > WATER_LEVEL_CHECK_TIME_MS) || isSimulationMode) && (millis() > 2*60*1000)) {
        doWaterLevelCheck();
        timeSinceWaterLevelCheck = millis();
    }        
        
        
    
    
    
    // Left off: test new class thoroughlly, and add safety check block here to ensure mixing station outputs are never active (they should only be set high in certain code blocks)
    
    
    
    
    
    
    
    // test
    /*
    co2BubbleSensor.stopInterrupts();
    static unsigned long tmpToggleTime = 0;
    static bool tmpToggle = true;
    if (millis() - tmpToggleTime > 5000) {
        tmpToggleTime = millis();
        tmpToggle = !tmpToggle;
        
        if (tmpToggle) {
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string("IO initialized: " + String(mixingStation.initialize();));
            delay(1);
            lcd.setCursor(0,1);
            lcd.send_string("IO LED on");
            mixingStation.digitalWriteSafe(MixingStationIO::Components::TEST_LED, tmpToggle, false);
        } else {
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string("IO brd disabled");
            digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, HIGH); // HIGH disables the IO expansion board
            delay(3000);
            lcd.setCursor(0,1);
            lcd.send_string("IO board enabled");
            digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, LOW); // HIGH disables the IO expansion board
            delay(1);
        }
        
        //digitalWrite(LED, tmpToggle);
        //digitalWrite(PIN__IO_EXP_2_DISABLE, !tmpToggle); // LOW disables the IO expansion board
    }
return;    
    */
    
    
    
    
    if (!uptimeExceededOneMinute && (millis() - firstConnectedMillis) > 1000*60) {
        uptimeExceededOneMinute = true;
    }

    /*
    // persist (and temporarily double-alert on) any CO2 faults
    CO2BubbleSensor_v3::Faults currentFaultCode = co2BubbleSensor.getFaultCode();
    if (currentFaultCode != lastCO2ControllerFault) {
        lastCO2ControllerFault = currentFaultCode;
        if (currentFaultCode != CO2BubbleSensor_v3::Faults::NONE) {
            errorBeep(); // TODO: move this to utility class and encapsulate in updateFaultCodeAndSendAlerts()
        }
        Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] debug (delete me): CO2 fault status changed to " + String(currentFaultCode) + ".""\" }", PRIVATE);
    }
    */
    
    // check for manual CO2 toggle
    if (flag__cloud_toggleCO2) {
        flag__cloud_toggleCO2 = false;
        if (co2BubbleSensor.isCO2Started()) {
            Particle.publish("debug", "Toggling CO2 off");
            co2BubbleSensor.stopCO2();
        } else {
            co2BubbleSensor.startCO2();
            Particle.publish("debug", "Toggling CO2 on");
        }
    }
    
    if (flag__cloud_tempTest) {
        flag__cloud_tempTest = false;
        automaticWaterChange.resetLastRunEEPROM();
        
        
        /*
        float rodiWaterGallons = (TANK_WATER_CHANGE_GALLONS * RODI_TO_TAP_TARGET_RATIO) / (RODI_TO_TAP_TARGET_RATIO + 1);
        float tapWaterGallons = TANK_WATER_CHANGE_GALLONS - rodiWaterGallons;
            
        bool cancelledViaButtonPress = false;
        unsigned long debug_numInvalidReadings = 0;
        float rodiTargetGallons = rodiWaterGallons * (1-0.9638);
        mixingStation.dispenseWater(0.0, rodiTargetGallons, cancelledViaButtonPress, debug_numInvalidReadings); //float tapWaterGallons, float rodiWaterGallons, bool &cancelledViaButtonPress, unsigned long &debug_numInvalidReadings) {
        */

///        mixingStation.dispenseWaterCalibrationMode(true);
        //mixingStation.dispenseWaterCalibrationMode(false);
        
        //mixingStation.dispenseWaterAndCountPulses();
        //mixingStation.dispenseWaterAndTestFlowMeter();
        //mixingStation.temp_measureWaterAtBothSensors();
        
        /*
        displayStrings(
            3,
            "test 1", 
            "this is a very long string number two", 
            "test number three"
        );
        */
        
        /*
        lcdDisplayWaitPressButton("Press to", "test");
        lcd.display("Opening", "valve...", 0);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, LOW); }
        delay(5000);
        lcd.display("Closing", "valve...", 0);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, HIGH); }
        delay(5000);
        lcd.clear();
        return;
        */
        
        /*
        lcdDisplayWaitPressButton("Press to", "disable");
        disableFilterAndHeatWithAlert();
        
        lcdDisplayWaitPressButton("Press to", "enable");
        enableFilterAndHeatWithAlert();

        lcdDisplayWaitPressButton("Debug test", "next?");
        co2BubbleSensor.pushDebugInfo();
        
        lcd.clear();
        */

        /////co2BubbleSensor.pushDebugInfo();
        //CO2BubbleSensor_v3 bubbleSensor = CO2BubbleSensor_v3(PIN__CO2_SENSOR_IN, PIN_IOEXP2__C02_SOLENOID, &mcp2);
        //co2BubbleSensor.integrationTest();
        //Particle.publish("getBubblesPerSecondCombined()", String(co2BubbleSensor.getBubblesPerSecondCombined(false)));
        //Particle.publish("getBubblesPerSecond(120)", String(co2BubbleSensor.getBubblesPerSecond(120, false)));
        //Particle.publish("getBubblesPerSecond(5)", String(co2BubbleSensor.getBubblesPerSecond(5, true)));
    }
    
    // TODO: convert to part of controller state machine
    if (flag__cloud_sensorTestToggle) {
        flag__cloud_sensorTestToggle = false;
        while (1) { // for now, reset to exit this mode
            static unsigned long timeSinceSensorsLastUpdated = 0;
             if (millis() - timeSinceSensorsLastUpdated > 333) {
                timeSinceSensorsLastUpdated = millis();
                
                // "For the MCP23017 you specify a pin number from # 0 to 15 for the GPIOA0...GPIOA7, GPIOB0...GPIOB7 pins (i.e. pin 12 is GPIOB4)."
                // I.e. Pin number 0..7 corresponds to A0..A7, and 8..15 corresponds to B0..B7
                uint16_t values = mixingStation.readGPIOAB();
                lcd.setCursor(0,0);
                
                if (bitRead(values, MixingStationIO::Components::PUSHBUTTON_UPPER) == ACTIVE_LOW && bitRead(values, MixingStationIO::Components::PUSHBUTTON_LOWER) == ACTIVE_LOW) {
                    lcd.send_string("Both pushbuttons");
                } else if (bitRead(values, MixingStationIO::Components::PUSHBUTTON_UPPER) == ACTIVE_LOW) {
                    lcd.send_string("Pushbutton upper");
                } else if (bitRead(values, MixingStationIO::Components::PUSHBUTTON_LOWER) == ACTIVE_LOW) {
                    lcd.send_string("Pushbutton lower");
                } else {
                    lcd.send_string(String(values, BIN) + "               "); // testing
                }
                
                /*
                #define PIN__TOPOFF_FLOW_METER  A0
                #define PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE  A2
                #define PIN__LOWER_LIQUID_LEVEL_SENSOR_CAPACITIVE  A3 // new connector pinout (looking top down onto PCB with buzzer to the left): signal, +5V, GND (top to bottom)
                #define PIN__WATER_LEAK_SENSOR  A1 // active high (note to test: make sure you bridge contacts e.g. with hands; don't just touch one to make the test LED light up)
                #define PIN__PUSHBUTTON         D12 // active low. new connector pinout (looking top down onto PCB with buzzer to the left): GND, signal (top to bottom)
                #define PIN_IOEXP2__TOPOFF_SOLENOID         9 //B1
                #define PIN_IOEXP2__CUTOFF_BALL_VALVE        8 //B0
                #define PIN_IOEXP2__CANNISTER_PUMP_RELAY    15 //B7
                #define PIN_IOEXP2__C02_SOLENOID            10 //B2
                #define PIN_IOEXP2__PUSHBUTTON_2            2 //A2
                */
                uint8_t gpioB;
                WITH_LOCK(Wire) { gpioB = mcp2.readGPIO(1); }
                bool pushbutton2Value;
                WITH_LOCK(Wire) { pushbutton2Value = mcp2.digitalRead(PIN_IOEXP2__PUSHBUTTON_2); }
                lcd.setCursor(0,1);
                lcd.send_string(String::format("%d%d%d%d%d %s%d", 
                    0, //digitalRead(PIN__TOPOFF_FLOW_METER),
                    digitalRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE),
                    digitalRead(PIN__LOWER_LIQUID_LEVEL_SENSOR_CAPACITIVE),
                    digitalRead(PIN__WATER_LEAK_SENSOR),
                    digitalRead(PIN__PUSHBUTTON),
                    String(gpioB, BIN).c_str(), 
                    pushbutton2Value
                ));
            }
            Particle.process(); // probably redundant since we're returning from loop() quickly, but adding here for safety
        }
    }
    


    if (flag__cloud_drainTank_pwd) {
        flag__cloud_drainTank_pwd = false;
        mixingStation_drainAquarium(123, false); // important: set leaveFilterAndHeaterOff to false
    }
    
    if (flag__cloud_dosingPump2Ctrl) {
        flag__cloud_dosingPump2Ctrl = false;
        
        
        // temp -- TODO delete
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.send_string("Btn 1: Dosing #2");
        lcd.setCursor(0,1);
        lcd.send_string("Btn 2: Exit");
        do {
            if (digitalRead(PIN__PUSHBUTTON) == ACTIVE_LOW) {
                digitalWrite(PIN__DOSING_PUMP_2, HIGH);
            } else {
                digitalWrite(PIN__DOSING_PUMP_2, LOW);
            }
                
            bool mainButton2Value;
            WITH_LOCK(Wire) { mainButton2Value = mcp2.digitalRead(PIN_IOEXP2__PUSHBUTTON_2); }
            if (mainButton2Value == ACTIVE_LOW) {
                delay(100); // debounce hack
                WITH_LOCK(Wire) { mainButton2Value = mcp2.digitalRead(PIN_IOEXP2__PUSHBUTTON_2); }
                if (mainButton2Value == ACTIVE_LOW) {
                    lcd.clear();
                    break;
                }
            }

            Particle.process();
        } while (1);

        digitalWrite(PIN__DOSING_PUMP_2, LOW);
        
        
        // TODO: comment this out :-)
        /*
        digitalWrite(PIN__DOSING_PUMP_1, HIGH);
        delay(250);
        digitalWrite(PIN__DOSING_PUMP_1, LOW);
        delay(1000);

        digitalWrite(PIN__DOSING_PUMP_2, HIGH);
        delay(250);
        digitalWrite(PIN__DOSING_PUMP_2, LOW);
        delay(1000);

        digitalWrite(PIN__DOSING_PUMP_3, HIGH);
        delay(250);
        digitalWrite(PIN__DOSING_PUMP_3, LOW);
        delay(1000);
        
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, HIGH); }
        delay(250);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, LOW); }
        delay(1000);
        
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__C02_SOLENOID, HIGH); }
        delay(250);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__C02_SOLENOID, LOW); }
        delay(1000);
        
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CANNISTER_PUMP_RELAY, HIGH); }
        delay(250);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CANNISTER_PUMP_RELAY, LOW); }
        delay(1000);
        
        digitalWrite(PIN__DRAIN_BALL_VALVE, HIGH);
        delay(2*1000);
        digitalWrite(PIN__DRAIN_BALL_VALVE, LOW);
        delay(1000);
        
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, HIGH); }
        delay(2*1000);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, LOW); }
        delay(1000);
        */

        
        
        
        
        
        /*
        flag__cloud_dosingPump2Ctrl = false;
        lcd.display("Output test", "", 1);

        lcd.setCursor(0,1);
        lcd.write(0b00011000); // 0b00011001; 00011110, 00011111 -- see http://www.martyncurrey.com/arduino-with-hd44780-based-lcds/
        lcd.setCursor(1,1);
        lcd.send_string("Test");
        lcd.setCursor(8,1);
        lcd.write(0b00011001); // ; 00011110, 00011111 -- see http://www.martyncurrey.com/arduino-with-hd44780-based-lcds/
        lcd.setCursor(9,1);
        lcd.send_string("Skip");
        
        lcd.setCursor(0,0);
        lcd.send_string("Test name 1");

        // TODO: add lower level buttons too
        bool topButtonPressed = false;
        bool bottomButtonPressed = false;
        do {
            topButtonPressed = (digitalRead(PIN__PUSHBUTTON) == ACTIVE_LOW);
            
            WITH_LOCK(Wire) {
                bottomButtonPressed = (mcp2.digitalRead(PIN_IOEXP2__PUSHBUTTON_2) == ACTIVE_LOW);
            }

            if (topButtonPressed || bottomButtonPressed) {
                break;
            }
        } while(1);
        */
        
        
    }
    
    // testing...
    // inputs
    // PIN__LOWER_LIQUID_LEVEL_SENSOR_CAPACITIVE // (TODO: INPUT_PULLUP?? check docs)
    // PIN__WATER_LEAK_SENSOR (INPUT_PULLUP?)
    // PIN__TOPOFF_FLOW_METER
    // PIN__PUSHBUTTON_2 (TODO: retool this)
    // outputs
    // PIN__DRAIN_BALL_VALVE
    // PIN__MIXING_STATION_IO_EXP_DISABLE (TODO: pinSetDriveStrength(PIN__PIEZO_BUZZER, DriveStrength::HIGH); // TODO: consider checking for returned success/error code)
    // PIN__IO_EXP_2_DISABLE
    // SEE: "mcp2.begin(I2C_ADDRESS_ADDITION); ..."
    //mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, LOW);
    //mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, LOW);
    //mcp2.digitalWrite(PIN_IOEXP2__CANNISTER_PUMP_RELAY, LOW);
    //mcp2.digitalWrite(PIN_IOEXP2__C02_SOLENOID, LOW);
/*    
    static bool outputTestToggle = false;
    static unsigned long timeSinceOutputToggle = 0;
    if (millis() - timeSinceOutputToggle > 2000) {
        timeSinceOutputToggle = millis();
        outputTestToggle = !outputTestToggle;
        
    // PIN__DRAIN_BALL_VALVE
    // PIN__MIXING_STATION_IO_EXP_DISABLE (TODO: pinSetDriveStrength(PIN__PIEZO_BUZZER, DriveStrength::HIGH); // TODO: consider checking for returned success/error code)
    // PIN__IO_EXP_2_DISABLE
        digitalWrite(PIN__DRAIN_BALL_VALVE, outputTestToggle);
        digitalWrite(PIN__MIXING_STATION_IO_EXP_DISABLE, outputTestToggle);
///        digitalWrite(PIN__IO_EXP_2_DISABLE, );
        WITH_LOCK(Wire) { 
            mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, outputTestToggle); 
            mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, outputTestToggle); 
            mcp2.digitalWrite(PIN_IOEXP2__CANNISTER_PUMP_RELAY, outputTestToggle); 
            mcp2.digitalWrite(PIN_IOEXP2__C02_SOLENOID, outputTestToggle); 
        }
    }
    
    lcd.setCursor(0,0);
    lcd.send_string(String::format("Test mode     %d", (int)millis()/1000));
    lcd.setCursor(0,1);
    bool a, b, c;
    lcd.send_string(String::format("%d %d %d     %s", 
        digitalRead(PIN__LOWER_LIQUID_LEVEL_SENSOR_CAPACITIVE),
        digitalRead(PIN__WATER_LEAK_SENSOR),
        digitalRead(PIN__TOPOFF_FLOW_METER),
        outputTestToggle ? "on " : "off"
    ));
////    delay(200);
////    Particle.process();
////    return;
*/
    
    if (flag__cloud_fillMixingTank_pwd) {
        flag__cloud_fillMixingTank_pwd = false;
        bool success = mixingStation.drainReservoir();
        if (success) {
            bool cancelledViaButtonPress = false;
            unsigned long debug_numInvalidReadings = 0;
            success = mixingStation.dispenseWater(TANK_WATER_CHANGE_GALLONS, cancelledViaButtonPress, debug_numInvalidReadings);
            Particle.publish("mixingStation.dispenseWater()", String::format("success: %d, cancelledViaButtonPress: %d, debug_numInvalidReadings: %u", success, cancelledViaButtonPress, debug_numInvalidReadings));
        }
    }
    
    if (flag__cloud_drainReserv) {
        flag__cloud_drainReserv = false;
        float sensorDetectionPercent;
        bool success = mixingStation.drainReservoir(sensorDetectionPercent);
        Particle.publish("mixingStation.drainReservoir()", String::format("success: %d, sensorDetectionPercent: %2f", success, sensorDetectionPercent));
    }

    if (flag__cloud_dosePrime) {
        flag__cloud_dosePrime = false;
        bool success = mixingStation.dosePrime();//flag__cloud_dosePrime_optOverrideTimeSeconds);
        flag__cloud_dosePrime_optOverrideTimeSeconds = 0;
        Particle.publish("mixingStation.dosePrime()", String::format("success: %d", success));
    }
    
    if (flag__cloud_fullWaterChange_pwd) {
        bool success;
        if (flag__cloud_fullWaterChange_pwd == 1) { // normal
            //success = runFullWaterChangeAutomatic(true, true, false, false); // runs the real thing! // bool runFullWaterChangeAutomatic(bool requireButtonPresses=true, bool minimumPressesOnly=false, bool limitedTestMode=false) {
            success = runFullWaterChangeAutomatic(WaterChangeMode::FULL_WATER_CHANGE, WaterChangeAutomationLevel::SEMI_AUTOMATED);
        } else if (flag__cloud_fullWaterChange_pwd == 2) { // skip filling steps
            //success = runFullWaterChangeAutomatic(true, true, false, true); // runs the real thing! // bool runFullWaterChangeAutomatic(bool requireButtonPresses=true, bool minimumPressesOnly=false, bool limitedTestMode=false) {
            success = runFullWaterChangeAutomatic(WaterChangeMode::SKIP_MIXING_TANK_FILL, WaterChangeAutomationLevel::SEMI_AUTOMATED);
        } else {
            Particle.publish("runFullWaterChangeAutomatic(true)", "Error: Unknown mode");
        }
        Particle.publish("runFullWaterChangeAutomatic(true)", String::format("success: %d", success));
        flag__cloud_fullWaterChange_pwd = 0;
    }
    
    if (flag__cloud_mainPumpTest_pwd) {
        flag__cloud_mainPumpTest_pwd = false;
        bool success = mixingStation.pumpWaterToAquarium(123, isLiquidDetectedTop);
        Particle.publish("mixingStation.pumpWaterToAquarium()", String::format("success: %d", success));
    }
    
    
    
    
    
    
    
    
    
    if (tmp__ioTestEnable) {
    
// DANGEROUS -- REMOVE THIS RIGHT AFTER TESTING :-0
/*
tmp__ioTestEnable = false;
mixingStation.digitalWriteSafe(MixingStationIO::Components::MAIN_PUMP, HIGH);
delay(1000);
emergencyIODisable(); note: does NOT Seem to work  <-- left off here
//delay(1000);
//mixingStation.digitalWriteSafe(MixingStationIO::Components::MAIN_PUMP, LOW);
return;        
*/
        
        
        
        
        
        // I/O test: burst of I2C reads to test for glitches
        static unsigned long tmp_lastIOL2CTest = 0;
        static unsigned long tmp_lastFail = 0;
        static bool tmp_runOnce = false;
    // **TODO** this is hanging system    
        if (!firmwareUpdateBegun && millis() - tmp_lastIOL2CTest > 5*1000) {
            if (!tmp_runOnce) {
                tmp_runOnce = true;
                Particle.publish("IO temp test running for 1st time");
            }
            unsigned long tmp_lastFailElapsedTimsMs = 0;
            bool failFlag1 = false;
            bool failFlag2 = false;
    ////////Particle.publish("I2C test running");
            wd->checkin(); // start clean here
            hwWatchdog.pet(); // start clean here
            unsigned long start = millis();
    
    //**check this**
    ////mixingStation.isConnected(false);  // seeing if this changes fail # of 1 each time...       
    
            ///uint16_t originalGPIOValues = mixingStation.readGPIOAB();
            do {
                tmp_numChecks++;

/*
                // new test
                uint16_t newGPIOValues = mixingStation.readGPIOAB();
                if (newGPIOValues != originalGPIOValues) {
                    Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"IO Expander temp test failed. Difference detected. Original: " + String(originalGPIOValues, BIN) + ", new: " + String(newGPIOValues, BIN) + """\" }"), PRIVATE);
                    delay(1000);
                    break;
                }
*/

// new test
/*
if (isLiquidDetectedTop()) {
    Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"Temp test: isLiquidDetectedTop() returned true""\" }"), PRIVATE);
    delay(1000);
    break;    
}
*/

                // original test
                if (!mixingStation.isConnected(false)) { // disable auto-repair if disconnected
                    tmp_numChecksFailed++;
                    tmp_lastFailElapsedTimsMs = millis() - tmp_lastFail;
                    tmp_lastFail = millis();
                    failFlag1 = mixingStation.__tmp_test1Failed;
                    failFlag2 = mixingStation.__tmp_test2Failed;
                }

    //  TODO: consider Particle.process(), or even running a motor here...
            } while (millis() - start < 8*1000);
            
            if (tmp_numChecksFailed > 0) {
                /*Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"IO Expander temp test failed. tmp_lastFailElapsedTims sec: %.2f, numChecks: %d, numChecksFailed: %d, fail flag 1: %d, fail flag 2: %d""\" }", 
                    tmp_lastFailElapsedTimsMs/1000.0, tmp_numChecks, tmp_numChecksFailed, 
                    failFlag1, failFlag2), PRIVATE);
                delay(1000);*/
            }
    
            tmp_lastIOL2CTest = millis();
    
            //Particle.publish("I2C test tmp_numChecks: " + String(tmp_numChecks));
        }
    }
    
    
    
    /*
    // toggle between select modes when pushbutton pushed
    if (digitalRead(PIN__PUSHBUTTON) == ACTIVE_LOW && (controllerMode == ControllerModes::NORMAL || controllerMode == ControllerModes::TANK_CLEANING_MODE)) {
        // beep and wait for button release
        beepBuzzer(1);
        while (!digitalRead(PIN__PUSHBUTTON));
        delay(25);
        static bool wasCO2Active = false;
        
        // entering into tank cleaning mode
        if (controllerMode == ControllerModes::NORMAL) {
            controllerMode = ControllerModes::TANK_CLEANING_MODE;
            lcd.setRGB(LCD_RGB_CLEANING_MODE);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string("Paused; heat off");
            relayModule.setHeater(false);
            wasCO2Active = co2BubbleSensor.isCO2Started();
            if (wasCO2Active) {
                co2BubbleSensor.stopCO2();
            }
        } else 
        
        // entering back into "normal" mode
        if (controllerMode == ControllerModes::TANK_CLEANING_MODE) {
            controllerMode = ControllerModes::NORMAL;
            lcd.setRGB(LCD_RGB_NORMAL);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string("Resuming!");
            relayModule.setHeater(true);
            if (wasCO2Active && CO2BubbleSensor_v3::isActiveTimeOfDay()) {
                //co2BubbleSensor.resetMonitoringStats();
                co2BubbleSensor.startCO2();
            }
        }
        
        delay(1000);
        timeSinceLcdUpdate = millis() - LCD_UPDATE_TIME_MS - 1; // force LCD refresh next cycle
    }
    */

    
    if (controllerMode == ControllerModes::TANK_CLEANING_MODE) {
        
    } else if (controllerMode == ControllerModes::NORMAL) {
    
        // sample topoff sensor
        sampleTopoffSensorTimed(TopoffSensorModes::NORMAL);
        /*
        if (millis() - topoffSensorLastSampled_normal >= TOPOFF_SENSOR_INTERVAL_NORMAL) {
            topoffSensorLastSampled_normal = millis();
            topoffSensorSamples_normal.addValue(analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE)); // normal value seems to be ~3100 when no contact with wqter    
            //topoffSensorSamples.addValue(analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE) < 2900); // normal value seems to be ~3100 when no contact with wqter    
        }
        */
        
        
        
        
        
        
        
    
        

        
    //    left off: moving retained variable to startDosingPumpSafe(int pumpNum, long *lastRunEpoch_retained)
        
        
        // TODO: safety check that CO2 is disabled after certain hour (e.g. if we exceed our Timer limit and lose expected functionality)
    
        /*
        // see if we should run dosing pump 1
        if (Time.isValid() && Time.hour() == dosingPump1.startHour && Time.minute() == dosingPump1.startMinute) {
            long now = Time.now();
            
            // start by checking our retained variable before hitting EEPROM
            float hoursSinceLastRun_Retained = (dosingPump1LastRunEpoch_Retained == -1) ? -1 : (now - dosingPump1LastRunEpoch_Retained)/60.0;
            
            if (hoursSinceLastRun_Retained == -1 || hoursSinceLastRun_Retained >= 24*dosingPump1.repeatIntervalDays) {
                // then check our EEPROM timestamp, for redundancy
                long dosingPump1LastRunEpoch_EEPROM;
                EEPROM.get(dosingPump1.lastRunEEPROMAddress, dosingPump1LastRunEpoch_EEPROM); // will return 0xFFFFFFFF if no existing value has been stored
                float hoursSinceLastRun_EEPROM = (dosingPump1LastRunEpoch_EEPROM == 0xFFFFFFFF) ? -1 : (now - dosingPump1LastRunEpoch_EEPROM)/60.0;
            
                if (hoursSinceLastRun_EEPROM == -1 || hoursSinceLastRun_EEPROM >= 24*dosingPump1.repeatIntervalDays) {
                    dosingPump1LastRunEpoch_Retained = now;
                    EEPROM.put(dosingPump1.lastRunEEPROMAddress, now);
                    Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Running dosing pump 1. hoursSinceLastRun_Retained: %ld, hoursSinceLastRun_EEPROM: %ld""\" }", hoursSinceLastRun_Retained, hoursSinceLastRun_EEPROM), PRIVATE);                
                    
                    // TODO: run dosing pump
                } else {
                    // this should never happen!
                    Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *ERROR* Dosing pump 1 scheduling control mismatch. hoursSinceLastRun_Retained: %ld, hoursSinceLastRun_EEPROM: %ld""\" }", hoursSinceLastRun_Retained, hoursSinceLastRun_EEPROM), PRIVATE);
                    delay(60*1000); // don't spam push notifications
                }
            }
        }
        */
        
        /*
        // testing: see if we should turn CO2 injection on or off
        // note that we only want to try turning CO2 on once a day, vs anytime during the day within the scheduled window (e.g. if there's a fault state followed by a reset)
        if (Time.isValid() && Time.hour() >= 8 && Time.hour() <= 16 && !co2BubbleSensor.isCO2Started()) {
            if (lastCO2ControllerFault == CO2BubbleSensor_v3::Faults::NONE) {
                co2BubbleSensor.startCO2();
                Particle.publish("debug", "Starting/resuming CO2...")
                //int secondsSinceLastActivationFlag = Time.now() - scheduler_co2LastActivation
                
                
                
                //scheduler_co2LastActivation = ...
            }
        }
        */
    
        
        
        
        
        
        
        
        //hardwareWatchdog.pet();
        
        // safety in case timed task fails
        if (!CO2BubbleSensor_v3::isActiveTimeOfDay() && Time.minute() > 5 && co2BubbleSensor.isCO2Started()) { // new: added 5 minute grace period
            if (mixingStation.isEnabled()) { //new. eg not via mixingStation.disable(), emergencyIODisable()
                co2BubbleSensor.stopCO2();
                errorBeep();
                static PushNotification notification_co2BackupAlert(10min); 
                notification_co2BackupAlert.sendWithCooldown("WARNING: C02 stopped via backup. You should never see this!", true);
            }
        }
        /*
        if (Time.isValid()) {
            if (Time.hour() >= 8 && Time.hour() <= 16 && && !co2BubbleSensor.isCO2Started()) {
                // TODO: only set this once per day, e.g. avoid turning back on right after a fault
                co2BubbleSensor.startCO2();
            } else if (!(Time.hour() >= 8 && Time.hour() <= 16) && co2BubbleSensor.isCO2Started()) {
                co2BubbleSensor.stopCO2();
                Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Stopped C02.""\" }", PRIVATE);
            }
        }
        */
    
        
        // testing
        if (flag_injectForSec > 0 && flag_injectForSecPumpNum != -1) {
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.send_string(String::format("Dosing pump %d on", flag_injectForSecPumpNum));
            
            lcd.setCursor(0,1);
            //lcd.send_string("Memory: " + String(System.freeMemory()));
            
            float secondsPerMl = getSecondsPerMl(flag_injectForSecPumpNum);
            if (secondsPerMl > 0) {
                float doseMilliliters = flag_injectForSec/secondsPerMl;
                //bool startDosingPumpSafe(int dosingPumpNum, float doseMilliliters, long &lastRunEpoch_retained) {
                long tmp__lastRunEpoch_retained = -1; // dummy -- BE CAREFUL USING THIS AS YOU COULD EASILY OVERDOSE!   
                bool success = startDosingPumpSafe(flag_injectForSecPumpNum, doseMilliliters, tmp__lastRunEpoch_retained); //dosingPump{number}LastRunEpoch_Retained);
            } else {
                Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *Not* running dosing pump %d: Unknown pump number (loop)""\" }", flag_injectForSecPumpNum), PRIVATE);                
            }
            // alternate version below for testing -- but it's dangerous and could result in an instant overdose
            //long tmp__lastRunEpoch_retained = -1; // dummy
            //bool success = startDosingPumpSafe(1, doseMilliliters, tmp__lastRunEpoch_retained);
                
            /*
            if (hwWatchdog.setOneOffWatchdogAlarmThenDisable__see_notes(flag_injectForSec * 1000 + 100)) { // include a bit of buffer time
                digitalWrite(PIN__DOSING_PUMP_1, HIGH);    
                delay(flag_injectForSec * 1000);
                digitalWrite(PIN__DOSING_PUMP_1, LOW);
                hwWatchdog.enable();
            }
            */
    
            lcd.clear();
            flag_injectForSec = 0;
        }

        
    
    
        
        // testing
        //Particle.process();
        
        if (flag_doWaterLevelCheck) {
            flag_doWaterLevelCheck = false;
            doWaterLevelCheck(true);
        }
        
        //digitalWrite(LED, digitalRead(PIN__FLOW_SENSOR));
        /*
        if (cloud_samplePump_flag) {
            cloud_samplePump_flag = false;
            samplePumpReadings();
        }
        */
        
        
        if (millis() - timeSinceLogToSpreadsheet > LOG_TO_SPREADSHEET_TIME_MS) {
            timeSinceLogToSpreadsheet = millis();
            Particle.publish("google-sheets", String::format(
                "{"
                    "\"sheet\": \"Aquarium-sensor-data\", "
                    "\"Temperature (tank)\": %.5f, "
                    "\"Temperature (heater)\": %.5f, "
                    "\"Flow rate\": %.5f, "
                    "\"Free memory\": %lu, " // new
                "}",
                tempInF, tempOutF, flowRate, System.freeMemory()
            ));
        } 
    }
}


void doTempTempChecks(double &tempInF, double &tempOutF, double &flowRate) {
    if (tempInF > 83 || tempOutF > 90) {
        if (!flag__tempTooHigh) {
            flag__tempTooHigh = true;
            relayModule.setHeater(false);
            errorBeep();
            errorBeep();
            lcd.setRGB(LCD_RGB_WARNING);
            lcd.display("WARNING: Temp", "too high", 1);
            PushNotification::send(String::format("WARNING: Temperature too high (inlet: %.2fF, outlet: %.2fF). Disabling heater.", tempInF, tempOutF));
        }
    } else if (flag__tempTooHigh) {
        flag__tempTooHigh = false;
        relayModule.setHeater(true);
        PushNotification::send(String::format("Update: Temperature no longer too high (inlet: %.2fF, outlet: %.2fF). Re-enabling heater.", tempInF, tempOutF));
        lcd.setRGB(LCD_RGB_NORMAL);
    }

    if ((tempInF < 74 || tempOutF < 74) && (mixingStation.waterLastPumpedToAquarium == 0 || millis() - mixingStation.waterLastPumpedToAquarium > 5*60*1000)) { // new: allow time for heating if water was just changed
        if (!flag__tempTooLow) {
            flag__tempTooLow = true;
            errorBeep();
            errorBeep();
            lcd.setRGB(LCD_RGB_WARNING);
            lcd.display("WARNING: Temp", "too low", 1);
            PushNotification::send(String::format("WARNING: Temperature too low (inlet: %.2fF, outlet: %.2fF).", tempInF, tempOutF));
            /*
            for (int i=0; i<3; i++) {
                // jiggle the relay in case it's stuck
                relayModule.setHeater(false);
                delay(200);
                relayModule.setHeater(true);
                delay(200);
            }
            */
        }
    } else if (flag__tempTooLow) {
        flag__tempTooLow = false;
        relayModule.setHeater(true);
        PushNotification::send(String::format("Update: Temperature no longer too low (inlet: %.2fF, outlet: %.2fF). Re-enabling heater.", tempInF, tempOutF));
        lcd.setRGB(LCD_RGB_NORMAL);
    }
}
/*bool waterDetectedCapacitiveSensorFluctuating() {
    int reading = analogRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE);
    return (reading > 1000 && reading < 2900);
}*/

int cloud_bypassFlowSensorTgl(String extra) {
    setting__disableFlowRateSensor = !setting__disableFlowRateSensor;
    return setting__disableFlowRateSensor;
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

int cloud_toggleCO2(String extra) {
    flag__cloud_toggleCO2 = true;
    return 1;
}

/*
int cloud_addTapWater(String extra) {
    mixingStation.test_addSomeTapWater();
    return 1;
}


int cloud_fillTest(String extra) {
    flag_cloud_ioTest = true;
    return 1;
}
*/

int cloud_coolingTest(String extra) {
    flag__coolingPumpTest = true;
    return 1;
}

int cloud_tempTest(String extra) {
    
    flag__cloud_tempTest = true;
    return 1;
    
    // testing the two 120VAC relays
    /*
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.send_string("Heater relay off");
    //pinMode(PIN__HEATER_RELAY, OUTPUT);
    //digitalWrite(PIN__HEATER_RELAY, LOW); // active low
    
    
    pinMode(PIN__HEATER_RELAY, OUTPUT_OPEN_DRAIN);//OUTPUT_OPEN_DRAIN); // "HIGH (1) leaves the output in high impedance state, LOW (0) pulls the output low. Typically used with an external pull-up resistor to allow any of multiple devices to set the value low safely.
    pinSetDriveStrength(PIN__HEATER_RELAY, DriveStrength::HIGH);
    digitalWrite(PIN__HEATER_RELAY, LOW);
    
    
    
    
    delay(1000);
    
    lcd.setCursor(0,1);
    lcd.send_string("Heater relay on");
    
    
    
    //pinMode(PIN__HEATER_RELAY, OUTPUT_OPEN_DRAIN); // "HIGH (1) leaves the output in high impedance state, LOW (0) pulls the output low. Typically used with an external pull-up resistor to allow any of multiple devices to set the value low safely.
    //pinMode(PIN__HEATER_RELAY, OUTPUT);
    //pinSetDriveStrength(PIN__HEATER_RELAY, DriveStrength::HIGH);
    digitalWrite(PIN__HEATER_RELAY, HIGH); //HEATER_ENABLE);
    
    
    
    //digitalWrite(PIN__HEATER_RELAY, HIGH);
    //pinMode(PIN__HEATER_RELAY, OUTPUT_OPEN_DRAIN); // leave floating so internal pull-up on relay board takes this to a 5V logic HIGH; otherwise our 3.3V "high" seems to read as low.
    delay(1000);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.send_string("C pump relay off");
    WITH_LOCK(Wire) { 
        mcp2.pinMode(PIN_IOEXP2__CANNISTER_PUMP_RELAY, OUTPUT);
        mcp2.digitalWrite(PIN_IOEXP2__CANNISTER_PUMP_RELAY, LOW);
    } 
    delay(1000);
    
    lcd.setCursor(0,1);
    lcd.send_string("C pump relay on");
    //mcp2.digitalWrite(PIN_IOEXP2__CANNISTER_PUMP_RELAY, HIGH); 
    WITH_LOCK(Wire) { 
        mcp2.pinMode(PIN_IOEXP2__CANNISTER_PUMP_RELAY, INPUT);
    }
    delay(1000);
    
    return 1;
    */
}

int cloud_ioTestToggle(String extra) {
// temp test
//mixingStation.digitalWriteOrReset(MixingStationIO::Components::DRAIN_PUMP, HIGH); // left off: flicks on then turns off
///////    tmp__ioTestEnable = !tmp__ioTestEnable;
    return tmp__ioTestEnable;
}

int cloud_ioTest2(String extra) {
    uint16_t values = mixingStation.readGPIOAB();
    Particle.publish("readGPIOAB()", String(values, BIN));
    return values >> 8; // return "B" side of IO board
}

int cloud_fillMixingTank_pwd(String extra) {
    int passcode = extra.toInt();
    if (passcode == 123) {
        flag__cloud_fillMixingTank_pwd = true;
        return 1;
    } else {
        return -1;
    }
}

int cloud_sensorTestToggle(String extra) {
    flag__cloud_sensorTestToggle = !flag__cloud_sensorTestToggle;
    if (flag__cloud_sensorTestToggle) {
        lcd.clear();
        //lcd.setCursor(0,1);
        //lcd.send_string("(Ctrl disabled)");
    } else {
        lcd.clear();
    }
    return flag__cloud_sensorTestToggle;
}

int cloud_dosingPump2Ctrl(String extra) {
    flag__cloud_dosingPump2Ctrl = !flag__cloud_dosingPump2Ctrl;
    return flag__cloud_dosingPump2Ctrl;
}

int cloud_drainReserv(String extra) {
    flag__cloud_drainReserv = true;
    return 1;
}

int cloud_dosePrime(String extra) {
    //flag__cloud_dosePrime_optOverrideTimeSeconds = extra.toFloat(); // should return 0 if empty or invalid string
    flag__cloud_dosePrime = true;
    return 1;
}

int cloud_fullWaterChange_pwd(String extra) {
    int passcode = extra.toInt();
    if (passcode == 123) {
        flag__cloud_fullWaterChange_pwd = 1;
        return 1;
    } if (passcode == 456) {
        flag__cloud_fullWaterChange_pwd = 2;
        return 2;
    } else {
        return -1;
    }
}

int cloud_restart_pwd(String extra) {
    int passcode = extra.toInt();
    if (passcode == 123) {
        emergencyIODisable();
        System.reset(RESET_NO_WAIT);
        return -2;
    } else {
        return -1;
    }
}

int cloud_safeMode_pwd(String extra) {
    int passcode = extra.toInt();
    if (passcode == 123) {
        emergencyIODisable();
        System.enterSafeMode(RESET_NO_WAIT);
        return -2;
    } else {
        return -1;
    }
}

int cloud_mainPumpTest_pwd(String extra) {
    int passcode = extra.toInt();
    if (passcode == 123) {
        flag__cloud_mainPumpTest_pwd = true;
        return 1;
    } else {
        return -1;
    }
}

int cloud_drainTank_pwd(String extra) {
    int passcode = extra.toInt();
    if (passcode == 123) {
        flag__cloud_drainTank_pwd = true;
        return 1;
    } else {
        return -1;
    }
}


/*
int cloud_ioTest1(String extra) {
    //beepBuzzer(2);
    
    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_1, HIGH);
    delay(1000);
    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_1, LOW);
    delay(1000);
    //beepBuzzer(1);

    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_2, HIGH);
    delay(1000);
    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_2, LOW);
    delay(1000);
    //beepBuzzer(1);

    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_3, HIGH);
    delay(1000);
    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_3, LOW);
    delay(1000);
    //beepBuzzer(1);

    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_4, HIGH);
    delay(1000);
    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_4, LOW);
    delay(1000);
    //beepBuzzer(1);

    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_5, HIGH);
    delay(1000);
    mixingStation.digitalWriteSafe(MixingStationIO::Components::TMP_OUTPUT_5, LOW);
    delay(1000);
    //beepBuzzer(1);

    mixingStation.digitalWriteSafe(MixingStationIO::Components::MAIN_PUMP, HIGH);
    delay(1000);
    mixingStation.digitalWriteSafe(MixingStationIO::Components::MAIN_PUMP, LOW);

    //beepBuzzer(2);
    
    return 1;
}
*/

int cloud_injectForSec(String extra) {
    float seconds = extra.toFloat();
    if (seconds <= 0 || seconds > 10) {
        return -1;
    } else {
        flag_injectForSec = seconds;
    }
    return 200;
}

int cloud_dosePumpNum(String extra) {
    int pumpNum = extra.toInt();
    if (pumpNum < 1 || pumpNum > 3) {
        flag_injectForSecPumpNum = -1;
    } else {
        flag_injectForSecPumpNum = pumpNum;
    }
    flag_injectForSec = 0; // reset this to avoid triggering an injection
    return flag_injectForSecPumpNum;
}


int cloud_watchdogTest(String extra) {
    hwWatchdog.disable();
    beepBuzzer(3);
    delay(HW_WATCHDOG_TIMEOUT_SEC*1000 + 2000);
    beepBuzzer(3);
    delay(500);
    if (hwWatchdog.enable()) {
        beepBuzzer(4);
        delay(HW_WATCHDOG_TIMEOUT_SEC*1000 + 2000);
        beepBuzzer(4);
        hwWatchdog.stopPetTimer();
        delay(HW_WATCHDOG_TIMEOUT_SEC*1000 + 2000);
        // should never get here
    } else {
        beepBuzzer(5);
    }
    return 1;
    /*
    wd->dispose();
    wd = new ApplicationWatchdog(1000*3, swWatchdogHandler, SW_WATCHDOG_STACK_SIZE);
    while(1);
    return 1;
    */
}

int cloud_getCO2BPSx100(String extra) {
    return co2BubbleSensor.getBubblesPerSecondCombined()*100;
}

int cloud_deadlockTest(String extra) {
    
    //hardwareWatchdog.pet();
//return hardwareWatchdog.isEnabled() ? 2 : 3;    
    
    // note: bubbles must be active here
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.send_string("Testing...");

    int count = 0;    
    unsigned long start = millis();
    while (millis() - start < 60*1000) {
        lcd.setCursor(0,1);
        lcd.send_string("Count: " + String(round((millis() - start)/1000.0)) + "   ");
        wd->checkin(); // avoid application wd triggering
        delay(500);
        //rampPump(true, 20000, true);
        //rampPump(false, 20000, true);
    }
    //digitalWrite(PIN__PUMP_CONTROL, LOW);
    WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, LOW); }
    return 1;
}

int HW_WD_toggle(String extra) {
    //return rtcModule.setAlarm1(rtcModule.now() + TimeSpan(70), DS3231_A1_Second); // 
    //return rtcModule.setAlarm1(rtcModule.now() + TimeSpan(70), DS3231_A1_Second);
    
    hwWatchdog.stopPetTimer();
    return 1;
}

int cloud_HW_WD_test(String extra) {
    hwWatchdog.stopPetTimer();
    delay(HW_WATCHDOG_TIMEOUT_SEC*1000 + 2*1000);
    // note: WDT should reset MCU and we should never get here...
    // TODO: make sure you get expected reset code (e.g. delaying too long here in cloud function could possibly cause reset?) 
    hwWatchdog.startPetTimer();
    return -1;
}
/*
    if (temp_milliseconds != 0) {
        Particle.publish("temp_milliseconds", String(temp_milliseconds));
        temp_milliseconds = 0;
    } else {
        float ms = extra.toInt();
        beepBuzzer(1);
        if (hwWatchdog.setOneOffWatchdogAlarmThenDisable__see_notes(ms)) {
            unsigned long start = millis();
            while (1) {
                temp_milliseconds = millis() - start;
                delay(1);
            }
        } else {
            return -1;
        }
    }
    
    return 1;
*/
/*
    beepBuzzer(1);
    hwWatchdog.disable();
    
    delay(3000);
    
    // stop SW watchdog
    beepBuzzer(2);
    wd->dispose();
    
    beepBuzzer(10);


return 123;

    float ms = extra.toInt();
    return hwWatchdog.stopPetTimerAndSetOneTimeAlarm_danger(ms, wd);
    */
//}

int cloud_deadlockTest_old_but_worked(String extra) {
    // note: bubbles must be active here
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.send_string("Testing...");

    int count = 0;    
    unsigned long start = millis();
    while (millis() - start < 10*1000) {
        lcd.setCursor(0,1);
        lcd.send_string("Count: " + String(count++));
        rampPump(true, 20000, true);
        rampPump(false, 20000, true);
    }
    //digitalWrite(PIN__PUMP_CONTROL, LOW);
    WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, LOW); }
    return 1;
}

int cloud_dosingPump1Test(String extra) {
    // bool startDosingPumpSafe(int dosingPumpNum, float doseMilliliters, long &lastRunEpoch_retained) {
    //long tmp = -1;
    //return startDosingPumpSafe(1, 0.05, tmp);
    return -1;
}

//void cloud_schedulerTest1_dummyHandler() {
//}

/*
int cloud_schedulerTest1(String extra) {
    long originalTime = Time.now();
    unsigned long startMillis = millis();
    
    long simulatedTime = originalTime;
    const int MAX_COUNT = 4;
    const int INCREMENT_MINUTES = 60;
    int count = 0;
    
    SimpleScheduler dosingPump1Scheduler("Test task", Time.hour(), Time.minute(), 2, SimpleScheduler::DayOfWeek::ANY, cloud_schedulerTest1_dummyHandler, 2100);
    dosingPump1Scheduler.setLastRunEEPROM(0xFFFFFFFF);
    
    do {
        simulatedTime = simulatedTime + INCREMENT_MINUTES*60;
        Time.setTime(simulatedTime);
        Particle.publish("simulated time", Time.format(Time.now(), "%Y-%m-%d %H:%M:%S"));
        
        dosingPump1Scheduler.loop();
        delay(1000);
        dosingPump1Scheduler.loop();
        delay(1000);
        //dosingPump1Scheduler.loop();
        //delay(1000);
        
        delay(3*1000);
    } while (++count <= MAX_COUNT);
    
    unsigned long elapsedTimeMs = millis() - startMillis;
    Time.setTime(originalTime + elapsedTimeMs/1000.0);
    Particle.publish("final time", Time.format(Time.now(), "%Y-%m-%d %H:%M:%S"));
    return 1;
}*/

int cloud_schedulerTest2(String extra) {
    // helper to debug scheduled task at a specific time
    
    Time.setTime(1663963650);
    Particle.publish("simulated time", Time.format(Time.now(), "%Y-%m-%d %H:%M:%S"));
    //dosingPump1Scheduler.setLastRunEEPROM(0xFFFFFFFF);

    //testTask2.loop(); // EEPROM value before test: 1663952826

    Particle.syncTime();
    while(Particle.syncTimePending()) {
        Particle.process();
    };
    Particle.publish("restored time", Time.format(Time.now(), "%Y-%m-%d %H:%M:%S"));
    return 1;

}

int cloud_readEEPROMLong(String extra) {
    int addr = extra.toInt();
    long lastRunEpoch;
    EEPROM.get(addr, lastRunEpoch); // will return 0xFFFFFFFF if no existing value has been stored
    Particle.publish("EEPROM long int", String(lastRunEpoch));
    return (int)lastRunEpoch; // may overflow
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

/*
int getPumpSpikes100ms() {
    int zeros = 0;
    int spikes = 0;
    unsigned long start = millis();
    do {
        float v = readMotorVoltageDrop();
        if (v == 0) {
            zeros++;
        }
        if (v > 0.05) { // this one looks even better. add up the counts per 100ms and (4 vs 15 after 10 repetitions in your test). every 100ms set had 1 (some more) when no water
            spikes++;
        }

    } while (millis() - start <= 100);    
    
    //return (float)spikes/zeros;
    return spikes;
}
*/

float tmp_min;
float tmp_max;
float tmp_max2;
float tmp_max3;
float tmp_max4;
/*
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
        //delay(1);
        //runningAvg.addValue(voltageDrop);
        //delayAndMotorSense(500); // testing -- does pump cause sensor output to spike falsely?
    } while (millis() - start <= 100);
    
    digitalWrite(LED, LOW);
    return numSpikes;
}
*/

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
    
    //digitalWrite(PIN__PUMP_CONTROL, LOW);
    WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, LOW); }
    
    return 123;
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
float readMotorVoltageDrop() {
    
    // 12v -- motor pin -- motor pin -- MOSFET -- GND
    //                       /\
    //                        | 10x voltage divider -- sense pin
    float volts = analogRead(PIN__PUMP_V_SENSE);
    volts = mapFloat(volts, 0, 4095, 0, 3.3);

    return volts;

}
*/

/*
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
*/

void reset_handler(system_event_t event, int param) {
    if (param == firmware_update_begin) {
        firmwareUpdateBegun = true;
        
        // TODO: centralize?
        //digitalWrite(PIN__PUMP_CONTROL, LOW);
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__TOPOFF_SOLENOID, LOW); }
        digitalWrite(PIN__DOSING_PUMP_1, LOW);
        digitalWrite(PIN__DOSING_PUMP_2, LOW);
        digitalWrite(PIN__DOSING_PUMP_3, LOW);

        // turn off hw watchdog timer to ensure we don't brick the device by accident
        // TODO: clean up this code
        hwWatchdog.disable();
        
        // stop SW watchdog
        wd->dispose();

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.send_string("Updating");
        lcd.setCursor(0,1);
        lcd.send_string("firmware...");

        // note: loop() will continue executing after this (or just in the background?)
    }
    
}














 
/************************************************************************************/
/* Mixing station controller code below                                             */
/************************************************************************************/

// define helper macros to use with mixingStationStrBuff
#define SET_MIXING_STATION_STR(...) strncpy(mixingStationStrBuff, __VA_ARGS__, sizeof(mixingStationStrBuff)-1)
#define GET_MIXING_STATION_CSTR(...) String(mixingStationStrBuff).c_str()

// string reference:
/*
SET_MIXING_STATION_STR("Mixing station I/O test message #1");
lcd.send_string(String::format("test: %s", GET_MIXING_STATION_CSTR()));
Particle.publish("str test 3", String::format("test: %s", GET_MIXING_STATION_CSTR()));
///
strncpy(mixingStationStrBuff, "abcdefghijklmnopqrstuvwxyz", sizeof(mixingStationStrBuff)-1);
Particle.publish("str test 2", mixingStationStrBuff);
lcd.send_string("test: " + String(mixingStationStrBuff)); // works
lcd.send_string(String::format("test 2: %s", String(mixingStationStrBuff).c_str())); // works
*/  

bool isLiquidDetectedTop() {
    return (digitalRead(PIN__LIQUID_LEVEL_SENSOR_CAPACITIVE) == ACTIVE_LOW);
}

bool isLiquidDetectedBottom() {
    return (digitalRead(PIN__LOWER_LIQUID_LEVEL_SENSOR_CAPACITIVE) == ACTIVE_LOW);
}

/*
bool runMixingStation_pumpReservoirToAquarium(int passcode) {
////    lcd.display("Checking water", "sensor...", 1);

    // make sure water is detected at the top of the tank. if not, top off the tank to confirme that that sensor works.
    bool success = doWaterLevelCheck();
    if (!success) {
        SET_MIXING_STATION_STR("Error topping off tank (doWaterLevelCheck())");
        return false;
    }
    // note: we add an additional check here since doWaterLevelCheck() is hacky and could under some circumstances skip topoff, e.g. if the heater state is set to disabled
    if (!isLiquidDetectedTop()) {
        SET_MIXING_STATION_STR("Error topping off tank (second check)");
        return false;
    }    
    
    
    success = mixingStation.pumpWaterToAquarium(passcode, isLiquidDetectedTop);
    
    return success;

}
*/

bool disableFilterAndHeatWithAlert() {
    // turn off canister filter and verify that flow rate is zero
    lcd.display("Canister filter", "shutting off", 1);
    relayModule.setCanisterFilter(false);
    
    if (setting__disableFlowRateSensor) {
        lcd.display("Bypassing flow", "rate check..", 3);
    } else {
        lcd.display("Verifying flow", "rate...", 0);
        // leave plenty of time for pump to spin up, esp in case of air bubble
        for (int i=3; i>=0; i--) {
            lcd.setCursor(15,1);
            lcd.send_string(String(i));
            delay(1000);
        }    
        float flowRate = flowRateSensor.readFlowRateGPH(true);
        if (flowRate > 0.01) {
            relayModule.setCanisterFilter(true);
            lcd.display("Error: Flow rate", "too high", 0);
    
            static PushNotification notification_disableFilterAndHeatWithAlert(60min);
            notification_disableFilterAndHeatWithAlert.sendWithCooldown(String::format("ERROR: Flow rate was not zero after shutting off canister filter (was %.2f GPH). Turning filter back on.", flowRate));
            delay(1000);
            lcd.clear();
            return false; // indicates that canister filter is left turned back on, and heat is unchanged
        } 
    }
    
    relayModule.setHeater(false);
    lcd.display("... success.", "Heat turned off.", 2);
    return true; // indicates that canister filter and heat are both turned off
}

bool enableFilterAndHeatWithAlert() {
    // turn on canister filter and verify that flow rate is > zero
    lcd.display("Canister filter", "turning on", 1);

    if (setting__disableFlowRateSensor) {
        lcd.display("Bypassing flow", "rate check..", 3);
        relayModule.setCanisterFilter(true);
        relayModule.setHeater(true);
        return true; // indicates that canister filter and heat are both turned on
    } else {
        lcd.display("Verifying flow", "rate...", 0);
        relayModule.setCanisterFilter(true);
        // leave plenty of time for pump to spin up, esp in case of air bubble
        for (int i=5; i>=0; i--) {
            lcd.setCursor(15,1);
            lcd.send_string(String(i));
            delay(1000);
        }
    
        // new: try several times if we don't get an accurate reading right away
        float flowRate;
        for (int i=0; i<3; i++) {
            flowRate = flowRateSensor.readFlowRateGPH(true);
            if (flowRate > 0.05) {
                relayModule.setHeater(true);
                lcd.display("... success.", "Heat turned on.", 2);
                return true; // indicates that canister filter and heat are both turned on
            }
        }
    
        // if we get here, the flow rate never picked up the way we expected it to    
        flag__flowRateTooLowAfterEnablingCanisterFilter = true;
        relayModule.setCanisterFilter(false);
        lcd.display("Error: Flow rate", "too low", 0);
        static PushNotification notification_enableFilterAndHeatWithAlert(60min);
        notification_enableFilterAndHeatWithAlert.sendWithCooldown(String::format("*ERROR*: Flow rate was too low (%.2f GPH) after powering back on. Leaving filter (and heat) off. Check manually ASAP.", flowRate), true);
        delay(1000);
        lcd.clear();
        return false; // indicates that canister filter is left off, and heat is unchanged
    }
}

// TODO: decide on refactoring
bool mixingStation_drainAquarium(int passcode, bool leaveValvesFilterAndHeaterForNextStage=false) {
    if (passcode != 123) {
        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] WARNING: mixingStation_drainAquarium() called without correct passcode""\" }"), PRIVATE);
        return false;
    } 
    
    lcd.display("Partial water", "drain...", 0);
    Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Draining aquarium...""\" }"), PRIVATE);
    
    // make sure we're reading a signal from the lower water sensor
    if (!isLiquidDetectedBottom()) {
        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Cancelling partial drain: Water not detected at lower sensor""\" }"), PRIVATE);
        return false;
    }
    
    // turn off canister filter and verify that flow rate is zero
    if (!disableFilterAndHeatWithAlert()) { //<-- left off here
        // note: push alert will already have gone out if we get here
        return false;
    }

    // ============================================================================================================
    // NOTE: Do not return below this line without first calling enableFilterAndHeatWithAlert() ****
    // ============================================================================================================
    
    #define PARTIAL_DRAIN_TIMEOUT_SEC 200 // was 75 prior to temp drain hookup in garage // 55 //40 // need to test once more, but I think 40 sec is a pretty accurate threshold here 
    unsigned long start = millis();
    unsigned long secondsLastUpdated = 0;
    particle::Future<bool> publishFuture; // use to make Publish() asynchronous so it doesn't block (and the pump can't get hung stuck on). details: https://docs.particle.io/firmware/low-power/stop-sleep-cellular/#the-future
    bool error = false;

    // actuate electric ball valves to open drain and cut off path to canister filter
    lcd.display("Opening", "valves...", 1);
    WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, LOW); }
    delay(3*1000); // give head start to cutoff valve
    digitalWrite(PIN__DRAIN_BALL_VALVE, HIGH);

    lcd.display("Monitoring water", "", 0); // leave 2nd line for countdown
    
    // NOTE: never return without checking / re-enabling canister filter and heater
    do {
        // sanity check for connection, in case we have to literally pull the plug while in progress
        if (!mixingStation.isConnected()) {
            lcd.display("Error: IO board", "disconnected", 3, true);
            publishFuture = Particle.publish("mixingStation_drainAquarium()", String::format("Error: IO board disconnected during drain operation"));
            error = true;
            break;
        }

        if (millis() - start >= PARTIAL_DRAIN_TIMEOUT_SEC*1000) {
//*TODO* Don't delay 3 seconds -- close valve ASAP            
            lcd.display("Error: Timed out", String::format("after %d ms", PARTIAL_DRAIN_TIMEOUT_SEC*1000), 1);
            publishFuture = Particle.publish("pumpWaterToAquarium()", String::format("Error: Partial drain timed out after %d ms", PARTIAL_DRAIN_TIMEOUT_SEC*1000));
            error = true;
            break;
        }    

        // print seconds to date so we know we haven't frozen
        if (millis() - secondsLastUpdated >= 1000) {
            secondsLastUpdated = millis();
            lcd.setCursor(0,1);
            lcd.send_string(String::format("%d   ", (int)(PARTIAL_DRAIN_TIMEOUT_SEC - (millis() - start)/1000.0)));
        }
        
        Particle.process();

    } while (isLiquidDetectedBottom());
    
    lcd.display("Actuating ball", "valve(s)...", 0);
    
    // close drain valve
    digitalWrite(PIN__DRAIN_BALL_VALVE, LOW);

    if (leaveValvesFilterAndHeaterForNextStage) {
        delay(7*1000); // wait for single ball valve to close
    } else {
        // open cutoff valve
        WITH_LOCK(Wire) { mcp2.digitalWrite(PIN_IOEXP2__CUTOFF_BALL_VALVE, HIGH); }
        delay(7*1000); // wait for both ball valves to close

        // turn on canister filter and verify that flow rate is > zero
        if (!enableFilterAndHeatWithAlert()) {
            // note: alert will already have gone out in this case
            return false;
        }
    }
    
    // ============================================================================================================
    // NOTE: It's safe to return without calling enableFilterAndHeatWithAlert() below this line
    // ============================================================================================================

    if (error) {
        Particle.publish("mixingStation_drainAquarium()", String::format("Error draining water"));
        return false; // note: if leaveValvesFilterAndHeaterForNextStage is true, the wrapper function will ensure the cutoff valve is opened back up eventually
    } else {
        Particle.publish("mixingStation_drainAquarium()", String::format("Success!"));
    }
    
    return true;
}


/*
    Notes: MECE JTBD
    - fully automated draining and filling
    - canister filter cleaning: locking up system while canister filter is detacched and cleaned, then fill a little and turn everything back on
    - gravel siphon mode: drain the tank a little bit, then fill a medium/large amount after cleaning is complete
    
    - debugging mode: step through each step for any mode above
    - manual drain, manual mixing tank refill, manual pump to aquarium: manually start either a drain or fill operation, to save time and water in case something went wrong
    
    SO THEREFORE: Stages to consider:
    - Turning off canister filter, heater, CO2, and closing electric ball valve (and do various safety checks)
    - Filling mixing tank with specified volume of water (note: do this and confirm it worked before draining the tank at all)
    - Draining aquarium (either to sensor level or for a number of seconds)
    - Pumping water from mixing tank to aquarium
    - Turning everything back on (and confirming that flow rate/etc fall within expected ranges)
*/

/*
enum WaterChangeMode {
    FULL_WATER_CHANGE, // does everything
    REFILL_AFTER_CANISTER_FILTER_CLEANING, // refills tank a bit
    SIPHON_AND_REFILL_FOR_GRAVEL_CLEANING // both drains a bit of water to recharge siphon pressure, and refills tank // TODO: prompt user to siphon or not?
    
    MANUAL_DRAIN_TANK_ONLY,
    MANUAL_FILL_RESERVOIR_ONLY,
    MANUAL_PUMP_TO_TANK_ONLY
};

enum WaterChangeState {
    FILL_MIXING_TANK_FULL,
    FILL_MIXING_TANK_CANISTER, // TODO: rename to represent small water changes
    STOP_AQUARIUM,
    DRAIN_AQUARIUM,
    PUMP_TO_AQUARIUM,
    RESTORE_AQUARIUM
};

enum WaterChangeAutomationLevel {
    NOT_AUTOMATED, // requires button press at each step
    SEMI_AUTOMATED, // requires button press at each major step
    FULLY_AUTOMATED, // runs start to finish automatically; adds more conservative checks as well
};

bool runWaterChange(WaterChangeMode waterChangeMode, WaterChangeAutomationLevel, automationLevel) {
    bool success = false;
    
    switch(waterChangeMode) {
        case WaterChangeMode::FULL_WATER_CHANGE:
            
            if (automationLevel == WaterChangeAutomationLevel::    )
            
            // TODO: pre-checks
            
            success = enterWaterChangeState(WaterChangeState::FILL_MIXING_TANK_FULL);
            if (!success) {
                // show error here
                return false;
            } 
            
            success = enterWaterChangeState(WaterChangeState::STOP_AQUARIUM);
            if (!success) {
                // show error here
                return false;
            } 




            
            
            break;
        // ...
        default:
            return false;
    };
}

bool enterWaterChangeState(WaterChangeState waterChangeState) {
    switch (waterChangeState) {
        case WaterChangeState::FILL_MIXING_TANK:
        
            break;
        // ...
        default:
            return false;
    };
}
*/

bool runFullWaterChangeAutomatic_internalOnly(WaterChangeMode waterChangeMode, WaterChangeAutomationLevel automationLevel) {
    // bool requireButtonPresses=true, bool minimumPressesOnly=false, bool limitedTestMode=false, bool debug_skipFillingSteps=false, bool fullyAutomaticMode=false) {
    
    
    
    
    
    // TODOs: 
    //   review and fine-tune not_automated vs semi_automated
    //   check wrapper function, e.g. how it handles error strings
    //   play tune
    //   play errorBeep() in wrapper function if an error occurs
    //   add expiring prompt to user before water change starts
    //   add expiring prompt to add in gravel filtering
    
    
    
    bool success = false;
    // TODO: add mode for smaller fills, e.g. after cleaning tank with siphon

    if (automationLevel == WaterChangeAutomationLevel::NOT_AUTOMATED) {
        lcdDisplayWaitPressButton("Start pre-", "checks?");
    } else {
        lcdDisplayWaitPressButton("Starting pre-", "checks...", 3);
    }

    /** check for expected readings before starting **/
    
    // make sure the mixing station IO board is connected
    if (!mixingStation.isConnected()) {
        SET_MIXING_STATION_STR("Mixing station I/O not connected");
        return false;
    }
    
    // make sure water isn't detected in dosing pump dispensing chamber
    if (mixingStation.digitalReadOrReset(MixingStationIO::Components::DOSING_PUMP_SENSOR) == ACTIVE_LOW) {
        SET_MIXING_STATION_STR("Liquid detected in dosing pump dispenser");
        return false;
    }
    
    // ensure flow rate is > 10 GPH
    float flowRate = flowRateSensor.readFlowRateGPH(true);
    if (flowRate <= 10 && !setting__disableFlowRateSensor) {
        SET_MIXING_STATION_STR(String::format("Expected flow rate to be > 10 GPH. Was %.2f GPH.", flowRate));
        return false;
    }
    
    if (automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED) { // additional checks to be more conservative if in fully automatic mode
        if (!Particle.connected()) {
            SET_MIXING_STATION_STR("Must be online to send messages in fully-automated mode");
            return false;            
        }
        
        // make sure water is detected at the top of the tank. if not, top off the tank to confirme that that sensor works.
        success = doWaterLevelCheck(true);
        if (!success) {
            SET_MIXING_STATION_STR("Tank top-off check failed");
            return false;
        }
        // note: we add an additional check here since doWaterLevelCheck() is hacky and could under some circumstances skip topoff, e.g. if the heater state is set to disabled
        if (!isLiquidDetectedTop()) {
            SET_MIXING_STATION_STR("Water not detected at upper tank sensor");
            return false;
        }
        
        // check for aquarium lower bound water sensor
        if (!isLiquidDetectedBottom()) {
            SET_MIXING_STATION_STR("Water not detected at lower tank sensor");
            return false;
        }
    }
    
    lcd.display("... done", "", 1);
    Particle.publish("Water change", "Completed pre-checks");
    delay(500);

    // note: we store CO2 state outside of this function so that it can be restored regardless of if/when we return

    // make sure water isn't detected in the mixing reservoir. if it is, drain the tank
    if (waterChangeMode != WaterChangeMode::SKIP_MIXING_TANK_FILL) {

        /** drain mixing tank **/
        
        if (mixingStation.digitalReadOrReset(MixingStationIO::Components::CALIBRATION_WATER_SENSOR) == ACTIVE_LOW) { // water detected in mixing station reservoir
            Particle.publish("Water change", "Note: Water detected in mixing tank");
            lcd.display("(Note: Water", "in mix tank)", 3);
        } 
        
        if (automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED || automationLevel == WaterChangeAutomationLevel::SEMI_AUTOMATED) {
            lcdDisplayWaitPressButton("Draining mixing", "tank...", 3);
        } else if (automationLevel == WaterChangeAutomationLevel::NOT_AUTOMATED) {
            lcdDisplayWaitPressButton("Drain mixing", "tank?");
        }

        Particle.publish("Water change", "Draining mixing tank...");
        delay(250);
        
        success = mixingStation.drainReservoir();
        if (!success) {
            SET_MIXING_STATION_STR("Draining reservoir failed");
            return false;
        }
        
        /** add Seachem Prime **/
        
        if (automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED || automationLevel == WaterChangeAutomationLevel::SEMI_AUTOMATED) {
            lcdDisplayWaitPressButton("Dosing with", "Prime...", 3);
        } else if (automationLevel == WaterChangeAutomationLevel::NOT_AUTOMATED) {
            lcdDisplayWaitPressButton("Dose with Prime?");
        }
    
        int numTries = 1;
        Particle.publish("Water change", "Dosing with Prime...");
        success = mixingStation.dosePrime();
        if (!success) { 
            numTries++;
            lcd.display("Dosing failed", "trying again...", 2);
            Particle.publish("Water change", "Dosing failed. Trying again...");
            success = mixingStation.dosePrime();
            if (!success) { // TODO: refactor
                numTries++;
                lcd.display("Dosing failed", "trying again...", 2);
                Particle.publish("Water change", "Dosing failed. Trying again...");
                success = mixingStation.dosePrime();
            }
            if (!success) {
                SET_MIXING_STATION_STR("Seachem Prime dosing failed");
                return false;
            }
        } else {
            Particle.publish("Water change", "Dosing succeeded on try " + String(numTries));
        }
        
        /** fill mixing tank **/
        
        if (automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED || automationLevel == WaterChangeAutomationLevel::SEMI_AUTOMATED) {
            lcdDisplayWaitPressButton("Filling mixing", "tank...", 3);
        } else {
            lcdDisplayWaitPressButton("Fill mixing", "tank?");
        }
        Particle.publish("Water change", "Filling mixing tank...");
        delay(500);
        
        bool cancelledViaButtonPress = false;
        unsigned long debug_numInvalidReadings = 0;
        success = mixingStation.dispenseWater(TANK_WATER_CHANGE_GALLONS, cancelledViaButtonPress, debug_numInvalidReadings);

        if (success && !cancelledViaButtonPress && debug_numInvalidReadings == 0) {
            //PushNotification::send("Finished filling mixing tank with RODI water"); // message is sent below
            Particle.publish("Water change", "Mixing tank filled successfully");
            delay(500);
        } else {
            PushNotification::send(String::format("Failed to fill mixing tank. success: %d, cancelledViaButtonPress: %d, debug_numInvalidReadings: %u", success, cancelledViaButtonPress, debug_numInvalidReadings));
            SET_MIXING_STATION_STR("Failed to fill mixing tank. success: " + String(success));
            return false;
        }
        
    }
    
    if (waterChangeMode == WaterChangeMode::SIPHON_CLEANING_MODE) {
        // TODO: TANK_WATER_CHANGE_GALLONS -->         #define TANK_WATER_CHANGE_GALLONS               4.25 //5.25 // Note: Current tank is approx 11.5" x 23", with an average depth of 3.5" between liquid level sensors. That equals 4 gallons, so we'll plan for 5.25 to be safe
        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"%s""\" }", "Aquarium is ready for siphoning."), PRIVATE);
        lcdDisplayWaitPressButton("Start siphoning;", "press to fill");
    } else {
        // new
        PushNotification::send("Aquarium is ready for siphoning. Press to pause within 2 minutes.");
        bool buttonPressed;
        buttonPressed = lcdDisplayWaitPressButton("Pause to", "siphon gravel?", 120);
        if (buttonPressed) {
            beepBuzzer(1);
            lcdDisplayWaitPressButton("Press to finish", "draining tank", -1);
        }

        /** drain aquarium **/
        if (automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED) {
            lcdDisplayWaitPressButton("Draining", "aquarium...", 3);
        } else {
            Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"%s""\" }", "Mixing tank filling complete. Ready to drain aquarium."), PRIVATE);
            lcdDisplayWaitPressButton("Drain aquarium?");
        }
        Particle.publish("Water change", "Draining aquarium (and checking/disabling canister filter and heater)...");
        delay(500);
    
        // =====================================================================================================================================
        // NOTE: Be careful ensuring that the state of electric ball valves as well as the canister filter and heater are maintained below
        // =====================================================================================================================================
    
        success = mixingStation_drainAquarium(123, true); // NOTE: we add flag to leave canister filter and heat off so we can move seamlessly to the next stage afterwards
        if (!success) {
            PushNotification::send("Warning: Aquarium partial drain returned fault flag, but will still continuing with refill process.");
            Particle.publish("Water change", "Warning: Aquarium partial drain returned fault flag, but will still continuing with refill process.");
            delay(1000);
            //SET_MIXING_STATION_STR("Draining aquarium failed");
            //return false; // note: no longer want to do this in case we time out -- we should continue refilling
        }    
        
        // TODO: shut off pump (and heater), set electric ball valves, (if !limitedTestMode) and wait a moment for flow to settle, etc.
        
        // ensure flow rate is zero
        
        // TODO: do we need to call co2BubbleSensor.stopInterrupts() too? // required to avoid system deadlock of some kind when rampPump() is called
    
        /** wait if user wants to clean the tank **/
        
        // let user press button to hold while they clean the tank
        PushNotification::send("Aquarium is ready for cleaning before filling back up. Press to pause within 2 minutes.");
        buttonPressed = lcdDisplayWaitPressButton("Pause to clean", "tank?", 120);
        if (buttonPressed) {
            beepBuzzer(1);
            delay(500);
            lcdDisplayWaitPressButton("Press to fill", "tank", -1);
        }
    }
    
    /** pump water up **/

    if (automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED) {
        lcdDisplayWaitPressButton("Pumping to", "aquarium...", 3);
    } else {
        lcdDisplayWaitPressButton("Pump to", "aquarium?");
    }

    // pump water up to aquarium
    Particle.publish("Water change", "Pumping water to aquarium...");
    success = mixingStation.pumpWaterToAquarium(123, isLiquidDetectedTop);
    if (success) {
        PushNotification::send("Finished pumping water to aquarium");
        Particle.publish("Water change", "Finished pumping water to aquarium");
        delay(1000);
    } else {
        SET_MIXING_STATION_STR("Pumping water to aquarium failed");
        return false;
    }
    
    // =====================================================================================================================================
    // End note re: state of electric ball valves as well as canister filter and heater
    // =====================================================================================================================================

    return true;
}

bool old__runFullWaterChangeAutomatic_internalOnly(bool requireButtonPresses=true, bool minimumPressesOnly=false, bool limitedTestMode=false, bool debug_skipFillingSteps=false) {
    
    bool success;
    // TODO: add mode for smaller fills, e.g. after cleaning tank with siphon
    
    /*
    if (!limitedTestMode && flowRateSensor.readFlowRateGPH() > 0) {
        SET_MIXING_STATION_STR("[testing up front for now] Flow rate expected to be zero");
        return false;
    }
    */
    
    /** check for expected readings before starting **/
    
    // make sure the mixing station IO board is connected
    if (!mixingStation.isConnected()) {
        SET_MIXING_STATION_STR("Mixing station I/O not connected");
        return false;
    }
/**************    
    // make sure water is detected at the top of the tank. if not, top off the tank to confirme that that sensor works.
    success = doWaterLevelCheck();
    if (!success) {
        SET_MIXING_STATION_STR("Error topping off tank (doWaterLevelCheck())");
        return false;
    }
    // note: we add an additional check here since doWaterLevelCheck() is hacky and could under some circumstances skip topoff, e.g. if the heater state is set to disabled
    if (!isLiquidDetectedTop()) {
        SET_MIXING_STATION_STR("Error topping off tank (second check)");
        return false;
    }
*///////////
    
    // TODO: check for aquarium lower bound water sensor

    // make sure water isn't detected in the mixing reservoir. if it is, drain the tank
    if (!debug_skipFillingSteps) {
        if (mixingStation.digitalReadOrReset(MixingStationIO::Components::CALIBRATION_WATER_SENSOR) == ACTIVE_LOW/* || 
            mixingStation.digitalReadOrReset(MixingStationIO::Components::LOWER_WATER_SENSOR) == ACTIVE_LOW*/) {
                // water detected in mixing station reservoir
                if (requireButtonPresses && !minimumPressesOnly) {
                    lcdDisplayWaitPressButton("Water detected.", "Drain mix tank?");
                }
                float sensorDetectionPercent;
                mixingStation.drainReservoir(sensorDetectionPercent);
                delay(1000);
        }
    }

    // make sure water isn't detected in dosing pump dispensing chamber
    if (mixingStation.digitalReadOrReset(MixingStationIO::Components::DOSING_PUMP_SENSOR) == ACTIVE_LOW) {
        SET_MIXING_STATION_STR("Liquid detected in dosing pump dispenser");
        return false;
    }
    
    // ensure flow rate is > 1 before going further
    float flowRate = flowRateSensor.readFlowRateGPH(true);
    if (flowRate <= 1 && !setting__disableFlowRateSensor) {
        SET_MIXING_STATION_STR("Expected flow rate to be non-zero. Was " + String(flowRate));
        return false;
    }
    
    
    
    
    // note: we store CO2 state outside of this function so that it can be restored regardless of if/when we return

    if (!debug_skipFillingSteps) {
        /** start filling the mixing reservoir **/
        
        // fill tap water to predetermined level
        if (requireButtonPresses && !minimumPressesOnly) {
            lcdDisplayWaitPressButton("Add tap water?");
        }
        
        
        
        
        /////////// new filling version to test ///////////// --> todo: 12 gal total? check against existing setpoint

        /** drain mixing tank **/
        
        success = mixingStation.drainReservoir();
        if (!success) {
            lcd.display("Reservoir drain", "failed", 2);
            SET_MIXING_STATION_STR("Draining reservoir failed");
            return false;
        }
        
        /** add Seachem Prime **/
        
        if (requireButtonPresses && !minimumPressesOnly) {
            lcdDisplayWaitPressButton("Dose with Prime?");
        } else {
            delay(1000);
        }
    
        success = mixingStation.dosePrime();
        if (!success) {
            lcd.display("Dosing failed", "trying again", 2);
            success = mixingStation.dosePrime();
            if (!success) {
                SET_MIXING_STATION_STR("Seachem Prime dosing failed");
                return false;
            }
        }
        
        /** fill mixing tank **/
        
        bool cancelledViaButtonPress = false;
        unsigned long debug_numInvalidReadings = 0;
        success = mixingStation.dispenseWater(TANK_WATER_CHANGE_SIPHON_GALLONS, cancelledViaButtonPress, debug_numInvalidReadings);

        if (success && !cancelledViaButtonPress && debug_numInvalidReadings == 0) {
            //PushNotification::send("Finished filling mixing tank with RODI water"); // message is sent below
        } else {
            errorBeep();
            PushNotification::send(String::format("Failed to fill mixing tank. success: %d, cancelledViaButtonPress: %d, debug_numInvalidReadings: %u", success, cancelledViaButtonPress, debug_numInvalidReadings));
            SET_MIXING_STATION_STR("Failed to fill mixing tank");
            return false;
        }
        
        /////////// end /////////////
        
        
        
        ///////// old filling version below //////////
        /*
        float sensorDetectionPercent_debug;
        success = mixingStation.fill(MixingStationIO::WaterTypes::TAP, MixingStationIO::WaterTypes::TAP, sensorDetectionPercent_debug);
        Particle.publish("runFullWaterChangeAutomatic_internalOnly", "debug: tap water: sensorDetectionPercent_debug: " + String(sensorDetectionPercent_debug));

        if (!success) {
            SET_MIXING_STATION_STR("Filling with tap water failed");
            return false;
        } else {
            delay(1000);
        }
        PushNotification::send("Finished adding tap water to mixing tank");
        
        //* add Seachem Prime
        
        if (requireButtonPresses && !minimumPressesOnly) {
            lcdDisplayWaitPressButton("Dose with Prime?");
        } else {
            delay(1000);
        }
    
        success = mixingStation.dosePrime();
        if (!success) {
            SET_MIXING_STATION_STR("Seachem Prime dosing failed");
            return false;
        }
    
        //* add RODI water (or tap for testing). note that this takes a long time. 
        
        if (requireButtonPresses && !minimumPressesOnly) {
            lcdDisplayWaitPressButton("Add RODI water?");
        } else {
            delay(1000);
        }
    
        if (limitedTestMode) {
            // have some fun signaling that we're just going to stick with tap for testing
            for (int i=0; i<3; i++) {
                mixingStation.digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, HIGH);
                delay(250);
                mixingStation.digitalWriteOrReset(MixingStationIO::Components::TAP_WATER_SOLENOID, LOW);
                mixingStation.digitalWriteOrReset(MixingStationIO::Components::RODI_SOLENOID, HIGH);
                delay(250);
                mixingStation.digitalWriteOrReset(MixingStationIO::Components::RODI_SOLENOID, LOW);
                delay(250);
            }
            delay(500);
        }
    
        success = mixingStation.fill(limitedTestMode ? MixingStationIO::WaterTypes::TAP : MixingStationIO::WaterTypes::RODI, MixingStationIO::WaterTypes::RODI, sensorDetectionPercent_debug);
        Particle.publish("runFullWaterChangeAutomatic_internalOnly", "debug: RODI (or tap for debug) water: sensorDetectionPercent_debug: " + String(sensorDetectionPercent_debug));
        if (success) {
            PushNotification::send("Finished filling mixing tank with RODI water");
        } else {
            SET_MIXING_STATION_STR("Filling with RODI (or tap for debug) water failed");
            return false;
        }
    */
    /////////// end /////////////
    
    }
    








    
    /** drain aquarium **/
    
    if (requireButtonPresses) {
        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"%s""\" }", "RODI/tap filling complete. Ready to drain aquarium."), PRIVATE);
        lcdDisplayWaitPressButton("Drain aquarium?");
    } else {
        delay(1000);
    }

    // =====================================================================================================================================
    // NOTE: Be careful ensuring that the state of electric ball valves as well as the canister filter and heater are maintained below
    // =====================================================================================================================================

    success = mixingStation_drainAquarium(123, true); // NOTE: we add flag to leave canister filter and heat off so we can move seamlessly to the next stage afterwards
    if (!success) {
        PushNotification::send("Warning: Aquarium partial drain returned fault flag, but will still continuing with refill process.");
        //SET_MIXING_STATION_STR("Draining aquarium failed");
        //return false; // note: no longer want to do this in case we time out -- we should continue refilling
    }    
    
    // TODO: shut off pump (and heater), set electric ball valves, (if !limitedTestMode) and wait a moment for flow to settle, etc.
    
    // ensure flow rate is zero
    
    // TODO: do we need to call co2BubbleSensor.stopInterrupts() too? // required to avoid system deadlock of some kind when rampPump() is called

    /** pump water up **/
    if (requireButtonPresses) {//requireButtonPresses && !minimumPressesOnly) {
        lcdDisplayWaitPressButton("Pump to aquarium?");
    } else {
        delay(1000);
    }

    // pump water up to aquarium
    success = mixingStation.pumpWaterToAquarium(123, isLiquidDetectedTop);
    if (success) {
        PushNotification::send("Finished pumping water to aquarium");
    } else {
        SET_MIXING_STATION_STR("Pumping water to aquarium failed");
        return false;
    }
    
    // =====================================================================================================================================
    // End note re: state of electric ball valves as well as canister filter and heater
    // =====================================================================================================================================

    return true;
}

/*
void tryToRestartCanisterFilter() {
     do {

        // try toggling filter and checking for flow every so often
        static system_tick_t canisterFilterLastToggled = 0;
        if (millis() - canisterFilterLastToggled > 60*1000) {
            // try toggling canister filter in case that helps resume flow
            relayModule.setCanisterFilter(true);
            delay(2000);
            relayModule.setCanisterFilter(false);
            delay(2000);
            relayModule.setCanisterFilter(true);
            
            canisterFilterLastToggled = millis();
        }


        static system_tick_t flowRateLastChecked = 0;
        float lastFlowRate = 0;
        if (millis() - flowRateLastChecked > 1100) {
            flowRateLastChecked = millis();
            if (lastFlowRate > 0.1) {
                
                
PushNotification::send(String::format("*ERROR* Flow rate not detected after turning canister filter back on. Canister filter and heat are off. Check manually before turning back on."), true);                    
            }
        }
        Particle.process();
    } while(???);    
}
*/

void siphonCleaningMode() {
    int itemIndex = displayStrings(
        2,
        "Start now",
        "Exit"
    );
    
    if (itemIndex == 1) {
        return;
    } else if (itemIndex == 0) {
        // note: use GET_MIXING_STATION_CSTR() to get error string if success gets set to false
        bool success = runFullWaterChangeAutomatic(WaterChangeMode::SIPHON_CLEANING_MODE, WaterChangeAutomationLevel::SEMI_AUTOMATED); //FULLY_AUTOMATED);
        if (success) {
            lcd.display("Siphon mode", "Complete!!", 3);
            arpeggioBeep();
            delay(250);
        } else {
            lcd.display("Error:", GET_MIXING_STATION_CSTR(), 0, true);
            arpeggioBeep(true);
            errorBeep();
            PushNotification::send(String::format("Error completing siphon mode: %s", GET_MIXING_STATION_CSTR()));
            delay(250);
        }
    }
}

bool runFullWaterChangeAutomatic(WaterChangeMode waterChangeMode, WaterChangeAutomationLevel automationLevel) {
    return runFullWaterChangeAutomatic(waterChangeMode, automationLevel, false);
}

bool runFullWaterChangeAutomatic(WaterChangeMode waterChangeMode, WaterChangeAutomationLevel automationLevel, bool runFromScheduler=false) {
    // was: (bool requireButtonPresses=true, bool minimumPressesOnly=false, bool limitedTestMode=false, bool debug_skipFillingSteps=false) {
    //    if (automationLevel == WaterChangeAutomationLevel::NOT_AUTOMATED) {
    //    if (waterChangeMode != WaterChangeMode::SKIP_MIXING_TANK_FILL) {

    // new: make sure we don't run via scheduler right after a manual run
    if (runFromScheduler && fullWaterChangeLastRunEpoc_Retained > 0 && Time.isValid()) {
        float hoursSinceLastWaterChange = (Time.now() - fullWaterChangeLastRunEpoc_Retained)/60/60;
        if (hoursSinceLastWaterChange < 24*3) {
            PushNotification::send("Automatic water change already ran recently; skipping. hoursSinceLastWaterChange: " + String(hoursSinceLastWaterChange));
            return false;
        }
    }
    
    // TODO: major refactor
    
    arpeggioBeep(); // testing
    
    lcd.init(); // reset displays just in case downstairs is corrupted    
    
    if (automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED) {
        PushNotification::send("Automatic water change will start in 5 minutes");
        lcdDisplayWaitPressButton("Automatic water", "change in...", 5*60);
        PushNotification::send("Automatic water change is starting now");
    } else {
        lcdDisplayWaitPressButton("Press to begin", "water change");
    }
    Particle.publish("Water change", String::format("Starting water change. waterChangeMode: %d, automationLevel: %d", waterChangeMode, automationLevel));
    delay(500);
    
    // store CO2 state, and shut off if it's on
    bool wasCO2Active = co2BubbleSensor.isCO2Started();
    if (wasCO2Active) {
        co2BubbleSensor.stopCO2();
        Particle.publish("Water change", "Stopping CO2...");
        delay(250);
        //co2BubbleSensor.stopInterrupts(); // required to avoid system deadlock of some kind when rampPump() is called
    }

    // run the mixing station routine
    // note: use GET_MIXING_STATION_CSTR() to get error string if success gets set to false
    // runFullWaterChangeAutomatic_internalOnly(WaterChangeMode waterChangeMode, WaterChangeAutomationLevel automationLevel) 
    // was: (requireButtonPresses, minimumPressesOnly, limitedTestMode, debug_skipFillingSteps); // may set flag__flowRateTooLowAfterEnablingCanisterFilter
    bool success = runFullWaterChangeAutomatic_internalOnly(waterChangeMode, automationLevel);
    
    if (flag__flowRateTooLowAfterEnablingCanisterFilter) {

        // ** this is a bad failure mode, since we don't want to overheat the heater or break the canister filter if we can't get our flow rate back up; but it's also possible the flow meter clogged up, which happens sometimes. this needs manual inspection.
        PushNotification::send(String::format("*ERROR* Flow rate not detected after turning canister filter back on. Canister filter and heat are off. Check manually before turning back on. Checking for flow restart automatically in the meantime..."), true);
        delay(1000);

        Particle.publish("Water change", "Warning: flag__flowRateTooLowAfterEnablingCanisterFilter flag is set");
        delay(1000);

        // try once to toggle the filter and see if anything changes        
        if (enableFilterAndHeatWithAlert()) {
            PushNotification::send(String::format("Update: Flow rate now higher (%.2f GPH) after toggling canister filter. Leaving filter and heat on.", flowRate), automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED);
        } else {
            relayModule.setCanisterFilter(false);
            PushNotification::send(String::format("Update: Flow rate still too low (%.2f GPH) after toggling canister filter. Leaving filter off. Heater state: %d", flowRate, relayModule.isHeaterOn()), automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED);
        }
    } else {
        // ensure canister filter and heater are both turned back on
        relayModule.setCanisterFilter(true);
        relayModule.setHeater(true);
        Particle.publish("Water change", "Turning canister filter and heater back on...");
        delay(250);
    }

    if (wasCO2Active && co2BubbleSensor.isActiveTimeOfDay()) {
        //co2BubbleSensor.resetMonitoringStats(); // important to ensure that gaps in data readings don't cause false positive fault detections
        //co2BubbleSensor.startInterrupts();
        co2BubbleSensor.startCO2();
        Particle.publish("Water change", "Turning CO2 back on...");
    } else {
        Particle.publish("Water change", "Not turning CO2 back on...");
    }
    delay(250);
    
    // TODO: restore canister pump (and check flow rate), heater, etc
    
    /** report results **/
    
    fullWaterChangeLastRunEpoc_Retained = Time.now();
    
    lcd.clear();
    if (success) {
        lcd.display("Water change", "successful!", 0);
        arpeggioBeep();
        Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"%s""\" }", "Successfully completed aquarium water change"), PRIVATE);
        Particle.publish("Water change", "Water change successful!");
        delay(250);
    } else {
        lcd.display("Error:", GET_MIXING_STATION_CSTR(), 0, true);
        arpeggioBeep(true);
        errorBeep();
        PushNotification::send(String::format("Error completing aquarium water change: %s", GET_MIXING_STATION_CSTR()), automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED);
        Particle.publish("Water change", "Error running water change -- see logs");
        delay(250);
    }
    
    /** drain excess water from mixing station **/

    bool doNotDrain = false;
    if (automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED) {
        if (success) {
            lcdDisplayWaitPressButton("Drain mix", "tank?", 30);
        } else {
            doNotDrain = true;
            PushNotification::send("Warning: Not draining mixing tank in fully automatic mode; success == false");
        }
    } else {
        lcdDisplayWaitPressButton("Drain mix tank?");
    }    

    if (doNotDrain) {
        lcd.display("Error; skipping", "mix draining", 0);
    } else {
        float sensorDetectionPercent;
        success = mixingStation.drainReservoir(sensorDetectionPercent);
        if (success) {
            lcd.display("Success!", "");
            arpeggioBeep();
            Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"%s""\" }", "Successfully drained mixing tank post-water change"), PRIVATE);
        } else {
            arpeggioBeep(true);
            errorBeep();
            //SET_MIXING_STATION_STR("Draining mixing tank failed");
            PushNotification::send("Error draining mixing tank", automationLevel == WaterChangeAutomationLevel::FULLY_AUTOMATED);
            Particle.publish("Water change", "Error draining mixing tank -- see logs");
            delay(500);
            return false;
        }
    
    }
    
    delay(5000);
    lcd.clear();
    
    return success;
}

int displayStrings(int numStrings, const char* firstString, ...) {
	va_list args;
	va_start(args, firstString);

	const char* strings[numStrings];
	strings[0] = firstString;

	for (int i = 1; i < numStrings; i++) { // was i=1 and ++i
		strings[i] = va_arg(args, const char*);
	}

	va_end(args);

	int currentIndex = 0;
	bool exit = false;

	const char* prefix = "> ";
	while (!exit) {
		lcd.clear();
		lcd.setCursor(0,0); //?
		lcd.send_string(prefix);

		int availableSpace = 16 * 2 - strlen(prefix);
		int stringLength = strlen(strings[currentIndex]);
		int avail = availableSpace; //was availableSpace - 3

		if (stringLength > avail) {
			char buffer[availableSpace - 2]; // Space for string + null terminator
			strncpy(buffer, strings[currentIndex], avail - 3); // 3 for "..."?
			buffer[avail] = '\0';
			strcat(buffer, "...");
			lcd.send_string(buffer, 14);
		} else {
			lcd.send_string(strings[currentIndex], 14);
		}

		// wait for the primary button to be released after updating LCD display, since we'll loop here after it's pressed
		delay(200); // debounce
		while (temp_isButton1Pressed() == ButtonPress::PRESSED) {
			// Wait for a button press
    		Particle.process();
		}
		
		//delay(200); // Debounce delay

		// Wait for a button press
		while (temp_isButton1Pressed() != ButtonPress::PRESSED && temp_isButton2Pressed() != ButtonPress::PRESSED) {
    		Particle.process();
		}

		if (temp_isButton1Pressed() == ButtonPress::PRESSED) {
			currentIndex = (currentIndex + 1) % numStrings;
			/*while (digitalRead(PIN__PUSHBUTTON) == ACTIVE_LOW) {
				// Wait for button release
    			Particle.process();
	    	}*/
		} else if (temp_isButton2Pressed() == ButtonPress::PRESSED) {
			exit = true;
    		lcd.clear();
			// Wait for button release
			while (temp_isButton2Pressed() == ButtonPress::PRESSED) {
    			Particle.process();
			}
		}
	}

	return currentIndex;
}










