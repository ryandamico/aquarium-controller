// LEFT OFF: separate and simplify SW watchdog; refactor WatchdogTypes enum; finish HW class; test everything :-)

class HWWatchdog {
        
    private:
        int timeoutSeconds;
        int petWatchdogSeconds;
        RTC_DS3231 rtcModule;
        Timer petHWWatchdog;
        void petHWWatchdog_handler() {
            pet();
        }

    public:
        HWWatchdog(int timeoutSeconds_, int petWatchdogSeconds_) : petHWWatchdog(petWatchdogSeconds_*1000, &HWWatchdog::petHWWatchdog_handler, *this) {
            timeoutSeconds = timeoutSeconds_;
            petWatchdogSeconds = petWatchdogSeconds_;
        }
        
        bool rtcInitialized = false;
        
        bool __tmp_test1Failed = false;
        bool __tmp_test2Failed = false;
        
        bool initialize() {
            // set up HW RTC watchdog
            // note: RTC seems to reset early (randomly?) if this is called after waiting to connect to the particle cloud
            rtcInitialized = rtcModule.begin();
            if (rtcInitialized) {
                // see: https://build.particle.io/libs/RTClibraryDS3231_DL/1.14.2/tab/example/DS3231_alarm.ino
                // we don't need the 32K Pin, so disable it
                rtcModule.disable32K(); 
                // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
                // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
                rtcModule.clearAlarm(1);
                rtcModule.clearAlarm(2);
                // stop oscillating signals at SQW Pin
                // otherwise setAlarm1 will fail
                rtcModule.writeSqwPinMode(DS3231_OFF);
                // turn off alarm 2 (in case it isn't off already)
                // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
                // rtcModule.disableAlarm(1); // note: calling this seems to cause an immediate alarm trigger later on when calling setAlarm(1)
                rtcModule.disableAlarm(1);
                rtcModule.disableAlarm(2);
        
                rtcInitialized &= enable(); // also starts pet timer
            }
            
            return rtcInitialized;
        }
        
        bool startPetTimer() {
            return petHWWatchdog.reset();
        }
        
        void stopPetTimer() {
            petHWWatchdog.stop();
        }
        
        void disable() {
            // note: have not tested best wayt to safely re-enable from here
            //rtcModule.clearAlarm(1);
            WITH_LOCK(Wire) {
                rtcModule.disableAlarm(1); 
            }
            stopPetTimer();
        }
        
        // note: this may be off by up to 1000ms depending on RTC tick
        bool setRTCAlarm(int seconds=-1) {
            if (seconds == -1) {
                seconds = timeoutSeconds;
            }
            
            //return rtcModule.setAlarm1(rtcModule.now() + TimeSpan(seconds), DS3231_A1_Second);
            bool result;
            WITH_LOCK(Wire) {
                result = rtcModule.setAlarm1(rtcModule.now() + TimeSpan(seconds), DS3231_A1_Second);
            }
            return result;
        }
        
        bool enable() {
            rtcModule.clearAlarm(1); // this seems critical to prevent RTC from immediately resetting after: disabling, waiting past original alarm, then enabling
            rtcInitialized &= setRTCAlarm() && startPetTimer();
            return rtcInitialized;
        }
        
        void pet() {
            // "pet" the watchdog by resetting the alarm
            
            // update: use locking since I2C is not thread safe (including software timers). see https://docs.particle.io/reference/device-os/api/wire-i2c/lock/
            // note: Never use lock() or WITH_LOCK() within a SINGLE_THREADED_BLOCK() as deadlock can occur.
            
            // updated version w/ retries
            if (!setRTCAlarm()) {
                delay(1);
                if (!setRTCAlarm()) {
                    delay(25);
                    if (!setRTCAlarm()) {
                        // new: reset the MCU to be safe, with custom reset code (TODO: centralize)
                        System.reset(RESET_REASON_HW_WATCHDOG_PET_FAILED);
                        //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *ERROR* Unable to reset HW RTC watchdog alarm.\" }", PRIVATE);    
                    }
                }
            }
            
            // old version
            /*
            bool rtcAlarmSet;
            //WITH_LOCK(Wire) {
                rtcAlarmSet = setRTCAlarm();
            //}
            
            if (rtcAlarmSet) { //setRTCAlarm()) {
                //beepBuzzer(1, 3); // void beepBuzzer(int count, int optOnDurationMs=35, int optOffDurationMs=50, int optFrequency=1048) {
            } else {
                // new: reset the MCU to be safe, with custom reset code (TODO: centralize)
                System.reset(RESET_REASON_HW_WATCHDOG_PET_FAILED);
                //Particle.publish("push-notification", "{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] *ERROR* Unable to reset HW RTC watchdog alarm.\" }", PRIVATE);    
            }
            */
        }
        
        // note: enable() must be called manually to reactivate RTC and set "petting" timer after this
        // note: this function may call delay() for up to 2 seconds; user may need to pet/reset/otherwise modify the SW watchdog as well
        bool setOneOffWatchdogAlarmThenDisable__see_notes(unsigned int alarmDelayMs) {//, ApplicationWatchdog *wd=NULL) { 
            // goal: before starting "fresh" where a 1 second RTC delay is actually 1 second in reality, we first delay until the remaining whole number of seconds matches our target
            // e.g. if the target delay is 1100ms, we'd set the RTC for 2000ms and delay 900ms before starting its timer

            // wait for the start of a new second (disabling alarm to avoid an unintended reset)
pet(); // temp hack until confirm that disable() works
            disable();
            int startSeconds;
            WITH_LOCK(Wire) {
                startSeconds = rtcModule.now().second();
            }
            int second;
            do {
                WITH_LOCK(Wire) {
                    second = rtcModule.now().second();
                }
                delay(1);
            } while (rtcModule.now().second() == startSeconds);
            /*
            while (rtcModule.now().second() == startSeconds) {
                ///if (wd != NULL) {
                ///    wd->checkin();
                ///}
            };
            */
        
            int remainderMs = alarmDelayMs % 1000; // example: 1100ms -> remainderMs of 100
            // set the alarm and return its success/failure 
            int wholeSeconds = (1000 + (alarmDelayMs-remainderMs))/1000; // example: delay of 1000 + (1100-100) = 2000. 
            bool result = setRTCAlarm(wholeSeconds); //(rtcModule.setAlarm1(rtcModule.now() + TimeSpan(wholeSeconds), DS3231_A1_Second));
            // use delay() to wait any fractional seconds
            delay(1000 - remainderMs - 3); // delay of 900ms, so that the remaining RTC period is 2000-900 = 1100ms (or original target). note that it takes ~3ms to set the RTC alarm
            return result;
        }

        /*
        enum WatchdogTypes { // note: ensure values do not conflict with other "reset reason" codes, e.g. 101 for setup() reset if no wifi connection
            GENERIC = 200,
            TOPOFF_PUMP_RAMP = 201,
            DOSING_PUMP_1 = 202
        };
        
        volatile WatchdogTypes currentWatchdogType = HWWatchdog::WatchdogTypes::GENERIC;
        */
};