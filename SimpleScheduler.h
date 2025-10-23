// TODO: decide whether to expand so that we have more than 60 seconds to match the hour and minute (e.g. if another process delays loop() for too long)
// TODO: note that hourly intervals that 24 is not divisuble by will result in resetting at the start hour

//#define SIMPLE_SCHEDULER_DEBUG // uncomment to enable debugging

class SimpleScheduler {

    public:
        enum DayOfWeek {
            ANY = 0, // run based on repeatIntervalHours only
            // run on days below only (use with 24-hour repeat period)
            SUNDAY = 1, // https://docs.particle.io/reference/device-os/api/time/weekday/
            MONDAY = 2,
            TUESDAY = 3,
            WEDNESDAY = 4,
            THURSDAY = 5,
            FRIDAY = 6,
            SATURDAY = 7
        };
        
    private:
        const char *nameStr; // inspired by https://github.com/particle-iot/device-os/blob/fdc9b99394d1f9d6ad27e7c7facfdca0525aa8fa/wiring/inc/spark_wiring_print.h
        int startHour;
        int startMinute; // tasks will be run between 0-5 minutes after the start hour/minute
        int repeatIntervalHours;
        DayOfWeek dayOfWeek;
        int runtimeMs;
        typedef std::function<void(void)> scheduler_callback_fn;
        scheduler_callback_fn callback; // inspiration: https://github.com/particle-iot/device-os/blob/fdc9b99394d1f9d6ad27e7c7facfdca0525aa8fa/wiring/inc/spark_wiring_timer.h
        int lastRunEEPROMID; 
        const int LAST_RUN_UNDEFINED = -1; // used by getLastExecutedEpoch(). must always be negative to avoid thinking we haevn't executed a task when we actually just did
        long hourMinuteLastMatchedEpoch = -1;
        bool invalidRepeatInterval = false;
        long loopLastCalledEpoch = -1;
        bool enabled = true;
        
        int getEEPROMAddrByID(int id) {
            // note: we're starting from address 1024 and giving each ID 16 bytes for leeway. leaves lots of room for scheduled tasks
            // note: "The Gen 3 (Argon, Boron, B Series SoM, Tracker SoM) devices have 4096 bytes of emulated EEPROM."
            return 1024 + id*16;
        }
        
        void delayBasedOnActivity(int numPublishes, bool taskWasExecuted=false) {
            if (numPublishes == 0) {
                return;
            } else {
                if (taskWasExecuted) {
                    delay((numPublishes + 2) * 1000);
                } else {
                    delay(numPublishes * 1000);
                }
            }
        }

    public:
    
        // NOTE: repeat intervals over 24 hours must be in increments of 24 hours
        SimpleScheduler(const char *nameStr_, int startHour_, int startMinute_, int repeatIntervalHours_, DayOfWeek dayOfWeek_, scheduler_callback_fn callback_, int EEPROMID_, bool optEnabled_=true) {
            nameStr = nameStr_;
            startHour = startHour_;
            startMinute = startMinute_;
            repeatIntervalHours = repeatIntervalHours_;
            dayOfWeek = dayOfWeek_;
            callback = callback_;
            lastRunEEPROMID = EEPROMID_;
            enabled = optEnabled_;
            
            // note: repeat intervals over 24 hours must be in increments of 24 hours; otherwise the notion of a "start hour" no longer makes any sense (e.g. a timer starting at 8am and repeating every 25 hours would behave erratically)
            if (repeatIntervalHours > 24 && (repeatIntervalHours % 24 != 0)) {
                invalidRepeatInterval = true;
            }
            
            // sanity check
            if (dayOfWeek != DayOfWeek::ANY && repeatIntervalHours != 24) {
                invalidRepeatInterval = true;
            }
        }
        
        long getLastRunEEPROM() {
            long lastRunEpoch_EEPROM;
            EEPROM.get(getEEPROMAddrByID(lastRunEEPROMID), lastRunEpoch_EEPROM); // will return 0xFFFFFFFF if no existing value has been stored
            return lastRunEpoch_EEPROM;
        }
        
        void setLastRunEEPROM(long lastRunEpoch) {
            EEPROM.put(getEEPROMAddrByID(lastRunEEPROMID), lastRunEpoch);
        }
        
        void resetLastRunEEPROM() {
            EEPROM.put(getEEPROMAddrByID(lastRunEEPROMID), 0xFFFFFFFF);
        }
        
        /*
         * Assumes that loop() will always get called at least once per minute; otherwise we'll miss the window to call our trigger
         * Returns true if the task was executed
         */
        bool loop(bool delayBasedOnActivity_=false, bool optForceDisabled_=false) {
            long now = Time.now();
            bool taskWasExecuted = false;
            int callsToPublish = 0;
            
            #ifdef SIMPLE_SCHEDULER_DEBUG
                int elapsedSeconds = now - loopLastCalledEpoch;
                if (loopLastCalledEpoch != -1 && elapsedSeconds >= 60) {
                    Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] debug warning: Scheduler %s loop() called >= 60 seconds ago. elapsed time: %d seconds""\" }", nameStr, elapsedSeconds), PRIVATE);                
                }
            #endif
            loopLastCalledEpoch = now;


            if (Time.isValid() && Time.minute() == startMinute) { // note: not checking hour here
                
                /*
                // new: see if we're between 0-5 minutes from our target time (5 min gives us leeway in case some long-running process blocks loop() for up to 5 minutes)
                int minMinute = startMinute;
                int maxMinute = (startMinute + 5) % 60
                if (maxMinute % 60 >= 0) {
                    maxHour = minHour
                }
                //bool isWithinTargetMinutes = (Time.minute() == startMinute)
                */
                
                // only check for match once per minute (vs every call to loop() for a minute)
                if (hourMinuteLastMatchedEpoch == -1 || now - hourMinuteLastMatchedEpoch >= 60) {
                    hourMinuteLastMatchedEpoch = now;
                    
                    if (invalidRepeatInterval) {
                        Particle.publish("ERROR", String::format("SimpleScheduler (%s): Timer contains an invalid repeat interval. Hours must be <= 24, or otherwise in increments of 24 hours. Hours must also match up with any day-of-week repeat schedule.", nameStr));
                        callsToPublish++;
                        if (delayBasedOnActivity_) {
                            delayBasedOnActivity(callsToPublish);
                        }
                        return false; //taskWasExecuted;
                    }
                    
                    if (dayOfWeek != DayOfWeek::ANY) {
                        if (Time.weekday() == dayOfWeek) {
                            #ifdef SIMPLE_SCHEDULER_DEBUG
                                Particle.publish("debug", String::format("SimpleScheduler (%s): Day of week match", nameStr));
                                callsToPublish++;
                            #endif
                        } else {
                            #ifdef SIMPLE_SCHEDULER_DEBUG
                                Particle.publish("debug", String::format("SimpleScheduler (%s): Minutes and hour match, but day of week does not match: %d, %d", nameStr, Time.day(), dayOfWeek));
                                callsToPublish++;
                            #endif
                            // if this isn't the right day, return without running the task
                            return false;
                        }
                    }
                    
                    // see if current hour is a valid execution hour given our repeat interval
                    // note that the calcualtion below only works for repeat intervals of <= 24 hours; otherwise it rolls over (so we check that hoursSinceLastRun_EEPROM >= repeatIntervalHours below)
                    bool isValidExecutionHour_24HOnly = ((Time.hour() - startHour) % repeatIntervalHours == 0);
                    if (isValidExecutionHour_24HOnly) {
                        long lastRunEpoch_EEPROM = getLastRunEEPROM(); // will return 0xFFFFFFFF if no existing value has been stored
                        float hoursSinceLastRun_EEPROM = (lastRunEpoch_EEPROM == 0xFFFFFFFF) ? LAST_RUN_UNDEFINED : (now - lastRunEpoch_EEPROM)/60.0/60.0;
                        
                        #ifdef SIMPLE_SCHEDULER_DEBUG
                            Particle.publish("debug", String::format("SimpleScheduler (%s): Hour/minute match. lastRunEpoch_EEPROM: %ld, hoursSinceLastRun_EEPROM: %f", nameStr, lastRunEpoch_EEPROM, hoursSinceLastRun_EEPROM));
                            callsToPublish++;
                        #endif
                    
                        const float ONE_MINUTE_AS_HOURS = 1/60.0; // bug fix: we need to allow for up to 60 sec of flex since tasks can be called w/in a 60 second window
                        if (hoursSinceLastRun_EEPROM == LAST_RUN_UNDEFINED || hoursSinceLastRun_EEPROM >= (repeatIntervalHours - ONE_MINUTE_AS_HOURS)) {
                            if (!enabled || optForceDisabled_) {
                                Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Scheduler skipping '%s'. Task is%s disabled.""\" }", nameStr, optForceDisabled_ ? " temporarily" : ""), PRIVATE);
                                callsToPublish++;
                                return false;
                            }

                            EEPROM.put(getEEPROMAddrByID(lastRunEEPROMID), now);
                            
                            #ifdef SIMPLE_SCHEDULER_DEBUG
                                Particle.publish("push-notification", String::format("{ \"type\": \"send-message-generic\", \"message\": \"[Aquarium controller] Scheduler running '%s' (debug: EEPROM addr %d, hoursSinceLastRun_EEPROM: %f""\" }", nameStr, lastRunEEPROMID, hoursSinceLastRun_EEPROM), PRIVATE);
                                callsToPublish++;
                            #endif
                            
                            if (callback) {
                                callback();
                                taskWasExecuted = true;
                            }
                        } else {
                            #ifdef SIMPLE_SCHEDULER_DEBUG
                                Particle.publish("debug", String::format("SimpleScheduler (%s): Minutes and hour interval match, but EEPROM value too recent. hoursSinceLastRun_EEPROM: %f, hourMinuteLastMatchedEpoch: %ld, now: %ld", nameStr, hoursSinceLastRun_EEPROM, hourMinuteLastMatchedEpoch, now));
                                callsToPublish++;
                            #endif
                        }
                    } else {
                        #ifdef SIMPLE_SCHEDULER_DEBUG
                            Particle.publish("debug", String::format("SimpleScheduler (%s): Minutes match, but hour does not match repeat interval", nameStr));
                            callsToPublish++;
                        #endif
                    }
                } else {
                    #ifdef SIMPLE_SCHEDULER_DEBUG
                        // commenting out now that loop() is getting called very frequently
                        //Particle.publish("debug", String::format("SimpleScheduler: Minutes matched less than 1 minute ago. name: %s, hourMinuteLastMatchedEpoch: %ld, now: %ld", nameStr, hourMinuteLastMatchedEpoch, now));
                    #endif
                }
            }
            if (delayBasedOnActivity_) {
                delayBasedOnActivity(callsToPublish, taskWasExecuted);
            }
            return taskWasExecuted;
        }
};