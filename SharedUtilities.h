#include "application.h"

class SharedUtilities
{
    public:
        /* Ensures that the current device ID ends with the specified string. If not, an error message is published to the console,
           the onboard RBG light blinks red, and the method does not return (to prevent downstream code execution). */
        static void checkDeviceId(String deviceIdEnding) {
            if (!System.deviceID().endsWith(deviceIdEnding)) {
                //Particle.publish("Error", "Error: Unexpected device ID!"); // update: docs say not to call publish() from startup()
                RGB.brightness(100);
                RGB.control(true);
                for (int i=0; i<5; i++) {
                    RGB.color(255, 0, 0);
                    delay(500);
                    RGB.color(0, 0, 0);
                    delay(500);
                }
                System.enterSafeMode(RESET_NO_WAIT);
            }
        };
        
        static bool isDST() {
            int currentMonth = Time.month();
            int currentDay = Time.day();
            int currentWeekday = Time.weekday();  // Sunday is 1, Monday is 2, etc.
        
            if (currentMonth > 3 && currentMonth < 11) {
                return true;
            }
            if (currentMonth == 3) {
                return currentDay - currentWeekday > 7;
            }
            if (currentMonth == 11) {
                return currentDay < currentWeekday;
            }
            return false;
        }
        
        static void checkAndAdjustDST() {
            if (!Time.isValid()) {
                Particle.publish("debug", "Warning: Time is not valid; not checking/changing DST status"); // note: we're technically not supposed to call publish() from Timer handlers, but it seems to work fine
                delay(1000);
                return;
            }
            //static bool currentDSTStatus = !isDST();  // Initialize to the opposite of current status for first-time check
            bool newDSTStatus = isDST();

            if (newDSTStatus != Time.isDST()) {
                Particle.publish("debug", "Changing DST status to " + String(newDSTStatus)); // note: we're technically not supposed to call publish() from Timer handlers, but it seems to work fine
                if (newDSTStatus) {
                    Time.beginDST();
                } else {
                    Time.endDST();
                }
                //currentDSTStatus = newDSTStatus;
            }
        }
        
        /* Example usage:
        
        Timer dstTimer(3600000, checkAndAdjustDST);  // Check every hour
        
        void setup() {
            // Start the DST check timer
            dstTimer.start();
        }
        
        void loop() {
            // Your main loop code...
        }
        */
        
};