#include "application.h"
#include "SharedUtilities.h"


/* Ensures that the current device ID ends with the specified string. If not, an error message is published to the console,
   the onboard RBG light blinks red, and the method does not return (to prevent downstream code execution). */
void SharedUtilities::checkDeviceId(String deviceIdEnding) {
    
    if (!System.deviceID().endsWith(deviceIdEnding)) {

        Particle.publish("Error", "Error: Unexpected device ID!");
        
        RGB.brightness(100);
        RGB.control(true);
        
        while (1) {
            RGB.color(255, 0, 0);
            delay(500);
            RGB.color(0, 0, 0);
            delay(500);
        }

    }
    
};
