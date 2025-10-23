// interrupt class reference: https://docs.particle.io/cards/firmware/interrupts/attachinterrupt/

class CO2BubbleSensor {
     // modified from https://www.amazon.com/gp/customer-reviews/RXFZUXSSWB9MY/ref=cm_cr_dp_d_rvw_ttl?ie=UTF8&ASIN=B07MY7K249
 
    private:
        Timer timer1;// = new Timer(1000, &CO2BubbleSensor::timerHandler);
        int _bubbleSensorPin;
    
    
    public:
        //int bubbleSensorPin = PIN__CO2_SENSOR_IN; // flow sensor attached to pin 6 (pin must also be an interrupt ("int") pin)
        //float flowRate=0.0;
        //volatile int flowPulseCount=0;
        //unsigned long oldTime=0;
        
        
        
        CO2BubbleSensor(int bubbleSensorPin) : timer1(1000, &CO2BubbleSensor::timerHandler, *this, false) { // https://community.particle.io/t/using-timer-in-a-class/17464/12
// TODO: set one-shot to true?            
            _bubbleSensorPin = bubbleSensorPin;
        }
        
        void init() {
            pinMode(_bubbleSensorPin, INPUT);//???????_PULLDOWN); // declare flow sensor pin
            //digitalWrite(_bubbleSensorPin, HIGH); // set flow sensor pin
            //attachInterrupt(digitalPinToInterrupt(_bubbleSensorPin), CO2BubbleSensor::read_flow_rate/*bubblePulseCounter*/, FALLING); // Configured to trigger on a FALLING state change (transition from HIGH state to LOW state)
            attachInterrupt(_bubbleSensorPin, &CO2BubbleSensor::bubblePulseCounter, this, CHANGE);//FALLING);
            
            // testing only
            //timer1(1000, &CO2BubbleSensor::timerHandler, this);
            timer1.start();
        }
        
        void timerHandler() {
            digitalWrite(LED, !digitalRead(LED));
        }

        void bubblePulseCounter() {
            //flowPulseCount++;
            digitalWrite(LED, digitalRead(_bubbleSensorPin));
        }
        
        /*float readFlowRateGPH(){    
            float flowRateGPH = -1;
            if ((millis() - oldTime) > 900) { // Make sure a reasonable time has passed to count pulses - ideally at least 1sec
                //detachInterrupt(digitalPinToInterrupt(_bubbleSensorPin)); //Disable the interrupt while calculating flow rate
                detachInterrupt(_bubbleSensorPin); //Disable the interrupt while calculating flow rate
                flowRateGPH = ((1000.0 / (millis() - oldTime)) * flowPulseCount) / FLOW_CALIBRATION_FACTOR*15.8503;
                //Scale to frequency (pulses per second) and convert to flow rate 1 LPM = 15.8503 gph
                oldTime = millis(); //Reset the reading clock
                flowPulseCount = 0; // Reset the pulse counter so we can start incrementing again
                //attachInterrupt(digitalPinToInterrupt(_bubbleSensorPin), bubblePulseCounter, FALLING); // Enable interrupt again
                attachInterrupt(_bubbleSensorPin, &CO2BubbleSensor::bubblePulseCounter, this, FALLING); // Enable interrupt again
            }
            return(flowRateGPH); // Return the flow rate for this reading
        }    */
        
};