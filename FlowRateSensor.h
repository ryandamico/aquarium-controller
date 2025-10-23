// interrupt class reference: https://docs.particle.io/cards/firmware/interrupts/attachinterrupt/

class FlowRateSensor {
     // modified from https://www.amazon.com/gp/customer-reviews/RXFZUXSSWB9MY/ref=cm_cr_dp_d_rvw_ttl?ie=UTF8&ASIN=B07MY7K249
 
    public:
        int flowSensorPin = PIN__FLOW_SENSOR; // flow sensor attached to pin 6 (pin must also be an interrupt ("int") pin)
        #define FLOW_CALIBRATION_FACTOR 5.5 // Note: F=(0.2*Q)±2% for OP's sensor, Q=L/Min, and F is pulse freq in 1/s. your model is GR-108 (1")
        float flowRate=0.0;
        volatile int flowPulseCount=0;
        unsigned long oldTime=0;
        
        FlowRateSensor() {
        }
        
        void init() {
            pinMode(flowSensorPin, INPUT_PULLDOWN); // declare flow sensor pin
            //digitalWrite(flowSensorPin, HIGH); // set flow sensor pin
            //attachInterrupt(digitalPinToInterrupt(flowSensorPin), FlowRateSensor::read_flow_rate/*flowPulseCounter*/, FALLING); // Configured to trigger on a FALLING state change (transition from HIGH state to LOW state)
            attachInterrupt(flowSensorPin, &FlowRateSensor::flowPulseCounter, this, FALLING);
        }

        void flowPulseCounter() {
            flowPulseCount++;
            //digitalWrite(LED, !digitalRead(LED));
        }
        
        float readFlowRateGPH(){    
float flowRateGPH = -1;
            if ((millis() - oldTime) > 900) { // Make sure a reasonable time has passed to count pulses - ideally at least 1sec
                //detachInterrupt(digitalPinToInterrupt(flowSensorPin)); //Disable the interrupt while calculating flow rate
                detachInterrupt(flowSensorPin); //Disable the interrupt while calculating flow rate
                flowRateGPH = ((1000.0 / (millis() - oldTime)) * flowPulseCount) / FLOW_CALIBRATION_FACTOR*15.8503;
                //Scale to frequency (pulses per second) and convert to flow rate 1 LPM = 15.8503 gph
                oldTime = millis(); //Reset the reading clock
                flowPulseCount = 0; // Reset the pulse counter so we can start incrementing again
                //attachInterrupt(digitalPinToInterrupt(flowSensorPin), flowPulseCounter, FALLING); // Enable interrupt again
                attachInterrupt(flowSensorPin, &FlowRateSensor::flowPulseCounter, this, FALLING); // Enable interrupt again
            }
            return(flowRateGPH); // Return the flow rate for this reading
        }    
        
};