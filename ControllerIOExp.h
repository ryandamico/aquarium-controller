/*class ControllerIOExp : public IOExpansionBoard {

    #define ACTIVE_LOW LOW
    #define INACTIVE_HIGH HIGH

    private:
        void setPinModes() { // implement virtual base class
            mcp.pinMode(ControllerIOExp::Components::TOPOFF_SOLENOID, OUTPUT);
            mcp.digitalWrite(ControllerIOExp::Components::TOPOFF_SOLENOID, LOW);
            mcp.pinMode(ControllerIOExp::Components::DRAIN_BALL_VALVE, OUTPUT);
            mcp.digitalWrite(ControllerIOExp::Components::DRAIN_BALL_VALVE, LOW);
            mcp.pinMode(ControllerIOExp::Components::CANNISTER_PUMP_RELAY, INPUT); // high impedience enables the pump; outputting a digital LOW disables it
            mcp.pinMode(ControllerIOExp::Components::C02_SOLENOID, OUTPUT); // TODO: should this happen both here and in the CO2 bubble controller?
            mcp.digitalWrite(ControllerIOExp::Components::C02_SOLENOID, LOW);
            mcp.pinMode(ControllerIOExp::Components::UNUSED_OUTPUT, OUTPUT); // currently unused
            mcp.digitalWrite(ControllerIOExp::Components::UNUSED_OUTPUT, LOW);
            mcp.pinMode(ControllerIOExp::Components::LED_1, OUTPUT); 
            mcp.digitalWrite(ControllerIOExp::Components::LED_1, HIGH);
            mcp.pinMode(ControllerIOExp::Components::LED_2, OUTPUT); 
            mcp.digitalWrite(ControllerIOExp::Components::LED_2, LOW);
            
            // inputs
            mcp.pinMode(ControllerIOExp::Components::PUSHBUTTON_2, INPUT);
            mcp.pullUp(ControllerIOExp::Components::PUSHBUTTON_2, HIGH);
        }
    
    public:
        typedef enum Components { 
            TOPOFF_SOLENOID = 9, //B1
            DRAIN_BALL_VALVE = 8, //B0
            CANNISTER_PUMP_RELAY = 15, //B7
            C02_SOLENOID = 10, //B2
            PUSHBUTTON_2 = 2, //A2
            LED_1 = 3, //A3
            LED_2 = 5, //A5
            LED_3 = 7, //A7
            UNUSED_OUTPUT = 11 //B3 -- unused two-pin output header
        };
        
};*/