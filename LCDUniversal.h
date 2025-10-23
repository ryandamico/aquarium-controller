#pragma once

#include "Waveshare_LCD1602_RGB.h"
#include "LiquidCrystal_I2C_Spark.h"

#define LCD_RGB_NORMAL 16, 16, 16 // TODO: dedupe with main
#define LCD_RGB_WARNING 32, 16, 16 // TODO: dedupe with main

class LCDUniversal {
    private:
        Waveshare_LCD1602_RGB lcd;//(16,2);
        LiquidCrystal_I2C *lcd2;
        
    public:
        LCDUniversal() : lcd(16,2) {
            lcd2 = new LiquidCrystal_I2C(0x27, 16, 2);
        }
        
        void display(const char *line1, const char *line2, int delaySec=3 /* set to zero to leave on screen indefinitely */, bool warningMode=false) {
            /*
        	for(uint8_t i = 0; str[i] != '\0';i++)
        		write_char(str[i]);
    		*/
            clear();
            if (warningMode) {
                setRGB(LCD_RGB_WARNING);
            }
            setCursor(0,0); 
            send_string(String(line1).c_str());
            
            setCursor(0,1); 
            send_string(String(line2).c_str());
            
            if (delaySec > 0) {
                delay(delaySec*1000);
                clear();
                if (warningMode) {
                    setRGB(LCD_RGB_NORMAL);
                }
            }
        }
        
        void init() {
            WITH_LOCK(Wire) {
                lcd.init();
                lcd2->init();
                lcd2->backlight();
            }
        }
    
        void clear() {
            WITH_LOCK(Wire) {
                lcd.clear();
            }
            WITH_LOCK(Wire) {
                lcd2->clear();
            }
        }
        
        void setCursor(uint8_t col, uint8_t row) {
            WITH_LOCK(Wire) {
                lcd.setCursor(col, row);
            }
            WITH_LOCK(Wire) {
                lcd2->setCursor(col, row);
            }
        }
    
        void write(uint8_t value) {
            WITH_LOCK(Wire) {
                lcd.write_char(value);
            }
            WITH_LOCK(Wire) {
                lcd2->write(value);
            }
        }
    
        void setRGB(uint8_t r, uint8_t g, uint8_t b) {
            WITH_LOCK(Wire) {
                lcd.setRGB(r, g, b);
                if (r + g + b == 0) {
                    lcd2->noBacklight();
                } else {
                    lcd2->backlight();
                }
            }
        }
        
        void send_string(const char *str, uint8_t wrapCharIndex) {
            WITH_LOCK(Wire) {
    	        for (uint8_t i = 0; str[i] != '\0'; i++) {
    	            if ((wrapCharIndex && i == wrapCharIndex) || (str[i] == '\n')) { //if (wrapCharIndex && i == wrapCharIndex) {
    	                setCursor(0, 1);
    	                if (str[i] == '\n') { // note: this doesn't actually work well at all 
    	                    continue;
    	                }
    	            }
        		    lcd.write_char(str[i]);
        		    lcd2->write(str[i]);
    	        }
            }
        }
        
        void send_string(const char *str) {
            WITH_LOCK(Wire) {
                lcd.send_string(str);
            }
            WITH_LOCK(Wire) {
                lcd2->print(str);
            }
        }
        
};