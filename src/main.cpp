#include <Arduino.h>

volatile int currTemp = 20;
volatile int setTemp = 300;
volatile int uVal = 0;

#include "Adafruit_MAX31855.h"


//#define DEBUG 1
#define USEDISPLAY 1

#ifdef USEDISPLAY
#include "utility/Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#endif

//thermocouple connections
#define tcSCK 5
#define tcCS 4
#define tcMISO 3


#define pwmPin 6        //controls the pwm on the mosfet
#define potPin A3       //potentiometer input temp
#define ledPin 13       //HIGH when the PID control is active

#define MINTEMP 200     //the minimum temperature that can be set
#define MAXTEMP 500     //the max temp



#include "lowPassFilter.h"
#include "pidController.h"

#ifdef USEDISPLAY
Adafruit_SSD1306 display(4);
#endif

PIDController pid;
LowPassFilter lpf;

Adafruit_MAX31855 thermocouple(tcSCK, tcCS, tcMISO);

unsigned long lastUpdate1 = 0, lastUpdate2 = 0, lastUpdateDisp = 0;

#include "timerCode.h"      //code for setting up the timer, and defining the ISR

void setup()
{

#ifdef DEBUG                //if we are not debugging no need for Serial
    Serial.begin(115200);       
#endif 

    thermocouple.begin();       //initialize the thermocouple

#ifdef USEDISPLAY
    display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);       //display iniy
    display.clearDisplay();
    display.setTextColor(WHITE);                        //black background, LED on only on the text
    display.display();                          //display the settings
#endif

    pinMode(pwmPin, OUTPUT);
    pinMode(potPin, INPUT);
    pinMode(ledPin, OUTPUT);
    
    digitalWrite(ledPin, LOW);              //default value, PID not active
    TCCR0B = TCCR0B & B11111000 | B00000101;        //set the pwm frequency to ~64Hz, reduces hum, but messes up the delay()

    setupTimer();           //from timerCode.h, sets up the timer 
    
    lastUpdate1 = micros();         //for the serial communication
    lastUpdate2 = micros() + 500000;    //we send data in 2 steps
    lastUpdateDisp = micros();          //for the display
}

void updateDisplay();       //function to update the display


int tempDiff = 0;           //only used in loop(), but declared globally to avoid declaration overhead

#define MODE_MAX    1       //constants for the control modes
#define MODE_ZERO    2
#define MODE_PID    3

int mode = MODE_MAX;


void loop()
{
    // put your main code here, to run repeatedly:
    currTemp = (int)thermocouple.readCelsius();
    if(currTemp == 0)
        return;         //false reading, ignore it, probably interference
        
    setTemp = (int)map(analogRead(potPin), 0, 1023, MINTEMP, MAXTEMP);      //read the temperature from the potentiometer
    
    tempDiff = setTemp - currTemp;          //calculate it only once, to avoid overhead, e in the control system

    if(tempDiff > 50) {
        //far from setpoint, turn on max
        analogWrite(pwmPin, 255);       //max width
        digitalWrite(ledPin, LOW);      //PID not active
        mode = MODE_MAX;
        if(tempDiff > 100) {
            //really far from setpoint, reset the controller
            pid.reset();
        }
    }
    else if(setTemp - currTemp < -30) {
        //overshoot, turn off
        analogWrite(pwmPin, 0);
        digitalWrite(ledPin, LOW);
        if(tempDiff < -80) {
            //really big overshoot, or setTemp reduce by user, reset the controller
            pid.reset();
        }
        mode = MODE_ZERO;
    }
    else {
        if(abs(pid.prop) < 70 && pid.integ < -50) {
            //some bug, after turning on the controller, when the iron is already hot
            //caused by the non-instantaneous change through the low pass filter
            pid.reset();
        }

        mode = MODE_PID;
        digitalWrite(ledPin, HIGH);     //indicates that the PID control is active
        analogWrite(pwmPin, map((int)uVal, 0, 325, 0, 255));        //calculate the PW
    }
    
    #ifdef DEBUG
    if(micros() - lastUpdate1 > 100000) {       //the first chunk of data
        Serial.println(setTemp);
        Serial.println(currTemp);
        lastUpdate1 = micros();
    }  
    if(micros() - lastUpdate2 > 100000) {       //the second part
        lastUpdate2 = micros();
        Serial.println(pid.prop);
        Serial.println(pid.integ);
        Serial.println();
    } 
    #endif

    #ifdef USEDISPLAY
    if((micros() - lastUpdateDisp > 100000)) {      //display update rate
        updateDisplay();
        lastUpdateDisp = micros();
    }
    #endif
}

#ifdef USEDISPLAY
void updateDisplay() {
    display.clearDisplay();
    display.setCursor(0,1);
    display.setTextSize(2);

    switch(mode) {
        case MODE_MAX:  display.println("MAX 325V");
                        break;
        case MODE_ZERO: display.println("OFF 0V");
                        break;
        case MODE_PID:  display.print("PID ");
                        display.print(uVal);
                        display.println("V");
                        break;
    }

    display.setTextSize(1);
    display.println();      //smaller gap between the lines

    display.setTextSize(2);
    display.print("S: ");
    display.println(setTemp);       //display the set temperature


    display.setTextSize(1);
    display.println();          //smaller gap between the lines


    display.setTextSize(2);
    display.print("R: ");
    display.println(currTemp);      //display the current temperature

    display.display();              //update the display
}
#endif