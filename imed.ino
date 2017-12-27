// include the library code:
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <PID_v1.h>

// These #defines make it easy to set the backlight color
#define WHITE 0x7

// define input and output pins
// Arduino Mega disponemos de 15 salidas PWM de 8bis en los pines 2 a 13 y 44 a 46
#define SENSOR_INPUT_PIN 14
#define PWM_OUTPUT 45

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=3, Ki=5, Kd=4;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, REVERSE);

// initial temperature
int temp = 30;

void setup() {
    // open the serial port at 9600 bps:
    Serial.begin(9600);
  
    // set up the LCD's number of columns and rows: 
    lcd.begin(16, 2);
    lcd.setBacklight(WHITE);

    // configure PWM_OUTPUT pin
    pinMode(PWM_OUTPUT, OUTPUT);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop() {
    uint8_t buttons;
    int sensor_measure;
    float measure_ajust, current_temp;
    float slope = -0.0199916;
    float origin_value = 1.0358194;

    // PID variable linked
    Setpoint = origin_value+slope*temp;

    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 0);
    // print temp that we hope
    lcd.print("Req. temp: " + String(temp));

    // print current temperature readed by the sensor
    sensor_measure = analogRead(SENSOR_INPUT_PIN); // return int(0 to 1023)
    measure_ajust = (sensor_measure*4.9999)/1023; // our temp is between 0-5V --> 5V = -200 degrees   0V = 50 degrees
    current_temp = (measure_ajust-origin_value)/(slope);

    // round to decimal precision
    current_temp = round(current_temp*10);
    current_temp /= 10;

    lcd.setCursor(0, 1);
    lcd.print ("Temp: " + String(current_temp));

    // change PWM signal to achive our request temperature 
    // PWM signal is between 0-255
    Input = measure_ajust;
    myPID.Compute();

    if ((measure_ajust-origin_value)/(slope) < temp)
        analogWrite(PWM_OUTPUT, int(Output));
    else
        analogWrite(PWM_OUTPUT, 0);

    // read buttons' signals
    buttons = lcd.readButtons();

    if (buttons) {
        lcd.clear();
        if (buttons & BUTTON_RIGHT) {
            if (temp < 50)
                temp ++;
        }
        if (buttons & BUTTON_LEFT) {
            if (temp > -200)
                temp --;
        }
    }

  delay(200);
}

