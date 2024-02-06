#include <Arduino.h>
#include <Motor.hpp>

constexpr int MA1 = 10,  MA2 = 11,
                MB1 = 9, MB2 = 8,
                PWMA = 5, PWMB = 6;

// A is left, B is right
Motor motor(MA2, MA1, MB1, MB2, PWMA, PWMB);

void setup()
{
    Serial.begin(9600);
    motor.stop();
}

void loop()
{
}

