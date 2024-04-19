#include "Arduino.h"
#include "Pid.hpp"
#include "Sensor.hpp"
#include "Motor.hpp"
#include "Encoders.hpp"

// Sensor setup
// Sensor pin names reflect the number on the pcb
enum SensorPin { s1 = 15, s2, s3, s4, s5, s6, s7, s8, s9 };
uint8_t sensorPins[] = {s1, s2, s3, s4, s5, s6, s7, s8, s9 };
constexpr int numSensorPins = 9;
Sensor sensor(sensorPins, numSensorPins);

Encoders encoders(8, 9, 11, 10);

// Pid setup
double kp = 0.85;
double ki = 0.02;
double kd = 0.04;
double setpoint = 4000;
PIDController pid(kp, ki, kd,  setpoint);

uint64_t elapsedTime = 0;

// Motor setup
// A is left motor, B is right motor
constexpr int MA1 = 3, MA2 = 4, MB1 = 6, MB2 = 7, PWMA = 2, PWMB = 5;
/*constexpr int MA1 = 3,  MA2 = 4,
        MB1 = 6, MB2 = 7,
        PWMA = 2, PWMB = 5;*/

Motor motor(MA1, MA2, MB1, MB2, PWMA, PWMB, encoders);

void setup()
{
    // Sensor setup
    Serial.begin(9600);
    motor.autoCalibrate(sensor, 300);
}

void loop()
{
    encoders.update();
    if (encoders.getRelativeEncoderDiff())
    {
        Serial.println(encoders.getTotalEncoderDiff());
    }
    uint16_t pos = sensor.readLine();
    //Serial.print("Pos: ");
    //Serial.println(pos);

    double dt = double(millis() - elapsedTime) / 1000;
    elapsedTime = millis();

    // Compute PID output
    double pidOutput = pid.compute(pos, dt);
    //Serial.print("PID: ");
    //Serial.println(pidOutput);

    /*auto v = sensor.getSensorValues();
    constexpr int mid = numSensorPins / 2;
    constexpr int threshold = 1000 - 200;
    int right = 0;
    int left = 0;
    for (int i = 0; i < numSensorPins; i++)
    {
        if (i < mid && v[i] > threshold)
        {
            right++;
        }
        else if (i > mid && v[i] > threshold)
        {
            left++;
        }
    }
    constexpr int minimumBlack = 4;
    if (right >= minimumBlack && left < minimumBlack)
    {
        motor.powerTurn(-75);
    }
    else if (left >= minimumBlack && right < minimumBlack)
    {
        motor.powerTurn(75);
    }*/
    motor.updateOutput((long)pidOutput, -2000, 2000);
}

