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
    //motor.powerTurn(90);
    motor.autoCalibrate(sensor, 300);
    /*digitalWrite(MA2, 0);
    digitalWrite(MA1, 1);
    digitalWrite(MB2, 0);
    digitalWrite(MB1, 1);

    digitalWrite(PWMA, 1);
    digitalWrite(PWMB, 1);*/
    /*digitalWrite(MA1, 1);
    digitalWrite(MA2, 0);
    digitalWrite(MB1, 0);
    digitalWrite(MB2, 1);*/
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

void loop()
{
    /*encoders.update();
    if (encoders.getRelativeEncoderDiff())
    {
        Serial.println(encoders.getTotalEncoderDiff());
    }
    /*uint16_t pos = sensor.readLine();
    //Serial.print("Pos: ");
    //Serial.println(pos);

    double dt = double(millis() - elapsedTime) / 1000;
    elapsedTime = millis();

    // Compute PID output
    double pidOutput = pid.compute(pos, dt);
    //Serial.print("PID: ");
    //Serial.println(pidOutput);

    motor.updateOutput((long)pidOutput, -2000, 2000);*/
}

