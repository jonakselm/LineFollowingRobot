#include "Arduino.h"
#include "Pid.hpp"
#include "Sensor.hpp"
#include "Motor.hpp"

// Sensor setup
// Sensor pin names reflect the number on the sensor chip
enum SensorPin { s9 = A0, s11, s13, s15, s17 };
uint8_t sensorPins[] = { s9, s11, s13, s15, s17 };
Sensor sensor(sensorPins, 5);

// Pid setup
double kp = 0.75;
double ki = 0.0;
double kd = 0.01;
double setpoint = 2000;
PIDController pid(kp, ki, kd,  setpoint);

uint64_t elapsedTime = 0;

// Motor setup
// A is left motor, B is right motor
constexpr int MA1 = 10,  MA2 = 11,
        MB1 = 9, MB2 = 8,
        PWMA = 5, PWMB = 6;

Motor motor(MA2, MA1, MB1, MB2, PWMA, PWMB);

void setup()
{
    // Sensor setup
    Serial.begin(9600);
    motor.autoCalibrate(sensor, 300);
}

void loop()
{
    uint16_t pos = sensor.readLine();
    Serial.print("Pos: ");
    Serial.println(pos);
    // 0 == høyre, 4000 == venstre
    Serial.println(pos);


    double dt = double(millis() - elapsedTime) / 1000;
    elapsedTime = millis();

    // Compute PID output
    double pidOutput = pid.compute(pos, dt);
    Serial.print("PID: ");
    Serial.println(pidOutput);

    motor.updateOutput((long)pidOutput, -2000, 2000);
}

