#include "Arduino.h"
#include "Pid.hpp"
#include "Sensor.hpp"
#include "Motor.hpp"
#include "Encoders.hpp"
#include <algorithm>


// Sensor setup
// Sensor pin names reflect the number on the pcb
enum SensorPin { s1 = 15, s2, s3, s4, s5, s6, s7, s8, s9 };
uint8_t sensorPins[] = {s1, s2, s3, s4, s5, s6, s7, s8, s9 };
constexpr int numSensorPins = 9;
Sensor sensor(sensorPins, numSensorPins);


Encoders encoders(8, 9, 11, 10);

// Pid setup
constexpr double kp = 1.75;
constexpr double ki = 0.15;
constexpr double kd = 0.2;
constexpr double setpoint = 4000;
PIDController pid(kp, ki, kd,  setpoint);

uint64_t elapsedTime = 0;

// Motor setup
// A is left motor, B is right motor
constexpr int MB1 = 3, MB2 = 4, MA1 = 6, MA2 = 7, PWMB = 2, PWMA = 5;

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
    uint16_t pos = sensor.readLine();

    const double dt = double(millis() - elapsedTime) / 1000;
    elapsedTime = millis();

    // Compute PID output
    const double pidOutput = pid.compute(pos, dt);

    auto sensorValues = sensor.getSensorValues();
    constexpr int numSensors = numSensorPins;
    constexpr int threshold = 600;  // Justert terskel for bedre følsomhet

    // Static variables to remember value beyond the loop
    static std::array<bool, numSensorPins> blackDetected = { true };
    static std::array<bool, numSensorPins> prevBlack = { true };

    // Analyser and update black detected sensors
    for (int i = 0; i < numSensors; i++)
    {
        blackDetected[i] = sensorValues[i] > threshold;
    }

    bool anyBlack = std::any_of(blackDetected.begin(), blackDetected.end(), [](bool b)
    {
        return b;
    });
    if (anyBlack)
    {
        std::copy(blackDetected.begin(), blackDetected.end(), prevBlack.begin());

        motor.setSpeedScaler(0.9);
        motor.updateOutput((long)pidOutput, -2000, 2000);
    }
    else
    {
        if (prevBlack[0])
        {
            motor.powerTurn(Motor::TurnDirection::Right);
        }
        else
        {
            motor.powerTurn(Motor::TurnDirection::Left);
        }
    }
}



