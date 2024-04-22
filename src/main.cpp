#include <atomic>
#include "Arduino.h"
#include "Pid.hpp"
#include "Sensor.hpp"
#include "Motor.hpp"
#include "Encoders.hpp"
#include "Mapper.hpp"
#include "Position.hpp"

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

// Motor setup
// A is left motor, B is right motor
constexpr int MA1 = 3, MA2 = 4, MB1 = 6, MB2 = 7, PWMA = 2, PWMB = 5;
/*constexpr int MA1 = 3,  MA2 = 4,
        MB1 = 6, MB2 = 7,
        PWMA = 2, PWMB = 5;*/

Motor motor(MA1, MA2, MB1, MB2, PWMA, PWMB, encoders);

Mapper mapper;
Position position;

uint64_t elapsedTime = 0, timeSincePoll = 0;

void setup()
{
    // Sensor setup
    Serial.begin(115200);
    motor.autoCalibrate(sensor, 300);
    //motor.manualRun(255);
}

void loop()
{
    elapsedTime = millis();
    encoders.update();
    /*if (encoders.getRelativeEncoderDiff())
    {
        Serial.println(encoders.getTotalEncoderDiff());
    }*/
    if (elapsedTime > timeSincePoll + 100)
    {
        timeSincePoll = elapsedTime;
        position.updatePosition(encoders.getRelativeEncoderDistance(), encoders.getTotalEncoderDiff());
    }
    static Point lastPos;
    if (position.getPosition().x > 20)
    {
        motor.stop();
    }
    /*if (lastPos != position.getPosition())
    {
        Serial.print(position.getPosition().x);
        Serial.print(", ");
        Serial.println(position.getPosition().y);
        mapper.addPoint(position.getPosition());
    }*/
    /*uint16_t pos = sensor.readLine();

    double dt = double(millis() - elapsedTime) / 1000;
    elapsedTime = millis();

    // Compute PID output
    double pidOutput = pid.compute(pos, dt);
    motor.updateOutput((long)pidOutput, -2000, 2000);*/
}

