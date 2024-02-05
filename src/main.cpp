#include "Arduino.h"
#include "Pid.hpp"
#include "Sensor.hpp"

// Sensor pin names reflect the number on the sensor chip
enum SensorPin { s9 = A0, s11, s13, s15, s17 };
uint8_t sensorPins[] = { s9, s11, s13, s15, s17 };
Sensor sensor(sensorPins, 5);

// Pid setup
double kp = 1;
double ki = 0.0;
double kd = 0.01;
double setpoint = 2000;
PIDController pid(kp, ki, kd,  setpoint);

uint64_t elapsedTime = 0;

void setup()
{
    // Sensor setup
    Serial.begin(9600);
    sensor.calibrate(75);
}

void loop()
{
    uint16_t pos = sensor.readLine();
    Serial.print("Pos: ");
    Serial.println(pos);
    // 0 == høyre, 4000 == venstre
    //Serial.println(pos);


    double dt = double(millis() - elapsedTime) / 1000;
    elapsedTime = millis();

    // Compute PID output
    double pid_output = pid.compute(pos, dt);
    Serial.print("PID: ");
    Serial.println(pid_output);

    // Assuming you have code to apply the PID output to your system (e.g., a motor)
    // ...

    // Update setpoint if needed
    // pid.setSetpoint(new_desired_setpoint);
    delay(500);
}

