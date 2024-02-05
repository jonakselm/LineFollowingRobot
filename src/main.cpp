#include "Arduino.h"
#include "Pid.hpp"
#include "Sensor.hpp"

// Sensor pin names reflect the number on the sensor chip
enum SensorPin { s9 = A0, s11, s13, s15, s17 };
uint8_t sensorPins[] = { s9, s11, s13, s15, s17 };
Sensor sensor(sensorPins, 5);

// Pid setup
double kp = 1;
double ki = 0.1;
double kd = 0.01;
double setpoint = 2000;
PIDController pid(kp, ki, kd,  setpoint);

void setup()
{
    // Sensor setup
    Serial.begin(9600);
    /*sensor.calibrate(600);
    sensor.saveCalibration();*/
    sensor.loadCalibration();
}

void loop()
{
    Serial.println(sensor.readLine());

    double current_value = 1000;


    double dt = 1;

    // Compute PID output
    double pid_output = pid.compute(current_value, dt);
    Serial.println(pid_output);

    // Assuming you have code to apply the PID output to your system (e.g., a motor)
    // ...

    // Update setpoint if needed
    // pid.setSetpoint(new_desired_setpoint);
}

