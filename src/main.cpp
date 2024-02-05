#include "pid.hpp"
#include "Arduino.h"

int main() {
    // Assuming you have some setup code here
    double kp = 1;
    double ki = 0.1;
    double kd = 0.01;
    double setpoint = 2000;
    PIDController pid(kp, ki, kd,  setpoint);

    // Assuming you have a loop where you read the current value and update the control
    Serial.begin(9600);
    while (true) {
        double current_value = 1000;


        double dt = 1;

        // Compute PID output
        double pid_output = pid.compute(current_value, dt);
        Serial.println(pid_output);

        // Assuming you have code to apply the PID output to your system (e.g., a motor)
        // ...

        // Update setpoint if needed
       // pid.setSetpoint(new_desired_setpoint);

        // Other control logic as needed

        // Delay or sleep to control the loop frequency
        // ...

    }

    return 0;
}
