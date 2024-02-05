//
// Created by oscar on 22.01.2024.
//

#ifndef LINEFOLLOWINGROBOT_PID_HPP
#define LINEFOLLOWINGROBOT_PID_HPP

class PIDController {
private:
    double kp, ki, kd;     // PID gains
    double setpoint;       // Desired setpoint
    double error, integral, derivative, prevError;  // PID terms

public:
    PIDController(double kp, double ki, double kd, double setpoint);
    double compute(double current, double dt);
    void setSetpoint(double setpoint);
};

#endif //LINEFOLLOWINGROBOT_PID_HPP
