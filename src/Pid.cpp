//
// Created by oscar on 22.01.2024.
//

#include "Pid.hpp"

PIDController::PIDController(double kp, double ki, double kd, double setpoint) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->setpoint = setpoint;
    this->error = 0.0;
    this->integral = 0.0;
    this->derivative = 0.0;
    this->prevError = 0.0;

}

double PIDController::compute(double current, double dt) {
    error = setpoint - current;

    integral += (error + prevError) * dt / 2.0;
    derivative = (error - prevError) / dt;

    double Pid_Output = kp * error + ki * integral + kd * derivative;

    prevError = error;

    return Pid_Output;
}

void PIDController::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
};






