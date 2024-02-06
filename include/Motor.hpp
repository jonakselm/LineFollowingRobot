#ifndef LINEFOLLOWINGROBOT_MOTOR_HPP
#define LINEFOLLOWINGROBOT_MOTOR_HPP

class Motor
{
public:
    Motor(int AIN0, int AIN1, int BIN0, int BIN1);
    void updateOutput(double pidOutput);



private:
    int AIN0, AIN1, BIN0, BIN1;
    double PWMA = 0, PWMB = 0;
};

#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
