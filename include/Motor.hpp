#ifndef LINEFOLLOWINGROBOT_MOTOR_HPP
#define LINEFOLLOWINGROBOT_MOTOR_HPP
#include "Sensor.hpp"

class Motor
{
public:
    Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards,
          int PWMLeft, int PWMRight);

    void autoCalibrate(Sensor &sensor, int cycles);
    void updateOutput(long pidOutput, long pidMin, long pidMax);
    void powerTurn(bool left);

    void manualRun(unsigned char speed);

    void stop() const;

private:
    void driveLeft();
    void driveRight();

    int m_leftForward, m_leftBackwards, m_rightForward, m_rightBackwards,
        m_PWMLeft, m_PWMRight;
    static constexpr int MAX_SPEED = 255 * 0.70;
};

#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
