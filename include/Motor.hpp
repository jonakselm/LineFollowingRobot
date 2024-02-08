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

    void stop() const;

private:
    int m_leftForward, m_leftBackwards, m_rightForward, m_rightBackwards,
        m_PWMLeft, m_PWMRight;
    static constexpr int MAX_SPEED = 210;
};

#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
