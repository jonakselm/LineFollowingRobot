#ifndef LINEFOLLOWINGROBOT_SIMPLEMOTOR_HPP
#define LINEFOLLOWINGROBOT_SIMPLEMOTOR_HPP
#include <Arduino.h>


class SimpleMotor
{
public:
    enum class Motion { Empty, Forward, Left, Right, Backward, Stop, Switchoff };
public:
    SimpleMotor() = default;
    SimpleMotor(int leftForward, int leftBackwards, int rightForward, int rightBackward);
    void setMotorPins(int leftForward, int leftBackwards, int rightForward, int rightBackwards);
    void setDefSpeed(int speed);

    void updateMotion(int s17, int s15, int s13, int s11, int s9);

    void drive();

private:
    int m_leftForward, m_rightForward;
    int m_leftBackwards, m_rightBackwards;
    int m_lSpeed = 0, m_rSpeed = 0;
    int m_defSpeed, quickFix = 100;
};

#endif //LINEFOLLOWINGROBOT_SIMPLEMOTOR_HPP
