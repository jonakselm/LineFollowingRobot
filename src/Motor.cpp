#include <Motor.hpp>
#include <Arduino.h>

Motor::Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards,
             int PWMLeft, int PWMRight)
    : m_leftForward(leftForward),
    m_leftBackwards(leftBackwards),
    m_rightForward(rightForward),
    m_rightBackwards(rightBackwards),
    m_PWMLeft(PWMLeft),
    m_PWMRight(PWMRight)
{
}

void Motor::updateOutput(long pidOutput, long pidMin, long pidMax)
{
    pidOutput = min(pidOutput, pidMax);
    pidOutput = max(pidOutput, pidMin);
    digitalWrite(m_leftForward, 1);
    digitalWrite(m_leftBackwards, 0);
    digitalWrite(m_rightForward, 1);
    digitalWrite(m_rightBackwards, 0);

    /*
     * Left is negative, Right is positive
     * 0 == 255
     * Pid negative == Left < Right
     * Pid positive == Right < Left
     * */
    int outputMapped = (int)map(pidOutput, pidMin, pidMax, -DEFAULT_SPEED, DEFAULT_SPEED - 1);
    analogWrite(m_PWMLeft, DEFAULT_SPEED + outputMapped);
    analogWrite(m_PWMRight, DEFAULT_SPEED - outputMapped);

}

void Motor::stop() const
{
    analogWrite(m_PWMLeft, 0);
    analogWrite(m_PWMRight, 0);
}