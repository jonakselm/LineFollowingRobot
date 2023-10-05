#include "SimpleMotor.hpp"

SimpleMotor::SimpleMotor(int leftForward, int leftBackwards, int rightForward, int rightBackwards)
    :
    m_leftForward(leftForward),
    m_rightForward(rightForward),
    m_leftBackwards(leftBackwards),
    m_rightBackwards(rightBackwards)
{
}

void SimpleMotor::setBaseSpeed(int speed)
{
    m_baseSpeed = speed;
}

void SimpleMotor::setMotorPins(int leftForward, int leftBackwards, int rightForward, int rightBackwards)
{
    m_leftForward = leftForward;
    m_rightForward = rightForward;
    m_leftBackwards = leftBackwards;
    m_rightBackwards = rightBackwards;
}

// Low values == whiteness, High values == blackness
void SimpleMotor::updateMotion(int s17, int s15, int s13, int s11, int s9)
{
    // s == sensor
    //
    //
    const int sMin = 260;
    const int sMax = 400;
    const int sRange = sMax - sMin;
    const int threshold = 60;
    int sLLeft = max(s17 - sMin - threshold, 0);
    int sLeft = max(s15 - sMin - threshold, 0);
    int sMid = max(s13 - sMin - threshold, 0);
    int sRight = max(s11 - sMin - threshold, 0);
    int sRRight = max(s9 - sMin - threshold, 0);

    /*Serial.print(sLLeft);
    Serial.print(' ');
    Serial.print(sLeft);
    Serial.print(' ');
    Serial.print(sMid);
    Serial.print(' ');
    Serial.print(sRight);
    Serial.print(' ');
    Serial.print(sRRight);
    Serial.print(" \n");*/

    double sNLLeft = double(sLLeft) / sRange * 2;
    double sNLeft = double(sLeft) / sRange * 2;
    double sNMid = double(sMid) / sRange * 2;
    double sNRight = double(sRight) / sRange * 2;
    double sNRRight = double(sRRight) / sRange * 2;

    /*Serial.print(sNLLeft);
    Serial.print(' ');
    Serial.print(sNLeft);
    Serial.print(' ');
    Serial.print(sNMid);
    Serial.print(' ');
    Serial.print(sNRight);
    Serial.print(' ');
    Serial.print(sNRRight);
    Serial.print('\n');*/

    // Right goes double the speed of left for some reason
    m_lSpeed = min(int((sNRRight + sNRight + sNMid) * m_baseSpeed), 255);
    m_rSpeed = min(int((sNLLeft + sNLeft + sNMid) * m_baseSpeed), 255);

    Serial.print(m_lSpeed);
    Serial.print(' ');
    Serial.print(m_rSpeed);
    Serial.print('\n');
}

void SimpleMotor::drive()
{
    analogWrite(m_leftForward, m_lSpeed);
    analogWrite(m_leftBackwards, 0);
    analogWrite(m_rightForward, m_rSpeed);
    analogWrite(m_rightBackwards, 0);
}
