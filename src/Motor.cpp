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

void Motor::autoCalibrate(Sensor &sensor, int cycles)
{
    bool goingLeft = false;
    auto driveLeft = [&]()
    {
        digitalWrite(m_leftForward, 1);
        digitalWrite(m_leftBackwards, 0);
        digitalWrite(m_rightForward, 0);
        digitalWrite(m_rightBackwards, 1);
        goingLeft = true;
    };
    auto driveRight = [&]()
    {
        digitalWrite(m_leftForward, 0);
        digitalWrite(m_leftBackwards, 1);
        digitalWrite(m_rightForward, 1);
        digitalWrite(m_rightBackwards, 0);
        goingLeft = false;
    };

    analogWrite(m_PWMLeft, 100);
    analogWrite(m_PWMRight, 100);
    // Negative == left, Positive == right
    int cyclesOffset = 0;
    constexpr int amountToEdge = 10;
    for (int i = 0; i < amountToEdge - 1; i++)
    {
        driveLeft();
        cyclesOffset -= 1;
    }
    for (int i = amountToEdge; i < cycles; i++)
    {
        if (goingLeft = (i / amountToEdge % 2 == 0))
        {
            driveLeft();
            cyclesOffset -= 1;
        }
        else
        {
            driveRight();
            cyclesOffset += 1;
        }
        Serial.print("Going left: ");
        Serial.print(goingLeft);
        Serial.print(", Cycles: ");
        Serial.println(i);
        sensor.calibrate(1);
    }
    for (int i = 0; i < abs(cyclesOffset); i++)
    {
        // Positive needs correction left
        if (cyclesOffset > 0)
        {
            driveLeft();
            cyclesOffset -= 1;
        }
        // Positive needs correction right
        else
        {
            driveRight();
            cyclesOffset += 1;
        }
        Serial.print("Going left: ");
        Serial.print(goingLeft);
        Serial.print(", Cycles: ");
        Serial.println(i);
        sensor.calibrate(1);
    }
    digitalWrite(m_leftForward, 1);
    digitalWrite(m_leftBackwards, 0);
    digitalWrite(m_rightForward, 1);
    digitalWrite(m_rightBackwards, 0);
    stop();
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
    int outputMapped = (int)map(pidOutput, pidMin, pidMax, -MAX_SPEED, MAX_SPEED - 1);
    int speedLeft = MAX_SPEED;
    int speedRight = MAX_SPEED;
    int absOutput = abs(outputMapped);
    if (outputMapped > 0)
        speedLeft -= absOutput;
    else
        speedRight -= absOutput;
    analogWrite(m_PWMLeft, speedLeft);
    analogWrite(m_PWMRight, speedRight);
}

void Motor::stop() const
{
    analogWrite(m_PWMLeft, 0);
    analogWrite(m_PWMRight, 0);
}