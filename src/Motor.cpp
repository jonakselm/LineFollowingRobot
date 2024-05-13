#include <Motor.hpp>
#include <Arduino.h>

Motor::Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards,
             int PWMLeft, int PWMRight, Encoders &encoders)
    : m_leftForward(leftForward),
      m_leftBackwards(leftBackwards),
      m_rightForward(rightForward),
      m_rightBackwards(rightBackwards),
      m_PWMLeft(PWMLeft),
      m_PWMRight(PWMRight),
      m_encoders(encoders)
{
}

void Motor::powerTurn(TurnDirection direction)
{
    if (direction == TurnDirection::Left)
    {
        driveLeft();
    }
    else
    {
        driveRight();
    }

    analogWrite(m_PWMLeft, MAX_SPEED);
    analogWrite(m_PWMRight, MAX_SPEED);
}


void Motor::autoCalibrate(Sensor &sensor, int cycles)
{
    analogWrite(m_PWMLeft, 50);
    analogWrite(m_PWMRight, 50);
    // Degrees to turn
    int16_t toTurn = 20;
    driveLeft();
    for (int i = 0; i < cycles; i++)
    {
        m_encoders.update();

        if (toTurn > 0 && m_encoders.getTotalEncoderDiff() >= toTurn)
        {
            toTurn = -toTurn;
            driveRight();
        }
        else if (toTurn < 0 && m_encoders.getTotalEncoderDiff() <= toTurn)
        {
            toTurn = -toTurn;
            driveLeft();
        }
        sensor.calibrate(1);
    }
    while (abs(m_encoders.getTotalEncoderDiff()) != 0)
    {
        m_encoders.update();
        if (m_encoders.getTotalEncoderDiff() > 0)
        {
            driveRight();
        }
        else
        {
            driveLeft();
        }
        sensor.calibrate(1);
    }
}

void Motor::updateOutput(long pidOutput, long pidMin, long pidMax)
{
    pidOutput = min(pidOutput, pidMax);
    pidOutput = max(pidOutput, pidMin);
    driveForward();

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

void Motor::driveForward() const
{
    digitalWrite(m_leftForward, 1);
    digitalWrite(m_leftBackwards, 0);
    digitalWrite(m_rightForward, 1);
    digitalWrite(m_rightBackwards, 0);
}

void Motor::driveBackwards() const
{
    digitalWrite(m_leftForward, 0);
    digitalWrite(m_leftBackwards, 1);
    digitalWrite(m_rightForward, 0);
    digitalWrite(m_rightBackwards, 1);
}

void Motor::driveLeft() const
{
    digitalWrite(m_leftForward, 1);
    digitalWrite(m_leftBackwards, 0);
    digitalWrite(m_rightForward, 0);
    digitalWrite(m_rightBackwards, 1);
}

void Motor::driveRight() const
{
    digitalWrite(m_leftForward, 0);
    digitalWrite(m_leftBackwards, 1);
    digitalWrite(m_rightForward, 1);
    digitalWrite(m_rightBackwards, 0);
}

void Motor::setSpeedScaler(double speedScaler)
{
    MAX_SPEED = 255 * speedScaler;
}

