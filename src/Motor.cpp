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

void Motor::autoCalibrate(Sensor &sensor, int cycles)
{
    analogWrite(m_PWMLeft, 50);
    analogWrite(m_PWMRight, 50);
    int16_t toTurn = 35;
    Serial.println("left");
    //driveLeft();
    while (true)
    {
        m_encoders.update();
        if (m_encoders.getRelativeEncoderDiff())
            Serial.println(m_encoders.getTotalEncoderDiff());
        if (toTurn > 0 && m_encoders.getTotalEncoderDiff() >= toTurn)
        {
            toTurn = -toTurn;
            Serial.println("right");
            //driveRight();
        }
        else if (toTurn < 0 && m_encoders.getTotalEncoderDiff() <= toTurn)
        {
            toTurn = -toTurn;
            Serial.println("left");
            //driveLeft();
        }
        sensor.calibrate(1);

    }
    while (m_encoders.getTotalEncoderDiff() != 0)
    {
        if (m_encoders.getTotalEncoderDiff() > 0)
        {
            driveRight();
        }
        else
        {
            driveLeft();
        }
    }
    digitalWrite(m_leftForward, 1);
    digitalWrite(m_leftBackwards, 0);
    digitalWrite(m_rightForward, 1);
    digitalWrite(m_rightBackwards, 0);
    stop();
}

void Motor::updateOutput(long pidOutput, long pidMin, long pidMax)
{
    if (m_powerTurning)
    {
        m_relativeEncoderDiff += m_encoders.getRelativeEncoderDiff();
        if (turnIsFinished())
        {
            m_powerTurning = false;
            m_relativeEncoderDiff = 0;
        }
    }
    else
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
}

void Motor::stop() const
{
    analogWrite(m_PWMLeft, 0);
    analogWrite(m_PWMRight, 0);
}

void Motor::manualRun(unsigned char speed)
{
    digitalWrite(m_leftForward, 1);
    digitalWrite(m_leftBackwards, 0);
    /*digitalWrite(m_rightForward, 1);
    digitalWrite(m_rightBackwards, 0);*/
    analogWrite(m_PWMLeft, speed);
    //analogWrite(m_PWMRight, speed);
}

void Motor::powerTurn(int16_t degs)
{
    turn(degs);
    /*digitalWrite(m_PWMLeft, 1);
    digitalWrite(m_PWMRight, 1);*/
    analogWrite(m_PWMLeft, 255);
    analogWrite(m_PWMRight, 255);
    m_powerTurning = true;
}

void Motor::driveLeft()
{
    digitalWrite(m_leftForward, 1);
    digitalWrite(m_leftBackwards, 0);
    digitalWrite(m_rightForward, 0);
    digitalWrite(m_rightBackwards, 1);
}

void Motor::driveRight()
{
    digitalWrite(m_leftForward, 0);
    digitalWrite(m_leftBackwards, 1);
    digitalWrite(m_rightForward, 1);
    digitalWrite(m_rightBackwards, 0);
}

void Motor::turn(int16_t degs)
{
    m_toTurn = degs;
    if (degs > 0)
    {
        driveLeft();
    }
    else
    {
        driveRight();
    }
}

bool Motor::turnIsFinished() const
{
    return abs(m_relativeEncoderDiff) >= abs(m_toTurn);
}
