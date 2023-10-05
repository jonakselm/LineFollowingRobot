#include "SimpleMotor.hpp"

SimpleMotor::SimpleMotor(int leftForward, int leftBackwards, int rightForward, int rightBackwards)
    :
    m_leftForward(leftForward),
    m_leftBackwards(leftBackwards),
    m_rightForward(rightForward),
    m_rightBackwards(rightBackwards)
{
}

// Framover funksjon
void SimpleMotor::Forward()
{
    Serial.println("Going forward...");
    analogWrite(m_leftForward, m_speed);
    analogWrite(m_leftBackwards, 0);
    analogWrite(m_rightForward, m_speed);
    analogWrite(m_rightBackwards, 0);
}

// Backover funksjon
void SimpleMotor::Backward()
{
    Serial.println("Going backward...");
    analogWrite(m_leftForward, 0);
    analogWrite(m_leftBackwards, m_speed);
    analogWrite(m_rightForward, 0);
    analogWrite(m_rightBackwards, m_speed);
}

void SimpleMotor::Left()
{
    Serial.println("Going left...");
    analogWrite(m_leftForward, 0);
    analogWrite(m_leftBackwards, 0);
    analogWrite(m_rightForward, m_speed);
    analogWrite(m_rightBackwards, 0);
    m_motion = Motion::Left;
}

void SimpleMotor::Right()
{
    Serial.println("Going right...");
    analogWrite(m_leftForward, m_speed);
    analogWrite(m_leftBackwards, 0);
    analogWrite(m_rightForward, 0);
    analogWrite(m_rightBackwards, 0);
    m_motion = Motion::Right;
}

// Stop funksjon
void SimpleMotor::Stop()
{
    Serial.println("Stopping...");
    digitalWrite(m_leftForward, 0);
    digitalWrite(m_leftBackwards, 0);
    digitalWrite(m_rightForward, 0);
    digitalWrite(m_rightBackwards, 0);
}

void SimpleMotor::setSpeed(uint8_t speed)
{
    m_speed = speed;
}

void SimpleMotor::setMotorPins(int leftForward, int leftBackwards, int rightForward, int rightBackwards)
{
    m_leftForward = leftForward;
    m_leftBackwards = leftBackwards;
    m_rightForward = rightForward;
    m_rightBackwards = rightBackwards;
    Serial.println(m_leftForward);
    Serial.println(m_leftBackwards);
    Serial.println(m_rightForward);
    Serial.println(m_rightBackwards);
}

void SimpleMotor::setMotion(SimpleMotor::Motion motion)
{
    m_motion = motion;
}

void SimpleMotor::drive()
{
    // Bytter motor funksjon / Kanskje legge til speed control input ogs�?
    switch (m_motion)
    {
        case Motion::Forward:      // Forward
            Forward();
            break;
        case Motion::Left:         // Left
            Left();
            break;
        case Motion::Right:        // Right
            Right();
            break;
        case Motion::Backward:     // Backward
            Backward();
            break;
        case Motion::Stop:         // Stop
            Stop();
            break;
        default:           // Default stop
            Stop();
            break;
    }
}

SimpleMotor::Motion SimpleMotor::getMotion() const
{
    return m_motion;
}
