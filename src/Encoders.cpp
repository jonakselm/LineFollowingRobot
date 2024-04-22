#include "Encoders.hpp"

RotaryEncoder *encA = nullptr, *encB = nullptr;

Encoders::Encoders(int pinA1, int pinA2, int pinB1, int pinB2)
    : m_encoderA(pinA1, pinA2), m_encoderB(pinB1, pinB2),
      m_encoderDiff(0), m_relativeEncoderDiff(0),
      m_lastA(0), m_lastB(0), m_relativeEncoderDistance(0)
{
    encA = &m_encoderA;
    encB = &m_encoderB;
    attachInterrupt(pinA1, tickA, CHANGE);
    attachInterrupt(pinA2, tickA, CHANGE);
    attachInterrupt(pinB1, tickB, CHANGE);
    attachInterrupt(pinB2, tickB, CHANGE);
}

Encoders::~Encoders()
{
    encA = nullptr;
    encB = nullptr;
}

void Encoders::update()
{
    int32_t posA = m_encoderA.getPosition();
    int32_t posB = m_encoderB.getPosition();
    int32_t newEncoderDiff = posA - posB;
    m_relativeEncoderDiff = newEncoderDiff - m_encoderDiff;
    int diffA = posA - m_lastA;
    int diffB = posB - m_lastB;
    m_lastA = posA;
    m_lastB = posB;
    m_relativeEncoderDistance = abs(diffA) > abs(diffB) ? diffB : diffA;
    //Serial.println(m_relativeEncoderDistance);
    if (m_encoderDiff != newEncoderDiff)
    {
        /*Serial.println(m_encoderDiff);
        Serial.println(newEncoderDiff);*/
        m_encoderDiff = newEncoderDiff;
    }
    // No need to check both, as the condition for this
    // else-statement requires that there is no change
    // in their difference since last update
    else
    {
        /*Serial.println(diffA);
        Serial.println(diffB);*/
        //m_relativeEncoderDistance = min(diffA, diffB);
        //m_relativeEncoderDistance = abs(diffA) > abs(diffB) ? diffB : diffA;

        /*if (abs(diffA) > abs(diffB))
        {
            m_relativeEncoderDistance = diffB;
        }
        else
        {
            m_relativeEncoderDistance = diffA;
        }*/
    }
}

int32_t Encoders::getTotalEncoderDiff() const
{
    return m_encoderDiff;
}

int32_t Encoders::getRelativeEncoderDiff() const
{
    return m_relativeEncoderDiff;
}

void Encoders::tickA()
{
    if (encA)
    {
        encA->tick();
    }
}

void Encoders::tickB()
{
    if (encB)
    {
        encB->tick();
    }
}

int32_t Encoders::getTotalEncoderDistance() const
{
    return m_lastA - m_encoderDiff;
}

int32_t Encoders::getOrientation() const
{
    int32_t orientation = m_encoderDiff % 360;
    if (orientation < 0)
    {
        orientation += 360;
    }
    return orientation;
}

int32_t Encoders::getRelativeEncoderDistance() const
{
    return m_relativeEncoderDistance;
}
