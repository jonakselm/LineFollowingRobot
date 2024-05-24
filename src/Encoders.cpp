#include "Encoders.hpp"

RotaryEncoder *encA = nullptr, *encB = nullptr;

Encoders::Encoders(int pinA1, int pinA2, int pinB1, int pinB2)
    : m_encoderA(pinA1, pinA2), m_encoderB(pinB1, pinB2),
      m_encoderDiff(0), m_relativeEncoderDiff(0),
      m_pinA1(pinA1), m_pinA2(pinA2), m_pinB1(pinB1), m_pinB2(pinB2)
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
    detachInterrupt(m_pinA1);
    detachInterrupt(m_pinA2);
    detachInterrupt(m_pinB1);
    detachInterrupt(m_pinB2);
    encA = nullptr;
    encB = nullptr;
}

void Encoders::update()
{
    int64_t newEncoderDiff = m_encoderA.getPosition() - m_encoderB.getPosition();
    m_relativeEncoderDiff = newEncoderDiff - m_encoderDiff;
    if (m_encoderDiff != newEncoderDiff)
    {
        m_encoderDiff = newEncoderDiff;
    }
}


int64_t Encoders::getTotalEncoderDiff() const
{
    return m_encoderDiff;
}

int64_t Encoders::getRelativeEncoderDiff() const
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
