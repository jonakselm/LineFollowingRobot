#include "Encoders.hpp"

RotaryEncoder *encA = nullptr, *encB = nullptr;

Encoders::Encoders(int pinA1, int pinA2, int pinB1, int pinB2)
    : m_encoderA(pinA1, pinA2), m_encoderB(pinB1, pinB2),
      m_encoderDiff(0), m_relativeEncoderDiff(0)
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
    int16_t newEncoderDiff = m_encoderA.getPosition() - m_encoderB.getPosition();
    m_relativeEncoderDiff = newEncoderDiff - m_encoderDiff;
    if (m_encoderDiff != newEncoderDiff)
    {
        m_encoderDiff = newEncoderDiff;
    }
}

int16_t Encoders::getTotalEncoderDiff() const
{
    return m_encoderDiff;
}

int16_t Encoders::getRelativeEncoderDiff() const
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
