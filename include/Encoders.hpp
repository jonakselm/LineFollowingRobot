#ifndef LINEFOLLOWINGROBOT_ENCODERS_HPP
#define LINEFOLLOWINGROBOT_ENCODERS_HPP

#include <RotaryEncoder.h>
#include <cmath>

class Encoders
{
public:
    Encoders(int pinA1, int pinA2, int pinB1, int pinB2);
    ~Encoders();

    void update();

    int32_t getTotalEncoderDiff() const;

    int32_t getRelativeEncoderDiff() const;

    int32_t getTotalEncoderDistance() const;
    int32_t getRelativeEncoderDistance() const;

    int32_t getOrientation() const;

private:
    static void tickA();
    static void tickB();
    RotaryEncoder m_encoderA, m_encoderB;
    int32_t m_encoderDiff, m_relativeEncoderDiff;
    int32_t m_lastA, m_lastB, m_relativeEncoderDistance;
};

#endif //LINEFOLLOWINGROBOT_ENCODERS_HPP
