#ifndef LINEFOLLOWINGROBOT_ENCODERS_HPP
#define LINEFOLLOWINGROBOT_ENCODERS_HPP

#include <RotaryEncoder.h>

class Encoders
{
public:
    Encoders(int pinA1, int pinA2, int pinB1, int pinB2);
    ~Encoders();

    void update();

    int64_t getTotalEncoderDiff() const;

    int64_t getRelativeEncoderDiff() const;

private:
    static void tickA();
    static void tickB();

    RotaryEncoder m_encoderA, m_encoderB;
    int64_t m_encoderDiff, m_relativeEncoderDiff;
    const int m_pinA1, m_pinA2, m_pinB1, m_pinB2;
};

#endif //LINEFOLLOWINGROBOT_ENCODERS_HPP
