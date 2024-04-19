#ifndef LINEFOLLOWINGROBOT_ENCODERS_HPP
#define LINEFOLLOWINGROBOT_ENCODERS_HPP

#include <RotaryEncoder.h>

class Encoders
{
public:
    Encoders(int pinA1, int pinA2, int pinB1, int pinB2);
    ~Encoders();

    void update();

    int16_t getTotalEncoderDiff() const;

    int16_t getRelativeEncoderDiff() const;

private:
    static void tickA();
    static void tickB();
    RotaryEncoder m_encoderA, m_encoderB;
    int16_t m_encoderDiff, m_relativeEncoderDiff;
};

#endif //LINEFOLLOWINGROBOT_ENCODERS_HPP
