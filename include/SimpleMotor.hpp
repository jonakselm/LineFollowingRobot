#ifndef LINEFOLLOWINGROBOT_SIMPLEMOTOR_HPP
#define LINEFOLLOWINGROBOT_SIMPLEMOTOR_HPP
#include <Arduino.h>

///////////////////////
// Helper funksjoner //
///////////////////////

// Motion for switch for � lett kontrollere retning og setter default state til stop.
class SimpleMotor
{
public:
    enum class Motion { Empty, Forward, Left, Right, Backward, Stop, Switchoff };
public:
    SimpleMotor() = default;
    SimpleMotor(int leftForward, int leftBackwards, int rightForward, int rightBackward);
    void setMotorPins(int leftForward, int leftBackwards, int rightForward, int rightBackwards);
    void setSpeed(uint8_t speed);

    void setMotion(Motion motion);
    Motion getMotion() const;

    void drive();

private:
    void Forward();
    void Backward();
    void Left();
    void Right();
    void Stop();

private:
    int m_leftForward, m_leftBackwards;
    int m_rightForward, m_rightBackwards;
    uint8_t m_speed, quickFix = 100;
    Motion m_motion = Motion::Empty;
};

#endif //LINEFOLLOWINGROBOT_SIMPLEMOTOR_HPP
