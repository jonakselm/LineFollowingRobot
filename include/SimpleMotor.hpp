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
    void Forward();
    void Backward();
    void Left();
    void Right();
    void Stop();
    void setSpeed(uint8_t speed);
    void MotorControl(Motion motion);

private:
    int m_leftForward, m_leftBackwards;
    int m_rightForward, m_rightBackwards;
    uint8_t m_speed;
};

#endif //LINEFOLLOWINGROBOT_SIMPLEMOTOR_HPP
