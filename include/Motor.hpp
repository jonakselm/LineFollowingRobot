#ifndef LINEFOLLOWINGROBOT_MOTOR_HPP
#define LINEFOLLOWINGROBOT_MOTOR_HPP

///////////////////////
// Helper funksjoner //
///////////////////////

// Motion for switch for � lett kontrollere retning og setter default state til stop.
class Motor
{
public:
    enum class Motion { Empty, Forward, Left, Right, Backward, Stop, Switchoff };
public:
    Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards);
    void Forward();
    void Backward();
    void Left();
    void Right();
    void Stop();
    void setSpeed(uint8_t speed);
    void MotorControl(Motion motion);

private:
    const int m_leftForward, m_leftBackwards;
    const int m_rightForward, m_rightBackwards;
    uint8_t m_speed;
}

Motor::Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards)
    : m_leftForward(leftForward), m_leftBackwards(leftBackwards),
    m_rightForward(rightForward), m_rightBackwards(rightBackwards)
{
}

// Framover funksjon
void Motor::Forward()
{
    Serial.println("Going forward...");
    analogWrite(m_leftForward, m_speed);
    analogWrite(m_leftBackwards, 0);
    analogWrite(m_rightForward, m_speed);
    analogWrite(m_rightBackwards, 0);
}

// Backover funksjon
void Motor::Backward()
{
    Serial.println("Going backward...");
    analogWrite(m_leftForward, 0);
    analogWrite(m_leftBackwards, m_speed);
    analogWrite(m_rightForward, 0);
    analogWrite(m_rightBackwards, m_speed);
}

void Motor::Right()
{
    Serial.println("Going left...");
    analogWrite(m_leftForward, m_speed);
    analogWrite(m_leftBackwards, 0);
    analogWrite(m_rightForward, 0);
    analogWrite(m_rightBackwards, m_speed);
}

void Motor::Left()
{
    Serial.println("Going right...");
    analogWrite(m_leftForward, 0);
    analogWrite(m_leftBackwards, m_speed);
    analogWrite(m_rightForward, m_speed);
    analogWrite(m_rightBackwards, 0);
}

// Stop funksjon
void Motor::Stop()
{
    Serial.println("Stopping...");
    digitalWrite(m_leftForward, 0);
    digitalWrite(m_leftBackwards, 0);
    digitalWrite(m_rightForward, 0);
    digitalWrite(m_rightBackwards, 0);
}

void Motor::MotorControl(Motion arg)
{
    // Bytter motor funksjon / Kanskje legge til speed control input ogs�?
    switch (arg)
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

void Motor::setSpeed(uint8_t speed)
{
    m_speed = speed;
}

#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
