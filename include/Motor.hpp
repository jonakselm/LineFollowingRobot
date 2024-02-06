#ifndef LINEFOLLOWINGROBOT_MOTOR_HPP
#define LINEFOLLOWINGROBOT_MOTOR_HPP

class Motor
{
public:
    Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards,
          int PWMLeft, int PWMRight);

    void updateOutput(long pidOutput, long pidMin, long pidMax);

    void stop() const;

private:
    int m_leftForward, m_leftBackwards, m_rightForward, m_rightBackwards,
        m_PWMLeft, m_PWMRight;
    static constexpr int DEFAULT_SPEED = 128;
};

#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
