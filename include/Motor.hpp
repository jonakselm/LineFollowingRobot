#ifndef LINEFOLLOWINGROBOT_MOTOR_HPP
#define LINEFOLLOWINGROBOT_MOTOR_HPP
#include "Sensor.hpp"
#include "Encoders.hpp"

class Motor
{
public:
    Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards,
          int PWMLeft, int PWMRight, Encoders &encoders);

    void autoCalibrate(Sensor &sensor, int cycles);
    void updateOutput(long pidOutput, long pidMin, long pidMax);
    void powerTurn(int16_t degs);

    void manualRun(unsigned char speed);

    void stop() const;

private:
    void driveLeft();
    void driveRight();
    void turn(int16_t degs);
    bool turnIsFinished() const;

    int m_leftForward, m_leftBackwards, m_rightForward, m_rightBackwards,
        m_PWMLeft, m_PWMRight;
    static constexpr int MAX_SPEED = 255 * 0.70;
    Encoders &m_encoders;
    bool m_powerTurning = false;
    int16_t m_toTurn = 0;
    int16_t m_relativeEncoderDiff = 0;
};

#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
