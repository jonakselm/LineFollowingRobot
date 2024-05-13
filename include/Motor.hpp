#ifndef LINEFOLLOWINGROBOT_MOTOR_HPP
#define LINEFOLLOWINGROBOT_MOTOR_HPP
#include "Sensor.hpp"
#include "Encoders.hpp"

class Motor
{
public:
    enum class TurnDirection { Left, Right };

    Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards,
          int PWMLeft, int PWMRight, Encoders &encoders);

    void autoCalibrate(Sensor &sensor, int cycles);
    void updateOutput(long pidOutput, long pidMin, long pidMax);
    void powerTurn(TurnDirection direction);

    void stop() const;
    void setSpeedScaler(double speedScaler);

private:
    void driveForward() const;
    void driveBackwards() const;
    void driveLeft() const;
    void driveRight() const;

    int m_leftForward, m_leftBackwards, m_rightForward, m_rightBackwards,
        m_PWMLeft, m_PWMRight;
    int MAX_SPEED = 255;
            ;
    Encoders &m_encoders;
    bool m_powerTurning = false;
    int16_t m_relativeEncoderDiff = 0;
};



#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
