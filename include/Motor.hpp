#ifndef LINEFOLLOWINGROBOT_MOTOR_HPP
#define LINEFOLLOWINGROBOT_MOTOR_HPP
#include "Sensor.hpp"
#include "Encoders.hpp"
#include "Position.hpp"
#include "Mapper.hpp"

class Motor
{
public:
    Motor(int leftForward, int leftBackwards, int rightForward, int rightBackwards,
          int PWMLeft, int PWMRight, Encoders &encoders, Position &position,
          Mapper &mapper);

    void autoCalibrate(Sensor &sensor, int cycles);
    void updateOutput(long pidOutput, long pidMin, long pidMax);
    void powerTurn(int16_t degs);

    void manualRun(unsigned char speed);

    void stop() const;

    void driveAfterArray(const std::vector<Point> &map);

private:
    void driveForward();
    void driveBackwards();
    void driveLeft();
    void driveRight();
    void turn(int16_t degs);
    bool turnIsFinished() const;

    int m_leftForward, m_leftBackwards, m_rightForward, m_rightBackwards,
        m_PWMLeft, m_PWMRight;
    static constexpr int MAX_SPEED = 255 * 0.60;
    Encoders &m_encoders;
    bool m_powerTurning = false;
    int16_t m_toTurn = 0;
    int16_t m_relativeEncoderDiff = 0;
    Position &m_position;
    Mapper &m_mapper;
    int m_pointIndex = 0;
    static constexpr int POINT_SAMPLE = 30;
};

#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
