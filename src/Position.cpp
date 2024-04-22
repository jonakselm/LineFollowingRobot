#include "Position.hpp"
#include <cmath>
#include <Arduino.h>

Position::Position()
{}

void Position::updatePosition(int relativeEncoderDistance, int totalEncoderDiff)
{
    double orientation = totalEncoderDiff % 360;
    if (orientation < 0)
    {
        orientation += 360;
    }
    orientation *= M_PI / 180;
    m_position.x += std::cos(orientation) * relativeEncoderDistance;
    m_position.y += std::sin(orientation) * relativeEncoderDistance;
}

Point Position::getPosition()
{
    return m_position;
}
