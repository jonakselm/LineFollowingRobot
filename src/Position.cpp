#include "Position.hpp"
#include <cmath>
#include <Arduino.h>

Position::Position()
{}

void Position::updatePosition(int relativeDistance, int totalEncoderDiff)
{
    Serial.println(relativeDistance);
    double orientation = totalEncoderDiff % 360;
    if (orientation < 0)
    {
        orientation += 360;
    }
    orientation *= M_PI / 180;
    m_position.x += std::cos(orientation) * relativeDistance;
    m_position.y += std::sin(orientation) * relativeDistance;
}

Point Position::getPosition()
{
    return m_position;
}
