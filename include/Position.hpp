#ifndef LINEFOLLOWINGROBOT_POSITION_HPP
#define LINEFOLLOWINGROBOT_POSITION_HPP

#include "Point.hpp"

class Position
{
public:
    Position();

    void updatePosition(int relativeEncoderDistance, int totalEncoderDiff);

    Point getPosition();

private:
    Point m_position;
};

#endif //LINEFOLLOWINGROBOT_POSITION_HPP
