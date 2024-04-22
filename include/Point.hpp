#ifndef LINEFOLLOWINGROBOT_POINT_HPP
#define LINEFOLLOWINGROBOT_POINT_HPP

struct Point
{
    Point() : x(0), y(0) {}
    bool operator!=(const Point &other) const
    {
        return this->x != other.x || this->y != other.y;
    }
    double x, y;
};

#endif //LINEFOLLOWINGROBOT_POINT_HPP
