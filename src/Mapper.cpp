#include "Mapper.hpp"

void Mapper::addPoint(Point &&point)
{
    m_points.push_back(point);
}

const std::vector<Point> &Mapper::getPoints() const
{
    return m_points;
}