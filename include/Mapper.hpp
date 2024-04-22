#ifndef LINEFOLLOWINGROBOT_MAPPER_HPP
#define LINEFOLLOWINGROBOT_MAPPER_HPP

#include <vector>
#include "Point.hpp"

class Mapper
{
public:
    void addPoint(Point &&point);

    const std::vector<Point> &getPoints() const;

    void reset();

private:
    std::vector<Point> m_points;
};


#endif //LINEFOLLOWINGROBOT_MAPPER_HPP
