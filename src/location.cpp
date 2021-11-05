#include "location.h"

#include <cmath>

std::ostream &operator<<(std::ostream &os, const Location &lo)
{
    return os << "(x=" << lo.x << ", y=" << lo.y << ", demand=" << lo.demand << ")";
}

double Location::distance(const Location &a, const Location &b)
{
    int dx = a.x - b.x;
    int dy = a.y - b.y;
    double dz = std::hypot(dx, dy);
    return std::floor(dz * 1000) / 1000;
}
