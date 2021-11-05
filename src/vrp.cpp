#include "vrp.h"

VRP::VRP(std::initializer_list<Location> location_list) : locations(location_list)
{
    for (size_t j = 0; j < locations.size(); ++j)
    {
        for (size_t i = 0; i <= j; ++i)
        {
            distance_.push_back(Location::distance(locations[j], locations[i]));
        }
    }
}

double VRP::distance(size_t i, size_t j) const
{
    if (i == j)
        return 0;
    if (i > j)
        return distance(j, i);
    size_t index = j * (j + 1) / 2 + i;
    return distance_[index];
}