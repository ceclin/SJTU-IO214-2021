#include "vrp.h"

VRP::VRP(
    std::initializer_list<VehicleMeta> vehicle_types,
    std::initializer_list<Location> locations)
    : vehicle_types(vehicle_types), locations(locations)
{
    for (size_t j = 0; j < this->locations.size(); ++j)
    {
        for (size_t i = 0; i <= j; ++i)
        {
            distance_.push_back(Location::distance(this->locations[j], this->locations[i]));
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