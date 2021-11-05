#pragma once

#include <vector>

#include "location.h"

class VRP
{
public:
    const std::vector<Location> locations;

    VRP(std::initializer_list<Location> locations);

    double distance(size_t i, size_t j) const;

private:
    std::vector<double> distance_;
};
