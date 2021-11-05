#pragma once

#include <iostream>

struct VehicleMeta
{
    friend std::ostream &operator<<(std::ostream &os, const VehicleMeta &meta);
    int capacity;
    int cost;
};
