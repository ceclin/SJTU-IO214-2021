#pragma once

#include <functional>
#include <vector>
#include <iostream>

#include "vrp.h"
#include "vehicle.h"

class Solution
{
    friend std::ostream &operator<<(std::ostream &os, const Solution &soln);

public:
    std::reference_wrapper<const VRP> vrp;
    std::vector<Vehicle> vehicles;

    Solution(const VRP &vrp) : vrp(vrp), vehicles() {}
};
