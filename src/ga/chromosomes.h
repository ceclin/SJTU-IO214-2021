#pragma once

#include <vector>

#include "../vrp.h"
#include "../solution.h"

class Chromosomes
{
public:
    std::reference_wrapper<const VRP> vrp;

    explicit Chromosomes(const Solution &soln);
    operator Solution() const;

    inline int fitness() const { return fitness_; }

private:
    std::vector<size_t> seq_;
    int fitness_ = 0;
};