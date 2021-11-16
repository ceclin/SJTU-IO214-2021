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

    inline double fitness() const { return fitness_; }

private:
    std::vector<size_t> seq_;
    double fitness_ = 0;
};