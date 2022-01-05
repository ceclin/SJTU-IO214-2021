#pragma once

#include <vector>
#include <utility>

#include "../vrp.h"
#include "../solution.h"

class Chromosomes
{
public:
    static std::pair<Chromosomes, Chromosomes> ox(const Chromosomes &a, const Chromosomes &b);
    std::reference_wrapper<const VRP> vrp;

    Chromosomes(const VRP &vrp, const std::vector<size_t> &seq);
    explicit Chromosomes(const Solution &soln);
    operator Solution() const;

    inline double fitness() const { return fitness_; }

private:
    std::vector<size_t> seq_;
    double fitness_ = 0;
};