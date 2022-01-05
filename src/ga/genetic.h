#pragma once

#include <vector>

#include "../vrp.h"
#include "../solution.h"
#include "chromosomes.h"

class Genetic
{
public:
    std::reference_wrapper<const VRP> vrp;
    size_t sigma = 30; // 50
    size_t tau = 50;
    double delta = 0.5; // 1.0
    size_t alpha_max = 60000;
    size_t beta_max = 20000;
    double mutation_probability = 0.05;
    std::vector<Solution> initials;
    Genetic(const VRP &vrp)
        : vrp(vrp) {}
    /**
     * should be invoked only once
     */
    Solution optimize();

private:
    std::vector<bool> table_;

    std::vector<Chromosomes> initial_population();

    void ensure_table(size_t index)
    {
        if (index >= table_.size())
            table_.resize(index + 1);
    }
};