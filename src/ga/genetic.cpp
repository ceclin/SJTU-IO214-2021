#include "genetic.h"

#include <algorithm>
#include <utility>
#include <random>
#include <limits>

#include "chromosomes.h"
#include "mutation.h"

namespace
{
    void sort_populatoin(std::vector<Chromosomes> &population)
    {
        std::sort(population.begin(), population.end(), [](const Chromosomes &a, const Chromosomes &b)
                  { return a.fitness() < b.fitness(); });
    }

    int random_int(int min, int max)
    {
        return min + rand() % (max - min);
    }

    double random_p()
    {
        return rand() / static_cast<double>(RAND_MAX);
    }

    std::pair<size_t, size_t> binary_tournament(const std::vector<Chromosomes> &population)
    {
        size_t parent[2];
        for (size_t i = 0; i < 2; ++i)
        {
            size_t a = random_int(0, population.size());
            size_t b = random_int(0, population.size());
            if (population[a].fitness() <= population[b].fitness())
                parent[i] = a;
            else
                parent[i] = b;
        }
        return {parent[0], parent[1]};
    }
}

std::vector<Chromosomes> Genetic::initial_population()
{
    const auto &vrp = this->vrp.get();
    std::vector<Chromosomes> population;
    for (const auto &soln : initials)
    {
        Chromosomes ch(soln);
        size_t scaled = ch.fitness() / delta;
        ensure_table(scaled);
        if (table_[scaled])
            continue;
        table_[scaled] = true;
        population.push_back(std::move(ch));
    }
    std::vector<size_t> seq(vrp.locations.size() - 1);
    for (size_t i = 0; i < seq.size(); ++i)
    {
        seq[i] = i + 1;
    }
    for (size_t i = population.size(); i < sigma; ++i)
    {
        bool flag = true;
        for (size_t count = 0; count < tau; ++count)
        {
            std::random_shuffle(seq.begin(), seq.end());
            Chromosomes ch(vrp, seq);
            size_t scaled = ch.fitness() / delta;
            ensure_table(scaled);
            if (!table_[scaled])
            {
                table_[scaled] = true;
                population.push_back(std::move(ch));
                flag = false;
                break;
            }
        }
        if (flag)
            break;
    }
    sort_populatoin(population);
    return population;
}

Solution Genetic::optimize()
{
    auto population = initial_population();
    size_t n = population.size();
    for (size_t alpha = 0, beta = 0; alpha < alpha_max && beta < beta_max;)
    {
        auto parent = binary_tournament(population);
        auto children = Chromosomes::ox(population[parent.first], population[parent.second]);
        Chromosomes *c = nullptr;
        if (random_int(0, 2) == 0)
            c = &children.first;
        else
            c = &children.second;
        size_t k = random_int(n / 2 - 1, n);
        size_t scaled_k = population[k].fitness() / delta;
        if (random_p() < mutation_probability)
        {
            auto m = mutation_with_local_search(*c);
            size_t scaled_m = m.fitness() / delta;
            ensure_table(scaled_m);
            if (scaled_k == scaled_m || !table_[scaled_m])
                *c = std::move(m);
        }
        size_t scaled_c = c->fitness() / delta;
        ensure_table(scaled_c);
        if (scaled_k == scaled_c || !table_[scaled_c])
        {
            static double best = INFINITY;
            if (c->fitness() < best)
            {
                best = c->fitness();
                std::cout << best << std::endl;
            }
            ++alpha;
            if (alpha % 1000 == 0)
                std::cout << "alpha=" << alpha << std::endl;
            if (c->fitness() < population[0].fitness())
            {
                beta = 0;
            }
            else
                ++beta;
             if (beta % 1000 == 0)
                std::cout << "beta=" << beta << std::endl;
            table_[scaled_k] = false;
            population[k] = std::move(*c);
            table_[scaled_c] = true;
            sort_populatoin(population);
        }
    }
    return population[0];
}