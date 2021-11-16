#include "chromosomes.h"

#include <algorithm>
#include <limits>

Chromosomes::Chromosomes(const Solution &soln)
    : vrp(soln.vrp)
{
    seq_.reserve(vrp.get().locations.size() - 1);
    for (const auto &v : soln.vehicles)
    {
        const auto route = v.route();
        seq_.insert(seq_.end(), route.begin() + 1, route.end() - 1);
        fitness_ += v.cost();
    }
}

#define INF (std::numeric_limits<double>::infinity())

namespace
{
    VehicleMeta select_type(const std::vector<VehicleMeta> &types, int load)
    {
        for (auto meta : types)
        {
            if (load <= meta.capacity)
                return meta;
        }
        return VehicleMeta{0, 0};
    }

    struct SplitResult
    {
        std::vector<double> v;
        std::vector<VehicleMeta> t;
        std::vector<size_t> p;
    };

    SplitResult split(const VRP &vrp, const std::vector<size_t> &seq)
    {
        Solution soln(vrp);

        auto types = vrp.vehicle_types;
        std::sort(types.begin(), types.end(), [](VehicleMeta a, VehicleMeta b)
                  { return a.cost < b.cost; });

        size_t n = seq.size();
        std::vector<double> v(n, INF); // from 0 to j (inclusive)
        std::vector<VehicleMeta> t(n); // meta
        std::vector<size_t> p(n);      // current begin (inclusive), previous end (exclusive).
        // update v[j] in each loop
        for (size_t i = 0; i < n; ++i)
        {
            double base = 0;
            if (i != 0)
                base = v[i - 1];
            int load = 0;
            double cost = 0.0;
            for (size_t j = i; j < n; ++j)
            {
                load += vrp.locations[seq[j]].demand;
                auto meta = select_type(types, load);
                if (meta.capacity == 0)
                    break;
                if (j == i)
                    cost += vrp.distance(0, seq[j]) + vrp.distance(seq[j], 0);
                else
                    cost += vrp.distance(seq[j - 1], seq[j]) + vrp.distance(seq[j], 0) - vrp.distance(seq[j - 1], 0);
                double total = base + cost + meta.cost;
                if (total < v[j])
                {
                    v[j] = total;
                    t[j] = meta;
                    p[j] = i;
                }
            }
        }
        return SplitResult{std::move(v), std::move(t), std::move(p)};
    }
}

Chromosomes::Chromosomes(const VRP &vrp, const std::vector<size_t> &seq)
    : vrp(vrp), seq_(seq)
{
    auto result = split(vrp, seq_);
    fitness_ = result.v.back();
}

Chromosomes::operator Solution() const
{
    const auto &vrp = this->vrp.get();
    Solution soln(vrp);

    auto result = split(vrp, seq_);
    const auto &t = result.t;
    const auto &p = result.p;
    for (size_t i = p.size() - 1;;)
    {
        size_t b = p[i];
        Vehicle v(vrp, t[i]);
        v.update_route([this, i, b](std::vector<size_t> &r)
                       {
                           auto begin = this->seq_.begin();
                           r.insert(r.begin() + 1, begin + b, begin + i + 1);
                       });
        soln.vehicles.push_back(std::move(v));
        if (b == 0)
            break;
        i = b - 1;
    }
    return soln;
}