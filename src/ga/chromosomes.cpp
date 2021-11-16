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

namespace
{
    const VehicleMeta *select_type(const std::vector<VehicleMeta> &types, int load)
    {
        for (const auto &meta : types)
        {
            if (load <= meta.capacity)
                return &meta;
        }
        return nullptr;
    }
}

#define INF (std::numeric_limits<double>::infinity())

Chromosomes::operator Solution() const
{
    const auto &vrp = this->vrp.get();
    Solution soln(vrp);

    auto types = vrp.vehicle_types;
    std::sort(types.begin(), types.end(), [](VehicleMeta a, VehicleMeta b)
              { return a.cost < b.cost; });

    size_t n = seq_.size();
    std::vector<double> v(n, INF);         // from 0 to j (inclusive)
    std::vector<const VehicleMeta *> t(n); // meta
    std::vector<size_t> p(n);              // current begin (inclusive), previous end (exclusive).
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
            load += vrp.locations[seq_[j]].demand;
            const auto *meta = select_type(types, load);
            if (meta == nullptr)
                break;
            if (j == i)
                cost += vrp.distance(0, seq_[j]) + vrp.distance(seq_[j], 0);
            else
                cost += vrp.distance(seq_[j - 1], seq_[j]) + vrp.distance(seq_[j], 0) - vrp.distance(seq_[j - 1], 0);
            double total = base + cost + meta->cost;
            if (total < v[j])
            {
                v[j] = total;
                t[j] = meta;
                p[j] = i;
            }
        }
    }
    for (size_t i = n - 1;;)
    {
        size_t b = p[i];
        Vehicle v(vrp, *t[i]);
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