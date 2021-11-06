#include "heuristic.h"

#include <algorithm>
#include <vector>
#include <list>
#include <functional>

namespace
{
    class SavingEntry
    {
        friend bool operator<(const SavingEntry &a, const SavingEntry &b)
        {
            return a.saving < b.saving;
        }

    public:
        std::reference_wrapper<const Vehicle> a;
        std::reference_wrapper<const Vehicle> b;
        double saving;

        SavingEntry(const VRP &vrp, const Vehicle &a, const Vehicle &b)
            : a(a), b(b), saving(compute_saving(vrp, a, b)) {}

    private:
        double compute_saving(const VRP &vrp, const Vehicle &a, const Vehicle &b)
        {
            const auto &ar = a.route();
            const auto &br = b.route();
            size_t ia = ar[ar.size() - 2];
            size_t ib = br[1];
            return vrp.distance(ia, 0) + vrp.distance(0, ib) - vrp.distance(ia, ib);
        }
    };

    Solution saving_heuristic(const VRP &vrp, VehicleMeta meta)
    {
        std::list<Vehicle> vehicles;
        const auto &locations = vrp.locations;
        for (size_t i = 1; i < locations.size(); ++i)
        {
            vehicles.emplace_front(vrp, meta);
            vehicles.front().update_route(
                [i](std::vector<size_t> &r)
                { r.insert(r.begin() + 1, i); });
        }
        std::vector<SavingEntry> saving_table;
        for (auto it = vehicles.begin(); it != vehicles.end(); ++it)
        {
            const auto &a = *it;
            for (auto jt = std::next(it); jt != vehicles.end(); ++jt)
            {
                const auto &b = *jt;
                if (a.load() + b.load() > meta.capacity)
                    continue;
                saving_table.push_back(SavingEntry(vrp, a, b));
            }
        }
        std::sort(saving_table.begin(), saving_table.end());
        while (!saving_table.empty())
        {
            auto entry = saving_table.back();
            auto x = &entry.a.get();
            auto y = &entry.b.get();
            saving_table.erase(
                std::remove_if(saving_table.begin(), saving_table.end(),
                               [x, y](const SavingEntry &e)
                               {
                                   auto a = &e.a.get();
                                   auto b = &e.b.get();
                                   return a == x || a == y || b == x || b == y;
                               }),
                saving_table.end());
            Vehicle merged(vrp, meta);
            merged.update_route([x, y](std::vector<size_t> &r)
                                {
                                    const auto &xr = x->route();
                                    const auto &yr = y->route();
                                    r.insert(r.begin() + 1, xr.begin() + 1, xr.end() - 1);
                                    r.insert(r.end() - 1, yr.begin() + 1, yr.end() - 1);
                                });
            vehicles.remove_if([x, y](const Vehicle &v)
                               { return &v == x || &v == y; });
            vehicles.push_front(std::move(merged));
            for (auto it = ++vehicles.begin(); it != vehicles.end(); ++it)
            {
                const auto &a = vehicles.front();
                const auto &b = *it;
                if (a.load() + b.load() > meta.capacity)
                    continue;
                saving_table.push_back(SavingEntry(vrp, a, b));
                saving_table.push_back(SavingEntry(vrp, b, a));
            }
            std::sort(saving_table.begin(), saving_table.end());
        }
        Solution soln = Solution(vrp);
        soln.vehicles.reserve(vehicles.size());
        std::move(vehicles.begin(), vehicles.end(), std::back_inserter(soln.vehicles));
        return soln;
    }
}

namespace
{
    void replace_if_better(VehicleMeta meta, Solution &soln)
    {
        const auto &vrp = soln.vrp.get();
        auto &vehicles = soln.vehicles;
        for (size_t i = 0; i < vehicles.size();)
        {
            const auto &c = vehicles[i];
            if (c.meta().capacity == meta.capacity)
                break;
            if (c.load() <= meta.capacity)
            {
                Vehicle v(vrp, meta);
                v.update_route([c](std::vector<size_t> &r)
                               { r = c.route(); });
                vehicles.erase(vehicles.begin() + i);
                vehicles.push_back(v);
                continue;
            }
            ++i;
        }
    }
}

Solution heuristic(const VRP &vrp)
{
    auto types = vrp.vehicle_types;
    std::sort(types.begin(), types.end(), [](VehicleMeta a, VehicleMeta b)
              { return a.capacity < b.capacity; });
    auto soln = saving_heuristic(vrp, types.back());
    types.pop_back();
    for (VehicleMeta meta : types)
    {
        replace_if_better(meta, soln);
    }
    return soln;
}
