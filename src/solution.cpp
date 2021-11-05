#include "solution.h"

std::ostream &operator<<(std::ostream &os, const Solution &soln)
{
    const auto &vrp = soln.vrp.get();
    os << "------\n";
    os << "Solution to VRP with " << vrp.locations.size() - 1 << " client locations and "
       << vrp.vehicle_types.size() << " vehicle types:" << '\n';
    const auto &vehicles = soln.vehicles;
    for (size_t i = 0; i < vehicles.size(); i++)
    {
        os << "Vehicle #" << (i + 1) << ' ' << vehicles[i] << '\n';
    }
    os << "------\n";
    return os;
}
