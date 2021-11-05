#include "vehicle.h"

std::ostream &operator<<(std::ostream &os, const Vehicle &v)
{
    const auto &vrp = v.vrp.get();
    os << v.meta_ << ": ";
    auto it = v.route_.begin();
    int load = vrp.locations[*it].demand;
    double cost = v.meta_.cost;
    os << '#' << *it << '[' << load << '/' << v.meta_.capacity << ',' << cost << ']';
    size_t prev = *it;
    while (++it != v.route_.end())
    {
        load += vrp.locations[*it].demand;
        cost += vrp.distance(prev, *it);
        os << " -> ";
        os << '#' << *it << '[' << load << '/' << v.meta_.capacity << ',' << cost << ']';
        prev = *it;
    }
    return os;
}

void Vehicle::update_route(std::function<void(std::vector<size_t> &)> f)
{
    f(route_);
    update();
}

void Vehicle::update()
{
    const auto &vrp = this->vrp.get();
    int load = 0;
    double cost = meta_.cost;
    for (size_t i = 1; i < route_.size(); ++i)
    {
        load += vrp.locations[route_[i]].demand;
        cost += vrp.distance(route_[i - 1], route_[i]);
    }
    load_ = load;
    cost_ = cost;
}
