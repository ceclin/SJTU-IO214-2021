#pragma once

#include <functional>
#include <vector>
#include <list>
#include <iostream>

#include "vrp.h"
#include "vehicle_meta.h"

class Vehicle
{
    friend std::ostream &operator<<(std::ostream &os, const Vehicle &v);

public:
    std::reference_wrapper<const VRP> vrp;

    Vehicle(const VRP &vrp, const VehicleMeta meta) : vrp(vrp), meta_(meta), route_{0, 0} { update(); }

    void update_route(std::function<void(std::vector<size_t> &)> f);

    inline VehicleMeta meta() const { return meta_; }
    inline const std::vector<size_t> &route() const { return route_; }
    inline int load() const { return load_; }
    inline double cost() const { return cost_; }

private:
    VehicleMeta meta_;
    std::vector<size_t> route_;
    int load_;
    double cost_;
    void update();
};
