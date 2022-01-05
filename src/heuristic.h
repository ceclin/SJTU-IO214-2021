#pragma once

#include "vrp.h"
#include "solution.h"
#include "vehicle_meta.h"

Solution heuristic(const VRP &vrp);

Solution heuristic(const VRP &vrp, VehicleMeta meta);
