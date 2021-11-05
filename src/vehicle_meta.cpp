#include "vehicle_meta.h"

std::ostream &operator<<(std::ostream &os, const VehicleMeta &meta)
{
    return os << "(capacity=" << meta.capacity << ", cost=" << meta.cost << ")";
}
