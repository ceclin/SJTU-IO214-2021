#pragma once

#include <iostream>

struct Location
{
    friend std::ostream &operator<<(std::ostream &os, const Location &lo);
    static double distance(const Location &a, const Location &b);
    int x;
    int y;
    int demand;
};
