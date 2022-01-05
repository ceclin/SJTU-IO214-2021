#include "mutation.h"

#include <algorithm>

namespace
{
    // if (&a == &b && i == j)
    //     return false;
    // const auto &p = a.route();
    // const auto &q = b.route();
    // size_t u = p[i], v = q[j];

    // x - y > 0.0005
    // &a == &b in some cases

    bool move1(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j)
    {
        // return false;
        if (&a == &b && (i == j || i == j + 1))
            return false;
        const auto &p = a.route();
        const auto &q = b.route();
        size_t u = p[i], v = q[j];
        if (u == 0 || j == q.size() - 1)
            return false;
        if (b.load() + vrp.locations[u].demand > b.meta().capacity)
            return false;
        double x = vrp.distance(p[i - 1], p[i]) + vrp.distance(p[i], p[i + 1]) +
                   vrp.distance(q[j], q[j + 1]);
        double y = vrp.distance(p[i - 1], p[i + 1]) +
                   vrp.distance(q[j], p[i]) + vrp.distance(p[i], q[j + 1]);
        if (x - y >= 0.0005)
        {
            a.update_route([i](std::vector<size_t> &r)
                           { r.erase(r.begin() + i); });
            b.update_route([j, u, v](std::vector<size_t> &r)
                           { r.insert(r.begin() + (r[j] == v ? j + 1 : j), u); });
            return true;
        }
        return false;
    }

    bool move2(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j)
    {
        // return false;
        if (&a == &b && (i == j || i == j + 1 || j == i + 1))
            return false;
        const auto &p = a.route();
        const auto &q = b.route();
        size_t u = p[i], v = q[j];
        if (u == 0 || j == q.size() - 1 || i == p.size() - 2)
            return false;
        if (b.load() + vrp.locations[p[i]].demand + vrp.locations[p[i + 1]].demand > b.meta().capacity)
            return false;
        double x = vrp.distance(p[i - 1], p[i]) + vrp.distance(p[i + 1], p[i + 2]) +
                   vrp.distance(q[j], q[j + 1]);
        double y = vrp.distance(p[i - 1], p[i + 2]) +
                   vrp.distance(q[j], p[i]) + vrp.distance(p[i + 1], q[j + 1]);
        if (x - y >= 0.0005)
        {
            size_t m = p[i + 1];
            a.update_route([i](std::vector<size_t> &r)
                           { r.erase(r.begin() + i, r.begin() + i + 2); });
            b.update_route([j, u, v, m](std::vector<size_t> &r)
                           { r.insert(++std::find(r.begin(), r.end(), v), {u, m}); });
            return true;
        }
        return false;
    }

    bool move3(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j)
    {
        // return false;
        if (&a == &b && (i == j || i == j + 1 || j == i + 1))
            return false;
        const auto &p = a.route();
        const auto &q = b.route();
        size_t u = p[i], v = q[j];
        if (u == 0 || j == q.size() - 1 || i == p.size() - 2)
            return false;
        if (b.load() + vrp.locations[p[i]].demand + vrp.locations[p[i + 1]].demand > b.meta().capacity)
            return false;
        double x = vrp.distance(p[i - 1], p[i]) + vrp.distance(p[i + 1], p[i + 2]) +
                   vrp.distance(q[j], q[j + 1]);
        double y = vrp.distance(p[i - 1], p[i + 2]) +
                   vrp.distance(q[j], p[i + 1]) + vrp.distance(p[i], q[j + 1]);
        if (x - y >= 0.0005)
        {
            size_t m = p[i + 1];
            a.update_route([i](std::vector<size_t> &r)
                           { r.erase(r.begin() + i, r.begin() + i + 2); });
            b.update_route([j, u, v, m](std::vector<size_t> &r)
                           { r.insert(++std::find(r.begin(), r.end(), v), {m, u}); });
            return true;
        }
        return false;
    }

    bool move4(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j)
    {
        // return false;
        if (&a == &b && i == j)
            return false;
        const auto &p = a.route();
        const auto &q = b.route();
        size_t u = p[i], v = q[j];
        if (u == 0 || v == 0)
            return false;
        if (&a == &b)
        {
            bool ok = false;
            if (j == i + 1)
                std::swap(i, j);
            if (i == j + 1)
            {
                double x = vrp.distance(p[i], p[i + 1]) + vrp.distance(p[j - 1], p[j]);
                double y = vrp.distance(p[j - 1], p[i]) + vrp.distance(p[j], p[i + 1]);
                ok = x - y > 0.0005;
            }
            else
            {
                double x = vrp.distance(p[i - 1], p[i]) + vrp.distance(p[i], p[i + 1]) +
                           vrp.distance(p[j - 1], p[j]) + vrp.distance(p[j], p[j + 1]);
                double y = vrp.distance(p[i - 1], p[j]) + vrp.distance(p[j], p[i + 1]) +
                           vrp.distance(p[j - 1], p[i]) + vrp.distance(p[i], p[j + 1]);
                ok = x - y > 0.0005;
            }
            if (ok)
            {
                a.update_route([i, j](std::vector<size_t> &r)
                               { std::swap(r[i], r[j]); });
                return true;
            }
        }
        else
        {
            int du = vrp.locations[u].demand;
            int dv = vrp.locations[v].demand;
            if (a.load() - du + dv > a.meta().capacity)
                return false;
            if (b.load() - dv + du > b.meta().capacity)
                return false;
            double x = vrp.distance(p[i - 1], p[i]) + vrp.distance(p[i], p[i + 1]) +
                       vrp.distance(q[j - 1], q[j]) + vrp.distance(q[j], q[j + 1]);
            double y = vrp.distance(p[i - 1], q[j]) + vrp.distance(q[j], p[i + 1]) +
                       vrp.distance(q[j - 1], p[i]) + vrp.distance(p[i], q[j + 1]);
            if (x - y > 0.0005)
            {
                a.update_route([&b, i, j](std::vector<size_t> &ra)
                               { b.update_route([&ra, i, j](std::vector<size_t> &rb)
                                                { std::swap(ra[i], rb[j]); }); });
                return true;
            }
        }
        return false;
    }

    bool move5(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j)
    {
        // return false;
        if (&a == &b && (i == j || j == i + 1))
            return false;
        const auto &p = a.route();
        const auto &q = b.route();
        size_t u = p[i], v = q[j];
        if (u == 0 || v == 0 || i == p.size() - 2)
            return false;
        size_t m = p[i + 1];
        if (&a == &b)
        {
            // a little trade-off
            Vehicle t = a;
            t.update_route([i, j, u, m](std::vector<size_t> &r)
                           {
                               std::swap(r[i], r[j]);
                               r.erase(r.begin() + i + 1);
                               r.insert(std::find(r.begin(), r.end(), u) + 1, m);
                           });
            if (a.cost() - t.cost() > 0.0005)
            {
                a = std::move(t);
                return true;
            }
        }
        else
        {
            int du = vrp.locations[u].demand;
            int dv = vrp.locations[v].demand;
            int dm = vrp.locations[m].demand;
            if (a.load() - du - dm + dv > a.meta().capacity)
                return false;
            if (b.load() - dv + du + dm > b.meta().capacity)
                return false;
            double x = vrp.distance(p[i - 1], p[i]) + vrp.distance(p[i + 1], p[i + 2]) +
                       vrp.distance(q[j - 1], q[j]) + vrp.distance(q[j], q[j + 1]);
            double y = vrp.distance(p[i - 1], q[j]) + vrp.distance(q[j], p[i + 2]) +
                       vrp.distance(q[j - 1], p[i]) + vrp.distance(p[i + 1], q[j + 1]);
            if (x - y > 0.0005)
            {
                a.update_route([i, v](std::vector<size_t> &r)
                               {
                                   r[i] = v;
                                   r.erase(r.begin() + i + 1);
                               });
                b.update_route([j, u, m](std::vector<size_t> &r)
                               {
                                   r[j] = u;
                                   r.insert(r.begin() + j + 1, m);
                               });
                return true;
            }
        }
        return false;
    }

    bool move6(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j)
    {
        // return false;
        if (&a == &b && (i == j || j == i + 1 || i == j + 1))
            return false;
        const auto &p = a.route();
        const auto &q = b.route();
        size_t u = p[i], v = q[j];
        if (u == 0 || v == 0 || i == p.size() - 2 || j == q.size() - 2)
            return false;
        size_t m = p[i + 1], n = q[j + 1];
        if (&a == &b)
        {
            // a little trade-off
            Vehicle t = a;
            t.update_route([i, j](std::vector<size_t> &r)
                           {
                               std::swap(r[i], r[j]);
                               std::swap(r[i + 1], r[j + 1]);
                           });
            if (a.cost() - t.cost() > 0.0005)
            {
                a = std::move(t);
                return true;
            }
        }
        else
        {
            int du = vrp.locations[u].demand;
            int dv = vrp.locations[v].demand;
            int dm = vrp.locations[m].demand;
            int dn = vrp.locations[n].demand;
            if (a.load() - du - dm + dv + dn > a.meta().capacity)
                return false;
            if (b.load() - dv - dn + du + dm > b.meta().capacity)
                return false;
            double x = vrp.distance(p[i - 1], p[i]) + vrp.distance(p[i + 1], p[i + 2]) +
                       vrp.distance(q[j - 1], q[j]) + vrp.distance(q[j + 1], q[j + 2]);
            double y = vrp.distance(p[i - 1], q[j]) + vrp.distance(q[j + 1], p[i + 2]) +
                       vrp.distance(q[j - 1], p[i]) + vrp.distance(p[i + 1], q[j + 2]);
            if (x - y > 0.0005)
            {
                a.update_route([&b, i, j](std::vector<size_t> &ra)
                               {
                                   b.update_route([&ra, i, j](std::vector<size_t> &rb)
                                                  {
                                                      std::swap(ra[i], rb[j]);
                                                      std::swap(ra[i + 1], rb[j + 1]);
                                                  });
                               });
                return true;
            }
        }
        return false;
    }

    // It just swaps x and v, which is handled by move4.
    constexpr bool move7(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j) { return false; }

    // It just swaps x and v, which is handled by move4.
    constexpr bool move8(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j) { return false; }

    bool move9(const VRP &vrp, Vehicle &a, Vehicle &b, size_t i, size_t j)
    {
        // return false;
        if (&a == &b)
            return false;
        const auto &p = a.route();
        const auto &q = b.route();
        if (i == p.size() - 1 || i == p.size() - 2 || q[j] == 0 || j == q.size() - 2)
            return false;
        double x = vrp.distance(p[i], p[i + 1]) + vrp.distance(p[i + 1], p[i + 2]) +
                   vrp.distance(q[j - 1], q[j]) + vrp.distance(q[j], q[j + 1]) + vrp.distance(q[j + 1], q[j + 2]);
        double y = vrp.distance(p[i], q[j + 1]) + vrp.distance(q[j + 1], p[i + 2]) +
                   vrp.distance(q[j - 1], p[i + 1]) + vrp.distance(p[i + 1], q[j]) + vrp.distance(q[j], q[j + 2]);
        if (x - y > 0.0005)
        {
            a.update_route([&b, i, j](std::vector<size_t> &ra)
                           {
                               b.update_route([&ra, i, j](std::vector<size_t> &rb)
                                              {
                                                  std::swap(ra[i + 1], rb[j + 1]);
                                                  std::swap(rb[j], rb[j + 1]);
                                              });
                           });
        }
        return false;
    }
}

Chromosomes mutation_with_local_search(const Chromosomes &c)
{
    const auto &vrp = c.vrp.get();
    Solution soln = c;
    std::vector<Vehicle> &vehicles = soln.vehicles;
loop:
    for (auto &&a : vehicles)
        for (auto &&b : vehicles)
            for (size_t i = 0; i < a.route().size(); ++i)
                for (size_t j = 0; j < b.route().size(); ++j)
                    if (false ||
                        move1(vrp, a, b, i, j) ||
                        move2(vrp, a, b, i, j) ||
                        move3(vrp, a, b, i, j) ||
                        move4(vrp, a, b, i, j) ||
                        move5(vrp, a, b, i, j) ||
                        move6(vrp, a, b, i, j) ||
                        move7(vrp, a, b, i, j) ||
                        move8(vrp, a, b, i, j) ||
                        move9(vrp, a, b, i, j) ||
                        false)
                        goto loop;
    return Chromosomes(soln);
}