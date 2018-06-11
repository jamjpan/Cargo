// MIT License
//
// Copyright (c) 2018 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <algorithm> /* std::iter_swap */
#include <iostream> /* debug */
#include <iterator>

#include "libcargo/cargo.h" /* static gtree() */
#include "libcargo/distance.h"
#include "libcargo/functions.h"
#include "libcargo/classes.h"
#include "libcargo/types.h"
#include "gtree/gtree.h"

namespace cargo {

// Complexity: O(|schedule|*|route|)
//   - for loop body executes n-1 times, for n stops
//   - std::copy is exactly |route| operations
//   - assume find_path, shortest_path_dist are O(1) with gtree
DistanceInt route_through(const Schedule& s, std::vector<Waypoint>& r)
{
    DistanceInt cost = 0;
    r.clear();
    r.push_back({0, s.data().front().location()});
    for (ScheduleIndex i = 0; i < s.data().size()-1; ++i) {
        std::vector<NodeId> seg;
        NodeId from = s.data().at(i).location();
        NodeId to = s.data().at(i+1).location();
        Cargo::gtree().find_path(from, to, seg);
        for (size_t i = 1; i < seg.size(); ++i) {
            cost += shortest_path_dist(seg.at(i-1), seg.at(i));
            r.push_back({cost, seg.at(i)});
        }
    }
    return cost;
}

DistanceInt route_through(const std::vector<Stop>& s, std::vector<Waypoint>& r)
{
    Schedule schedule(-1, s); // give it a dummy owner
    return route_through(schedule, r);
}

bool check_precedence_constr(const Schedule& s)
{
    // Last stop must be this vehicle destination
    if (s.data().back().type() != StopType::VehicleDest
            || s.data().back().owner() != s.owner()) {
        // std::cout << "last stop is not a vehicle dest, or last stop owner mismatch\n";
        return false;
    }

    // Second-to-last stop cannot be any origin if schedule size > 2
    if (s.data().size() > 2
            && (std::prev(s.data().end(), 2)->type() == StopType::CustomerOrigin
            || std::prev(s.data().end(), 2)->type() == StopType::VehicleOrigin)) {
        // std::cout << "second-to-last stop is an origin\n";
        return false;
    }

    // In the worst case, this algorithm walks the schedule for each stop in
    // order to find the paired requests. The complexity is O(|schedule|^2).
    bool paired;
    for (auto i = s.data().begin(); i != s.data().end(); ++i) {
        // Any vehicle origin cannot appear in the schedule, aside from at 0
        if (i > s.data().begin() && i->type() == StopType::VehicleOrigin) {
            // std::cout << "vehicle origin appears not at 0\n";
            return false;
        }

        // A vehicle origin that is not this vehicle cannot appear at 0
        if (i == s.data().begin()
                && i->type() == StopType::VehicleOrigin
                && i->owner() != s.owner()) {
            // std::cout << "vehicle origin mismatch\n";
            return false;
        }

        paired = false;
        for (auto j = s.data().begin(); j != s.data().end() && !paired; ++j) {
            if (i == j)
                continue;
            if (i->owner() == j->owner()) {
                if (i->type() == StopType::CustomerOrigin
                        && j->type() == StopType::CustomerDest
                        && i < j)
                    paired = true;
                else if (i->type() == StopType::CustomerOrigin
                        && j->type() == StopType::CustomerDest
                        && i > j) {
                    // std::cout << "cust origin appears after dest\n";
                    return false;
                }
                else if (i->type() == StopType::CustomerDest
                        && j->type() == StopType::CustomerOrigin
                        && i > j)
                    paired = true;
                else if (i->type() == StopType::CustomerDest
                        && j->type() == StopType::CustomerOrigin
                        && i < j) {
                    // std::cout << "cust dest appears before origin\n";
                    return false;
                }
                else if (i->type() == StopType::VehicleOrigin
                        && j->type() == StopType::VehicleDest
                        && i < j)
                    paired = true;
                else if (i->type() == StopType::VehicleOrigin
                        && j->type() == StopType::VehicleDest
                        && i > j) {
                    // std::cout << "veh origin appears after dest\n";
                    return false;
                }
                else if (i->type() == StopType::VehicleDest
                        && j->type() == StopType::VehicleOrigin
                        && i > j)
                    paired = true;
                else if (i->type() == StopType::VehicleDest
                        && j->type() == StopType::VehicleOrigin
                        && i < j) {
                    // std::cout << "veh dest appears before origin\n";
                    return false;
                }
            }
        }
        if (!paired
                && (i->type() != StopType::CustomerDest
                && i->type() != StopType::VehicleDest)) {
            // std::cout << "origin is unpaired\n";
            return false;
        }
    }
    return true;
}

bool check_timewindow_constr(const Schedule& s, const Route& r)
{
    // Check the end point first
    if (s.data().back().late() < r.data().back().first)
        return false;

    // Walk along the schedule and the route.
    // Total complexity is O(|schedule| + |route|)
    auto j = r.data().begin();
    for (auto i = s.data().begin(); i != s.data().end(); ++i) {
        while (j->second != i->location())
            ++j;
        if (i->late() < j->first/(float)Cargo::vspeed())
            return false;
    }
    return true;
}
bool check_timewindow_constr(const std::vector<Stop>& s, const std::vector<Waypoint>& r)
{
    Schedule sch(-1, s);
    Route route(-1, r);
    return check_timewindow_constr(sch, route);
}

DistanceInt sop_insert(const Vehicle& veh, const Customer& cust,
        std::vector<Stop>& best_schedule, std::vector<Waypoint>& best_route)
{
    return sop_insert(veh, cust, true, true, best_schedule, best_route);
}
DistanceInt sop_insert(const Vehicle& veh, const Customer& cust, bool fix_start,
        bool fix_end, std::vector<Stop>& best_schedule, std::vector<Waypoint>& best_route)
{
    DistanceInt best_cost = InfinityInt;
    best_schedule.clear();
    best_route.clear();

    std::vector<Stop> schedule = veh.schedule().data(); // copy
    std::vector<Waypoint> route;

    // The dist to next node in veh's route matches the first stop in veh.schedule()
    // (the first stop is always the next node, see cargo::step())
    // DistanceInt curr_traveled = veh.route().dist_at(veh.idx_last_visited_node()+1);

    auto check_best = [&](DistanceInt cost) {
        if (cost < best_cost) {
            best_cost = cost;
            best_schedule = schedule;
            best_route = route;
        }
    };

    // For debugging
    // auto print_schedule = [&](std::vector<Stop> schedule) {
    //     for (const auto& stop : schedule)
    //         std::cout << stop.location() << ", ";
    //     std::cout << std::endl;
    // };

    Stop cust_o(cust.id(), cust.origin(), StopType::CustomerOrigin, cust.early(), cust.late());
    Stop cust_d(cust.id(), cust.destination(), StopType::CustomerDest, cust.early(), cust.late());
    schedule.insert(schedule.begin()+fix_start, cust_o);
    schedule.insert(schedule.begin()+fix_start, cust_d);

    // This algorithm uses a series of swaps to generate all insertion
    // combinations. Uncomment print_schedule to view the combinations.
    // Here is an example of inserting customer (A, B) into a 3-stop sched:
    // A B - - -
    // A - B - -
    // A - - B -
    // A - - - B
    // - A - - B
    // - A - B -
    // - A B - -
    // - - A B -
    // - - A - B
    // - - - A B
    int inc = 1;
    bool reset = false;
    std::vector<Stop>::iterator start, end;
    for (auto i = schedule.begin()+fix_start; i != schedule.end()-1-fix_end; ++i) {
        start = (inc == 1) ? i : schedule.end()-1-fix_end;
        end = (inc == 1) ? schedule.end()-1-fix_end : i+1;
        for (auto j = start; j != end; j+=inc) {
            if (reset) {
                std::iter_swap(i-1, i+1); // <-- O(1)
                reset = false;
            } else
                std::iter_swap(j, j+inc);
            // print_schedule(schedule)
            check_best(route_through(schedule, route)/*+curr_traveled*/);
        }
        std::iter_swap(i, i+1);
        if (inc == 1 && i < schedule.end()-2-fix_end) {
            // print_schedule(schedule)
            check_best(route_through(schedule, route)/*+curr_traveled*/);
        }
        inc = -inc;
        if (inc == 1)
            reset = true;
    }

    // Add curr_traveled to the new nodes in the route
    // for (auto& wp : best_route)
    //    wp.first += curr_traveled;

    return best_cost;
}

} // namespace cargo

