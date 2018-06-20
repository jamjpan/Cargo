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
#include <memory> /* shared_ptr */

#include "libcargo/cargo.h" /* static gtree() */
#include "libcargo/distance.h"
#include "libcargo/functions.h"
#include "libcargo/classes.h"
#include "libcargo/types.h"
#include "gtree/gtree.h"

namespace cargo {

DistanceInt pickup_range(const Customer& cust, const SimTime now)
{
    return (cust.late()-(shortest_path_dist(cust.origin(), cust.destination())
                /Cargo::vspeed())-now)*Cargo::vspeed();
}

// Complexity: O(|schedule|*|route|)
//   - for loop body executes n-1 times, for n stops
//   - std::copy is exactly |route| operations
//   - assume find_path, shortest_path_dist are O(1) with gtree
DistanceInt route_through(const Schedule& sch, std::vector<Waypoint>& rteout)
{
    return route_through(sch.data(), rteout);
}

DistanceInt route_through(const std::vector<Stop>& sch,
        std::vector<Waypoint>& rteout)
{
    DistanceInt cost = 0;
    rteout.clear();
    rteout.push_back({0, sch.front().location()});
    for (ScheduleIndex i = 0; i < sch.size()-1; ++i) {
        std::vector<NodeId> seg;
        NodeId from = sch.at(i).location();
        NodeId to = sch.at(i+1).location();
        Cargo::gtree().find_path(from, to, seg);
        for (size_t i = 1; i < seg.size(); ++i) {
            cost += Cargo::edgeweight(seg.at(i-1), seg.at(i));
            rteout.push_back({cost, seg.at(i)});
        }
    }
    return cost;
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

bool check_timewindow_constr(const Schedule& sch, const Route& rte)
{
    return check_timewindow_constr(sch.data(), rte.data());
}

bool check_timewindow_constr(const std::vector<Stop>& sch,
        const std::vector<Waypoint>& rte)
{
    // Check the end point first
    if (sch.back().late() < rte.back().first/(float)Cargo::vspeed())
        return false;

    // Walk along the schedule and the route. O(|schedule|+|route|)
    auto j = rte.begin();
    for (auto i = sch.begin(); i != sch.end(); ++i) {
        while (j->second != i->location())
            ++j;
        if (i->late() < j->first/(float)Cargo::vspeed())
            return false;
    }
    return true;
}

DistanceInt sop_insert(const std::vector<Stop>& sch, const Stop& origin, const Stop& destnn,
        bool fix_start, bool fix_end, std::vector<Stop>& schout, std::vector<Waypoint>& rteout)
{
    DistanceInt mincst = InfinityInt;
    schout.clear();
    rteout.clear();

    std::vector<Stop> mutsch = sch; // mutable schedule
    std::vector<Waypoint> mutrte;   // mutable route

    auto check = [&](DistanceInt cost) {
        if (cost < mincst) {
            mincst = cost;
            schout = mutsch;
            rteout = mutrte;
        }
    };

    mutsch.insert(mutsch.begin()+fix_start, origin);
    mutsch.insert(mutsch.begin()+fix_start, destnn);

    // This algorithm uses a series of swaps to generate all insertion
    // combinations.  Here is an example of inserting stops (A, B) into a
    // 3-stop sched:
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
    bool rst = false;
    std::vector<Stop>::iterator bgn, end;
    for (auto i = mutsch.begin()+fix_start; i != mutsch.end()-1-fix_end; ++i) {
        bgn = (inc == 1) ? i : mutsch.end()-1-fix_end;
        end = (inc == 1) ? mutsch.end()-1-fix_end : i+1;
        for (auto j = bgn; j != end; j += inc) {
            if (rst) {
                std::iter_swap(i-1, i+1); // <-- O(1)
                rst = false;
            } else
                std::iter_swap(j, j+inc);
            check(route_through(mutsch, mutrte));
        }
        std::iter_swap(i, i+1);
        if (inc == 1 && i < mutsch.end()-2-fix_end)
            check(route_through(mutsch, mutrte));
        if ((inc = -inc) == 1)
            rst = true;
    }

    return mincst;
}

DistanceInt sop_insert(const Vehicle& veh, const Customer& cust,
        std::vector<Stop>& schout, std::vector<Waypoint>& rteout)
{
    // The distances to the nodes in the routes found by route_through need
    // to be corrected.  veh.schedule() passed here contains only un-visited
    // stops. The first stop in the schedule is the vehicle's next node
    // (because of step()).  route_through will give this stop a distance of 0.
    // The distances to other stops in the augmented schedule passed to
    // route_through will be relative to this first stop. The already-traveled
    // distance (the head) should be added.
    DistanceInt head = veh.route().dist_at(veh.idx_last_visited_node()+1);

    Stop cust_o(cust.id(), cust.origin(), StopType::CustomerOrigin, cust.early(), cust.late());
    Stop cust_d(cust.id(), cust.destination(), StopType::CustomerDest, cust.early(), cust.late());
    DistanceInt mincst = sop_insert(veh.schedule().data(), cust_o, cust_d,
            true, true, schout, rteout);

    // Add head to the new nodes in the route
    for (auto& wp : rteout)
        wp.first += head;
    rteout.insert(rteout.begin(), veh.route().at(veh.idx_last_visited_node()));

    return mincst;
}

DistanceInt sop_insert(const std::shared_ptr<MutableVehicle>& mutveh, const Customer& cust,
        std::vector<Stop>& schout, std::vector<Waypoint>& rteout)
{
    return sop_insert(*mutveh, cust, schout, rteout);
}


} // namespace cargo

