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
#include <algorithm> /* iter_swap */
#include <iostream>  /* debug */
#include <iterator>
#include <memory> /* shared_ptr */

#include "libcargo/cargo.h" /* Cargo::gtree() */
#include "libcargo/classes.h"
#include "libcargo/distance.h"
#include "libcargo/functions.h"
#include "libcargo/types.h"

#include "gtree/gtree.h"

namespace cargo {

DistInt pickup_range(const Customer& cust, const SimlTime& now,
                     GTree::G_Tree& gtree) {
  DistInt dist = shortest_path_dist(cust.orig(), cust.dest(), gtree);
  return (cust.late()-(dist/Cargo::vspeed())-now)*Cargo::vspeed();
}

DistInt pickup_range(const Customer& cust, const SimlTime& now) {
    return pickup_range(cust, now, Cargo::gtree());
}

// Complexity: O(|schedule|*|route|)
//   - for loop body executes n-1 times, for n stops
//   - std::copy is exactly |route| operations
//   - assume find_path, shortest_path_dist are O(1) with gtree
DistInt route_through(const std::vector<Stop>& sch, std::vector<Wayp>& rteout, GTree::G_Tree& gtree) {
  DistInt cst = 0;
  rteout.clear();
  rteout.push_back({0, sch.front().loc()});
  for (SchIdx i = 0; i < sch.size() - 1; ++i) {
    std::vector<NodeId> seg;
    gtree.find_path(sch.at(i).loc(), sch.at(i + 1).loc(), seg);
    for (size_t i = 1; i < seg.size(); ++i) {
      cst += Cargo::edgeweight(seg.at(i - 1), seg.at(i));
      rteout.push_back({cst, seg.at(i)});
    }
  }
  return cst;
}

DistInt route_through(const std::vector<Stop>& sch, std::vector<Wayp>& rteout) {
  return route_through(sch, rteout, Cargo::gtree());
}

DistInt route_through(const Schedule& sch, std::vector<Wayp>& rteout, GTree::G_Tree& gtree) {
  return route_through(sch.data(), rteout, gtree);
}

DistInt route_through(const Schedule& sch, std::vector<Wayp>& rteout) {
  return route_through(sch.data(), rteout);
}

bool check_precedence_constr(const Schedule& s) {
  // Last stop must be this vehicle destination
  if (s.data().back().type() != StopType::VehlDest ||
      s.data().back().owner() != s.owner()) {
    // std::cout << "last stop is not a vehicle dest, or last stop owner
    // mismatch\n";
    return false;
  }

  // Second-to-last stop cannot be any origin if schedule size > 2
  if (s.data().size() > 2 &&
      (std::prev(s.data().end(), 2)->type() == StopType::CustOrig ||
       std::prev(s.data().end(), 2)->type() == StopType::VehlOrig)) {
    // std::cout << "second-to-last stop is an origin\n";
    return false;
  }

  // In the worst case, this algorithm walks the schedule for each stop in
  // order to find the paired requests. The complexity is O(|schedule|^2).
  bool paired;
  for (auto i = s.data().begin(); i != s.data().end(); ++i) {
    // Any vehicle origin cannot appear in the schedule, aside from at 0
    if (i > s.data().begin() && i->type() == StopType::VehlOrig) {
      // std::cout << "vehicle origin appears not at 0\n";
      return false;
    }

    // A vehicle origin that is not this vehicle cannot appear at 0
    if (i == s.data().begin() && i->type() == StopType::VehlOrig &&
        i->owner() != s.owner()) {
      // std::cout << "vehicle origin mismatch\n";
      return false;
    }

    paired = false;
    for (auto j = s.data().begin(); j != s.data().end() && !paired; ++j) {
      if (i == j) continue;
      if (i->owner() == j->owner()) {
        if (i->type() == StopType::CustOrig &&
            j->type() == StopType::CustDest && i < j)
          paired = true;
        else if (i->type() == StopType::CustOrig &&
                 j->type() == StopType::CustDest && i > j) {
          // std::cout << "cust origin appears after dest\n";
          return false;
        } else if (i->type() == StopType::CustDest &&
                   j->type() == StopType::CustOrig && i > j)
          paired = true;
        else if (i->type() == StopType::CustDest &&
                 j->type() == StopType::CustOrig && i < j) {
          // std::cout << "cust dest appears before origin\n";
          return false;
        } else if (i->type() == StopType::VehlOrig &&
                   j->type() == StopType::VehlDest && i < j)
          paired = true;
        else if (i->type() == StopType::VehlOrig &&
                 j->type() == StopType::VehlDest && i > j) {
          // std::cout << "veh origin appears after dest\n";
          return false;
        } else if (i->type() == StopType::VehlDest &&
                   j->type() == StopType::VehlOrig && i > j)
          paired = true;
        else if (i->type() == StopType::VehlDest &&
                 j->type() == StopType::VehlOrig && i < j) {
          // std::cout << "veh dest appears before origin\n";
          return false;
        }
      }
    }
    if (!paired && (i->type() != StopType::CustDest &&
                    i->type() != StopType::VehlDest)) {
      // std::cout << "origin is unpaired\n";
      return false;
    }
  }
  return true;
}

bool check_timewindow_constr(const std::vector<Stop>& sch,
                             const std::vector<Wayp>& rte) {
  // Check the end point first
  if (sch.back().late() != -1 &&
      sch.back().late() < rte.back().first / (float)Cargo::vspeed())
    return false;

  // Walk along the schedule and the route. O(|schedule|+|route|)
  auto j = rte.begin();
  for (auto i = sch.begin(); i != sch.end(); ++i) {
    while (j->second != i->loc()) ++j;
    if (i->late() != -1 &&
        i->late() < j->first / (float)Cargo::vspeed()) return false;
  }
  return true;
}

bool check_timewindow_constr(const Schedule& sch, const Route& rte) {
  return check_timewindow_constr(sch.data(), rte.data());
}

DistInt sop_insert(const std::vector<Stop>& sch, const Stop& orig,
                       const Stop& dest, bool fix_start, bool fix_end,
                       std::vector<Stop>& schout, std::vector<Wayp>& rteout,
                       GTree::G_Tree& gtree) {
  DistInt mincst = InfInt;
  schout.clear();
  rteout.clear();

  std::vector<Stop> mutsch = sch;   // mutable schedule
  std::vector<Wayp> mutrte;         // mutable route

  auto check = [&](DistInt cst) {
    if (cst < mincst) {
      mincst = cst;
      schout = mutsch;
      rteout = mutrte;
    }
  };

  mutsch.insert(mutsch.begin() + fix_start, orig);
  mutsch.insert(mutsch.begin() + fix_start, dest);

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
  std::vector<Stop>::iterator beg, end;
  for (auto i = mutsch.begin() + fix_start; i != mutsch.end() - 1 - fix_end;
       ++i) {
    beg = (inc == 1) ? i : mutsch.end() - 1 - fix_end;
    end = (inc == 1) ? mutsch.end() - 1 - fix_end : i + 1;
    for (auto j = beg; j != end; j += inc) {
      if (rst) {
        std::iter_swap(i - 1, i + 1);  // <-- O(1)
        rst = false;
      } else
        std::iter_swap(j, j + inc);
      check(route_through(mutsch, mutrte, gtree));
    }
    std::iter_swap(i, i + 1);
    if (inc == 1 && i < mutsch.end() - 2 - fix_end) {
      check(route_through(mutsch, mutrte, gtree));
    }
    if ((inc = -inc) == 1) rst = true;
  }
  return mincst;
}

DistInt sop_insert(const std::vector<Stop>& sch, const Stop& orig,
                       const Stop& dest, bool fix_start, bool fix_end,
                       std::vector<Stop>& schout,
                       std::vector<Wayp>& rteout) {
  return sop_insert(sch, orig, dest, fix_start, fix_end, schout, rteout,
                    Cargo::gtree());
}

DistInt sop_insert(const Vehicle& vehl, const Customer& cust,
                       std::vector<Stop>& schout, std::vector<Wayp>& rteout,
                       GTree::G_Tree& gtree) {
  // The distances to the nodes in the routes found by route_through need
  // to be corrected.  veh.schedule() passed here contains only un-visited
  // stops. The first stop in the schedule is the vehicle's next node
  // (because of step()).  route_through will give this stop a distance of 0.
  // The distances to other stops in the augmented schedule passed to
  // route_through will be relative to this first stop. The already-traveled
  // distance (the head) should be added.
  DistInt head = vehl.route().dist_at(vehl.idx_last_visited_node() + 1);

  Stop cust_o(cust.id(), cust.orig(), StopType::CustOrig, cust.early(), cust.late());
  Stop cust_d(cust.id(), cust.dest(), StopType::CustDest, cust.early(), cust.late());
  DistInt mincst = sop_insert(vehl.schedule().data(), cust_o, cust_d, true,
                                  true, schout, rteout, gtree);
  // Add head to the new nodes in the route
  for (auto& wp : rteout) wp.first += head;
  rteout.insert(rteout.begin(), vehl.route().at(vehl.idx_last_visited_node()));
  return mincst;
}

DistInt sop_insert(const Vehicle& vehl, const Customer& cust,
                       std::vector<Stop>& schout,
                       std::vector<Wayp>& rteout) {
  return sop_insert(vehl, cust, schout, rteout, Cargo::gtree());
}

DistInt sop_insert(const std::shared_ptr<MutableVehicle>& mutvehl,
                       const Customer& cust, std::vector<Stop>& schout,
                       std::vector<Wayp>& rteout) {
  return sop_insert(*mutvehl, cust, schout, rteout);
}

}  // namespace cargo

