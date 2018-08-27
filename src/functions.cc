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
#include <algorithm> /* iter_swap, random_shuffle */
#include <iostream>
#include <iterator>
#include <memory> /* shared_ptr */
#include <mutex> /* lock_guard */

#include "libcargo/cargo.h" /* Cargo::gtree() */
#include "libcargo/classes.h"
#include "libcargo/debug.h"
#include "libcargo/distance.h"
#include "libcargo/functions.h"
#include "libcargo/types.h"

#include "gtree/gtree.h"

namespace cargo {

void print_rte(const std::vector<Wayp>& rte) {
  for (const auto& wp : rte)
    std::cout << " (" << wp.first << "|" << wp.second << ")";
  std::cout << std::endl;
}

void print_sch(const std::vector<Stop>& sch) {
  for (const auto& sp : sch)
    std::cout << " (owner=" << sp.owner() << ";loc=" << sp.loc() << ";e=" << sp.early()
              << ";late=" << sp.late() << ";type=" << (int)sp.type() << ")";
  std::cout << std::endl;
}


DistInt pickup_range(const Customer      & cust,
                     const SimlTime      & now) {
  DistInt dist = Cargo::basecost(cust.id());
  return (cust.late()-(dist/Cargo::vspeed())-now)*Cargo::vspeed();
}

// Complexity: O(|schedule|*|route|)
//   - for loop body executes n-1 times, for n stops
//   - std::copy is exactly |route| operations
//   - assume find_path, shortest_path_dist are O(1) with gtree
DistInt route_through(const std::vector<Stop> & sch,
                            std::vector<Wayp> & rteout,
                            GTree::G_Tree     & gtree) {
  DistInt cst = 0;
  rteout.clear();
  rteout.push_back({0, sch.front().loc()});
  for (SchIdx i = 0; i < sch.size()-1; ++i) {
    const NodeId& from = sch.at(i).loc();
    const NodeId& to = sch.at(i+1).loc();

    std::vector<NodeId> seg {};
    bool in_cache = false;
    { std::lock_guard<std::mutex> splock(Cargo::spmx); // Lock acquired
    in_cache = Cargo::spexist(from, to);
    if (in_cache)
      seg = Cargo::spget(from, to);
    } // Lock released
    if(!in_cache) {
      try { gtree.find_path(from, to, seg); }
      catch (...) {
        std::cout << "gtree.find_path(" << from << "," << to << ") failed" << std::endl;
        print_sch(sch);
        std::cout << "index: " << i << std::endl;
        throw;
      }
      std::lock_guard<std::mutex> splock(Cargo::spmx); // Lock acquired
      Cargo::spput(from, to, seg);
    } // Lock released
    for (size_t i = 1; i < seg.size(); ++i) {
      cst += Cargo::edgew(seg.at(i-1), seg.at(i));
      rteout.push_back({cst, seg.at(i)});
    }
  }
  return cst;
}

DistInt route_through(const std::vector<Stop> & sch,
                            std::vector<Wayp> & rteout) {
  return route_through(sch, rteout, Cargo::gtree());
}

bool chkpc(const Schedule& s) {
  // Last stop must be this vehicle destination
  if (s.data().back().type() != StopType::VehlDest ||
      s.data().back().owner() != s.owner()) {
    DEBUG(3, { std::cout << "chk_prec() last stop is not a vehl dest, or last stop owner mismatch" << std::endl; });
    return false;
  }

  // Second-to-last stop cannot be any origin if schedule size > 2
  if (s.data().size() > 2 &&
      (std::prev(s.data().end(), 2)->type() == StopType::CustOrig ||
       std::prev(s.data().end(), 2)->type() == StopType::VehlOrig)) {
    DEBUG(3, { std::cout << "chk_prec() 2nd-last stop is an origin" << std::endl; });
    return false;
  }

  // In the worst case, this algorithm walks the schedule for each stop in
  // order to find the paired requests. The complexity is O(|schedule|^2).
  bool paired;
  for (auto i = s.data().begin(); i != s.data().end(); ++i) {
    // Any vehicle origin cannot appear in the schedule, aside from at 0
    if (i > s.data().begin() && i->type() == StopType::VehlOrig) {
      DEBUG(3, { std::cout << "chk_prec() first stop is not a VehlOrig" << std::endl; });
      return false;
    }

    // A vehicle origin that is not this vehicle cannot appear at 0
    if (i == s.data().begin() && i->type() == StopType::VehlOrig &&
        i->owner() != s.owner()) {
      DEBUG(3, { std::cout << "chk_prec() first stop is not this VehlOrig" << std::endl; });
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
          DEBUG(3, { std::cout << "CustOrig appears after its CustDest" << std::endl; });
          return false;
        } else if (i->type() == StopType::CustDest &&
                   j->type() == StopType::CustOrig && i > j)
          paired = true;
        else if (i->type() == StopType::CustDest &&
                 j->type() == StopType::CustOrig && i < j) {
          DEBUG(3, { std::cout << "CustDest appears before its CustOrig" << std::endl; });
          return false;
        } else if (i->type() == StopType::VehlOrig &&
                   j->type() == StopType::VehlDest && i < j)
          paired = true;
        else if (i->type() == StopType::VehlOrig &&
                 j->type() == StopType::VehlDest && i > j) {
          DEBUG(3, { std::cout << "VehlOrig appears after its VehlDest" << std::endl; });
          return false;
        } else if (i->type() == StopType::VehlDest &&
                   j->type() == StopType::VehlOrig && i > j)
          paired = true;
        else if (i->type() == StopType::VehlDest &&
                 j->type() == StopType::VehlOrig && i < j) {
          DEBUG(3, { std::cout << "VehlDest appears before its VehlOrig" << std::endl; });
          return false;
        }
      }
    }
    if (!paired && (i->type() != StopType::CustDest &&
                    i->type() != StopType::VehlDest)) {
      DEBUG(3, { std::cout << "An origin is unpaired" << std::endl; });
      return false;
    }
  }
  return true;
}

bool chktw(const std::vector<Stop>& sch, const std::vector<Wayp>& rte) {
  DEBUG(3, { std::cout << "chktw() got sch:"; print_sch(sch); });
  DEBUG(3, { std::cout << "chktw() got rte:"; print_rte(rte); });

  // Check the end point first
  if (sch.back().late() != -1 &&
      sch.back().late() < rte.back().first / (float)Cargo::vspeed()) {
    DEBUG(3, { std::cout << "chktw() found "
      << "sch.back().late()["      << sch.back().late() << "] < "
      << "rte.back().first/speed[" << rte.back().first / (float)Cargo::vspeed() << "]"
      << std::endl;
    });
    return false;
  }

  // Walk along the schedule and the route. O(|schedule|+|route|)
  auto j = rte.begin();
  for (auto i = sch.begin(); i != sch.end(); ++i) {
    while (j->second != i->loc()) {
      ++j;
      if (j == rte.end()) {
        std::cout << "chktw reached end before schedule" << std::endl;
        print_sch(sch);
        print_rte(rte);
        throw;
      }
    }
    if (i->late() != -1 &&
        i->late() < j->first / (float)Cargo::vspeed()) {
      DEBUG(3, { std::cout << "chktw() found "
        << "i->late()["      << i->late() << "] < "
        << "j->first/speed[" << j->first / (float)Cargo::vspeed() << "]"
        << std::endl;
      });
      return false;
    }
  }
  return true;
}

CustId randcust(const std::vector<Stop>& sch) {
  std::vector<Stop> s = sch; // make a copy
  std::random_shuffle(s.begin(), s.end());  // randomize the order
  for (auto i = s.begin(); i != s.end()-1; ++i)
    if (i->type() != StopType::VehlOrig && i->type() != StopType::VehlDest)
      for (auto j = i+1; j != s.end(); ++j)
        if (j->owner() == i->owner())
            return i->owner();
  return -1;
}

void opdel(std::vector<Stop>& sch, const CustId& cust_id) {
  std::vector<Stop> new_sch {};
  for (const Stop& a : sch)
    if (a.owner() != cust_id)
      new_sch.push_back(a);
  if (sch.size() - new_sch.size() != 2) {
    std::cout << "opdel unknown error" << std::endl;
    throw;
  }
  sch = new_sch;
}

void opmove(std::vector<Stop>& src,
            std::vector<Stop>& dest, const CustId& cust_id) {

}

void opswap(std::vector<Stop>& s1, const CustId& c1,
            std::vector<Stop>& s2, const CustId& c2) {

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
  return mincst+head;
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

DistInt sop_replace(const std::shared_ptr<MutableVehicle>& mutvehl,
                    const CustId& rm, const Customer& cust,
                    std::vector<Stop>& schout, std::vector<Wayp>& rteout) {
  MutableVehicle mutcopy = *mutvehl;
  std::vector<Stop> sch1 = mutcopy.schedule().data();
  opdel(sch1, rm);
  mutcopy.set_sch(sch1);
  return sop_insert(mutcopy, cust, schout, rteout);
}

}  // namespace cargo

