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
#include <utility>

#include "libcargo/cargo.h" /* Cargo::gtree() */
#include "libcargo/classes.h"
#include "libcargo/debug.h"
#include "libcargo/distance.h"
#include "libcargo/functions.h"
#include "libcargo/types.h"

#include "gtree/gtree.h"

namespace cargo {

/* Print ---------------------------------------------------------------------*/
void print_rte(const vec_t<Wayp>& rte) {
  for (const auto& wp : rte)
    std::cout << " (" << wp.first << "|" << wp.second << ")";
  std::cout << std::endl;
}

void print_sch(const vec_t<Stop>& sch) {
  for (const auto& sp : sch)
    std::cout << " (" << sp.owner() << "|" << sp.loc() << "|" << sp.early()
              << "|" << sp.late() << "|" << (int)sp.type() << ")";
  std::cout << std::endl;
}


/* Random customer -----------------------------------------------------------*/
CustId randcust(const vec_t<Stop>& sch) {
  vec_t<Stop> s = sch; // make a copy
  std::random_shuffle(s.begin(), s.end());  // randomize the order
  for (auto i = s.begin(); i != s.end()-1; ++i)
    if (i->type() != StopType::VehlOrig && i->type() != StopType::VehlDest)
      for (auto j = i+1; j != s.end(); ++j)
        if (j->owner() == i->owner())
            return i->owner();
  return -1;
}


/* Pickup range --------------------------------------------------------------*/
// TODO: This function assumes the same vehicle speed for all vehicles. In the
// future, change it to accept specific speeds.
DistInt pickup_range(const Customer& cust) {
  return Cargo::vspeed() * cust.late() - Cargo::basecost(cust.id()) -
         Cargo::vspeed() * Cargo::now();
}


/* Route operations ----------------------------------------------------------*/
DistInt route_through(
    const vec_t<Stop>   & sch,
          vec_t<Wayp>   & rteout,
          GTree::G_Tree & gtree,
    const bool          & count)
{
  DistInt cost = 0;
  DistInt traveled = 0;
  rteout.clear();
  Wayp wp = std::make_pair(cost, sch.front().loc());
  // std::cout << "route_through push_back " << wp << std::endl;
  rteout.push_back(wp);

  vec_t<Wayp> path = {};
  for (auto i = sch.cbegin(); i != sch.cend() - 1; ++i) {
    const NodeId& from = i->loc();
    const NodeId& to = (std::next(i,1))->loc();
    path = {};
    // std::cout << from << ">" << to << std::endl;
    //if (from != to) {
      cost = get_shortest_path(from, to, path, gtree, count);
      for (size_t i = 1; i < path.size(); ++i) {
        wp = std::make_pair(path.at(i).first + traveled, path.at(i).second);
        // std::cout << "route_through push_back " << wp << std::endl;
        rteout.push_back(wp);
      }
      traveled += cost;
    //} else {
      // wp = std::make_pair(cost, to);
      // std::cout << "route_through push_back " << wp << std::endl;
      // rteout.push_back(wp);
    //}
  }
  return traveled;
}

DistInt route_through(const vec_t<Stop>& sch, vec_t<Wayp>& rteout, const bool& count) {
  return route_through(sch, rteout, Cargo::gtree(), count);
}

DistInt route_through(const vec_t<Stop>& sch, vec_t<Wayp>& rteout) {
  return route_through(sch, rteout, Cargo::gtree());
}

bool chkpc(const Schedule& s) {
  // Last stop must be this vehicle destination
  if (s.data().back().type() != StopType::VehlDest ||
      s.data().back().owner() != s.owner()) {
    DEBUG(3, { std::cout << "chk_prec() last stop is not a vehl dest, or last stop owner mismatch" << std::endl; });
    return false;
  }

  // A vehicle origin that is not this vehicle cannot appear at 0
  if (s.data().begin()->type() == StopType::VehlOrig &&
      s.data().begin()->owner() != s.owner()) {
    DEBUG(3, { std::cout << "chk_prec() first stop is not this VehlOrig" << std::endl; });
    return false;
  }

  return chkpc(s.data());
}

bool chkpc(const vec_t<Stop>& sch) {

  // Second-to-last stop cannot be any origin if schedule size > 2
  if (sch.size() > 2 &&
      (std::prev(sch.end(), 2)->type() == StopType::CustOrig ||
       std::prev(sch.end(), 2)->type() == StopType::VehlOrig)) {
    DEBUG(3, { std::cout << "chk_prec() 2nd-last stop is an origin" << std::endl; });
    return false;
  }

  // In the worst case, this algorithm walks the schedule for each stop in
  // order to find the paired requests. The complexity is O(|schedule|^2).
  bool paired;
  for (auto i = sch.begin(); i != sch.end(); ++i) {
    // Any vehicle origin cannot appear in the schedule, aside from at 0
    if (i > sch.begin() && i->type() == StopType::VehlOrig) {
      DEBUG(3, { std::cout << "chk_prec() VehlOrig in interior" << std::endl; });
      return false;
    }

    paired = false;
    for (auto j = sch.begin(); j != sch.end() && !paired; ++j) {
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

bool chktw(const vec_t<Stop>& sch, const vec_t<Wayp>& rte) {
  DEBUG(3, { std::cout << "chktw() got sch:"; print_sch(sch); });
  DEBUG(3, { std::cout << "chktw() got rte:"; print_rte(rte); });

  //DistInt remaining_distance = (rte.back().first - rte.front().first);
  //float remaining_time = remaining_distance/(float)Cargo::vspeed();
  //float arrival_time = remaining_time + Cargo::now();
  const int arrival_time =
    (rte.back().first - rte.front().first)/Cargo::vspeed() + Cargo::now();

  // Check the end point first
  if (sch.back().late() != -1 && sch.back().late() < arrival_time) {
    DEBUG(3, { std::cout << "chktw() found "
      << "sch.back().late(): " << sch.back().late()
      //<< "; remaining_distance: " << remaining_distance
      //<< "; remaining_time: " << remaining_time
      << "; current_time: " << Cargo::now()
      << "; arrival_time: " << arrival_time
      << std::endl;
    });
    return false;
  }

  // Walk along the schedule and the route. O(|schedule|+|route|)
  auto j = rte.cbegin();
  for (auto i = sch.cbegin(); i != sch.cend(); ++i) {
    j = std::find_if(rte.cbegin(), rte.cend(), [&](const Wayp& wp) {
      return wp.second == i->loc(); });
    if (j != rte.cend()) {
      // There is a small error in KT when it computes limits because it does
      // not know the "surplus" distance when a vehicle passes a node. So we
      // truncate the ETA here in order to accept the error.  If we're unlucky,
      // the max error is the distance a vehicle travels in 1 sec (10 meters).
      //float eta = (j->first-rte.front().first)/(float)Cargo::vspeed() + Cargo::now();
      const int eta = (j->first-rte.front().first)/Cargo::vspeed() + Cargo::now();
      if (i->late() < eta && i->late() != -1) {
        DEBUG(3, { std::cout << "chktw() found "
          << "i->late(): " << i->late()
          << "; j->first: " << j->first
          << "; rte.front().first: " << rte.front().first
          << "; speed: " << Cargo::vspeed()
          << "; current time: " << Cargo::now()
          << "; eta: " << eta
          << std::endl;
        });
        return false;
      }
    } else {
      std::cout << "chktw reached end before schedule" << std::endl;
      print_sch(sch);
      print_rte(rte);
      throw;
    }
  }
  return true;
}

bool chkcap(const Load& capacity, const vec_t<Stop>& sch) {
  DEBUG(3, { std::cout << "chkcap(2) got "; print_sch(sch); std::cout << std::endl; });
  int q = capacity;  // REMAINING capacity (-load)
  DEBUG(3, { std::cout << "chkcap(2) got capacity=" << capacity << std::endl; });
  for (auto i = sch.cbegin()+1; i != sch.cend()-1; ++i) {
    const Stop& stop = *i;
    if (stop.type() == StopType::CustOrig || stop.type() == StopType::CustDest)
      q = std::move(q) + (stop.type() == StopType::CustOrig ? -1 : 1);  // TODO: Replace 1 with customer's Load
    DEBUG(3, {
      std::cout << "chkcap(2) at " << stop.loc()
                << "(" << ((int)stop.type() == 0 ? "pickup" : "dropoff") << ")"
                << "; new q=" << q << std::endl; });
    if (q < 0) {
      DEBUG(3, { std::cout << "chkcap(2) failed (" << capacity << "): ";
        print_sch(sch);
      });
      return false;
    }
  }
  return true;
}


/* Schedule operations -------------------------------------------------------*/
void opdel(vec_t<Stop>& sch, const CustId& cust_id) {
  // Special case: removed cust is last stop for a TAXI
  bool last_customer_stop = (sch.size() > 2 ? sch.at(sch.size()-2).owner() == cust_id : false);
  bool is_taxi = (sch.back().late() == -1);

  vec_t<Stop> old_sch = sch;
  opdel_any(sch, cust_id);
  if (old_sch.size() - sch.size() != 2) {
    std::cout << "opdel unknown error" << std::endl;
    std::cout << "Schedule: ";
    print_sch(old_sch);
    std::cout << "To remove: " << cust_id << std::endl;
    throw;
  }

  if (last_customer_stop && is_taxi) {
    vec_t<Stop> new_sch = {};
    const Stop& last_stop = sch.at(sch.size()-2);
    Stop new_dest(sch.front().owner(), last_stop.loc(), StopType::VehlDest, last_stop.early(), -1, -1);
    for (size_t i = 0; i < sch.size()-1; ++i)
      new_sch.push_back(sch.at(i));
    new_sch.push_back(new_dest);
    sch = new_sch;
  }
}

void opdel_any(vec_t<Stop>& sch, const CustId& cust_id) {
  vec_t<Stop> new_sch {};
  for (const Stop& a : sch)
    if (a.owner() != cust_id)
      new_sch.push_back(a);
  sch = new_sch;
}

DistInt sop_insert(const vec_t<Stop>& sch, const Stop& orig,
                       const Stop& dest, bool fix_start, bool fix_end,
                       vec_t<Stop>& schout, vec_t<Wayp>& rteout,
                       GTree::G_Tree& gtree) {
  DistInt mincst = InfInt;
  schout.clear();
  rteout.clear();

  vec_t<Stop> mutsch = sch;   // mutable schedule
  vec_t<Wayp> mutrte;         // mutable route

  auto check = [&](DistInt cst) {
    if (cst <= mincst) {
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
  vec_t<Stop>::iterator beg, end;
  for (auto i = mutsch.begin() + fix_start; i != mutsch.end() - 1 - fix_end;
       ++i) {
    beg = (inc == 1) ? i : mutsch.end() - 1 - fix_end;
    end = (inc == 1) ? mutsch.end() - 1 - fix_end : i + 1;
    for (auto j = beg; j != end; j += inc) {
      if (rst) {
        std::iter_swap(i - 1, i + 1);
        rst = false;
      } else
        std::iter_swap(j, j + inc);
      //check(cost_through(mutsch, gtree));
      check(route_through(mutsch, mutrte, gtree));
    }
    std::iter_swap(i, i + 1);
    if (inc == 1 && i < mutsch.end() - 2 - fix_end) {
      //check(cost_through(mutsch, gtree));
      check(route_through(mutsch, mutrte, gtree));
    }
    if ((inc = -inc) == 1) rst = true;
  }

  // After got the best, THEN resolve the full route
  //route_through(schout, rteout, gtree);

  return mincst;
}

DistInt cost_through(const vec_t<Stop>& sch, GTree::G_Tree& gtree) {
  DistInt cst = 0;
  for (SchIdx i = 0; i < sch.size()-1; ++i) {
    const NodeId& from = sch.at(i).loc();
    const NodeId& to = sch.at(i+1).loc();

    if (from != to) {
      if (!Cargo::scexist(from, to)) {
        try { cst += gtree.search(from, to); }
        catch (...) {
          std::cout << "gtree.search(" << from << "," << to << ") failed" << std::endl;
          print_sch(sch);
          std::cout << "index: " << i << std::endl;
          throw;
        }
        std::lock_guard<std::mutex> sclock(Cargo::scmx); // Lock acquired
        Cargo::scput(from, to, cst);
      } else {
        std::lock_guard<std::mutex> sclock(Cargo::spmx); // Lock acquired
        cst += Cargo::scget(from, to);
      }
    } else
      cst = 0;
  }
  return cst;

  //DistInt cost = 0;

  //for (SchIdx i = 0; i < sch.size()-1; ++i) {
  //  const NodeId& from = sch.at(i).loc();
  //  const NodeId& to = sch.at(i+1).loc();

  //  if (from == to) continue;

  //  try { cost += gtree.search(from, to); }
  //  catch (...) {
  //    std::cout << "gtree.search(" << from << "," << to << ") failed" << std::endl;
  //    print_sch(sch);
  //    std::cout << "index: " << i << std::endl;
  //    throw;
  //  }
  //}
  //return cost;
}

DistInt cost_through(const vec_t<Stop>& sch) {
  return cost_through(sch, Cargo::gtree());
}

DistInt sop_insert(const vec_t<Stop>& sch, const Stop& orig,
                       const Stop& dest, bool fix_start, bool fix_end,
                       vec_t<Stop>& schout,
                       vec_t<Wayp>& rteout) {
  return sop_insert(sch, orig, dest, fix_start, fix_end, schout, rteout,
                    Cargo::gtree());
}

DistInt sop_insert(const Vehicle& vehl, const Customer& cust,
                       vec_t<Stop>& schout, vec_t<Wayp>& rteout,
                       GTree::G_Tree& gtree) {
  DistInt head = 0;
  // The distances to the nodes in the routes found by route_through need
  // to be corrected.
  head = vehl.route().data().at(vehl.idx_last_visited_node()+1).first;
  DEBUG(3, {
    std::cout << "set head=" << head << std::endl;
  });

  const Stop cust_o(cust.id(), cust.orig(), StopType::CustOrig, cust.early(), cust.late());
  const Stop cust_d(cust.id(), cust.dest(), StopType::CustDest, cust.early(), cust.late());

  DistInt mincst = 0;

  // If vehl is a taxi, it's last stop is NOT fixed.
  if (vehl.late() == -1) {
    vec_t<Stop> schin(vehl.schedule().data().begin(), vehl.schedule().data().end()-1);
    mincst = sop_insert(
      schin, cust_o, cust_d, true, false, schout, rteout, gtree);
    schout.push_back({vehl.id(), schout.back().loc(), StopType::VehlDest, schout.back().early(), -1, -1});
  } else
    mincst = sop_insert(
      vehl.schedule().data(), cust_o, cust_d, true, true, schout, rteout, gtree);

  DEBUG(3, {
    std::cout << "Before insert " << cust.id() << " into " << vehl.id() << ": " << std::endl;
    print_rte(vehl.route().data());
    std::cout << "After insert " << cust.id() << " into " << vehl.id() << ":" << std::endl;
    print_rte(rteout);
  });

  // Add head to the new nodes in the route
  for (Wayp& wp : rteout) { wp.first += head; }

  DEBUG(3, {
    std::cout << "After adding head: " << std::endl;
    print_rte(rteout);
  });

  // Why did I remove this?
  rteout.insert(rteout.begin(), vehl.route().at(vehl.idx_last_visited_node()));

  DEBUG(3, {
    std::cout << "After adding curloc:" << std::endl;
    print_rte(rteout);
    std::cout << "Returning cost: " << mincst+head << std::endl;
  });

  return mincst + head;
}

DistInt sop_insert(const Vehicle& vehl, const Customer& cust,
                       vec_t<Stop>& schout,
                       vec_t<Wayp>& rteout) {
  return sop_insert(vehl, cust, schout, rteout, Cargo::gtree());
}

DistInt sop_replace(const MutableVehicle& mutvehl,
                    const CustId& rm, const Customer& cust,
                    vec_t<Stop>& schout, vec_t<Wayp>& rteout) {
  MutableVehicle mutcopy = mutvehl;
  vec_t<Stop> sch1 = mutcopy.schedule().data();
  opdel(sch1, rm);
  mutcopy.set_sch(sch1);
  return sop_insert(mutcopy, cust, schout, rteout);
}

DistInt sop_replace(const std::shared_ptr<MutableVehicle>& mutvehl,
                    const CustId& rm, const Customer& cust,
                    vec_t<Stop>& schout, vec_t<Wayp>& rteout) {
  return sop_replace(*mutvehl, rm, cust, schout, rteout);
}

}  // namespace cargo

