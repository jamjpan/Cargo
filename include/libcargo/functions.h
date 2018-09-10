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
#ifndef CARGO_INCLUDE_LIBCARGO_FUNCTIONS_H_
#define CARGO_INCLUDE_LIBCARGO_FUNCTIONS_H_

#include <memory> /* shared_ptr */

#include "cargo.h"
#include "classes.h"
#include "dbsql.h"
#include "types.h"
#include "../gtree/gtree.h"
#include "../sqlite3/sqlite3.h"

/* -------
 * SUMMARY
 * -------
 * This file contains definitions for common routines.
 */

namespace cargo {

/* Print ---------------------------------------------------------------------*/
void print_rte(const std::vector<Wayp> &);
void print_sch(const std::vector<Stop> &);


/* Prepare -------------------------------------------------------------------*/
void prepare(const SqliteQuery, sqlite3_stmt **);


/* Find Pickup Range ---------------------------------------------------------*/
/* Given the current time (param2), return the maximum distance a vehicle can
 * be in order to pickup customer (param1) within time window constraints */
DistInt pickup_range(
  const Customer &,
  const SimlTime &  // current time
);


/* Create Route Through Schedule ---------------------------------------------*/
/* Given a schedule (param1), output the least-cost route (param2) through all
 * stops; return the cost of this route; O(|schedule|+|nodes|)*/
DistInt route_through(
  const std::vector<Stop> &,
        std::vector<Wayp> &,
        GTree::G_Tree &
);
DistInt route_through(
  const std::vector<Stop> &,
        std::vector<Wayp> &
);


/* Constraints Checking ------------------------------------------------------*/
bool chkpc(const Schedule &);  // check precedence constraint
bool chktw(                    // check time window constraint
  const std::vector<Stop> &,
  const std::vector<Wayp> &
);


/* Schedule Operations -------------------------------------------------------*/
/* Return a random customer from a given schedule; the returned customer is
 * guaranteed to have both stops in the schedule; return -1 if none eligible */
CustId randcust(const std::vector<Stop> &);

/* Remove a customer (param2) from a schedule (param1). The customer MUST have
 * both stops present in the schedule, otherwise function will throw. */
void opdel(
        std::vector<Stop> &,
  const CustId &
);

/* Move a customer (param3) from one schedule (param1) to another (param2). The
 * customer MUST have both stops present in the source schedule, otherwise
 * function will throw. Time windows are not checked in the target schedule
 * after insertion. */
void opmove(
        std::vector<Stop> &,
        std::vector<Stop> &,
  const CustId &
);

/* Swap a customer (param2) from one schedule (param1) with a customer (param4)
 * from a different schedule (param3). The swapping customers MUST have both
 * stops present in their respective schedules, otherwise function will throw.
 * Time windows are not checked in either schedule after swap. */
void opswap(
        std::vector<Stop> &,
  const CustId &,
        std::vector<Stop> &,
  const CustId &);

/* Given a schedule (param1), insert an origin (param2) and dest (param3) while
 * preserving the relative order of the other stops. Set param4 to true to fix
 * the start stop (cannot insert before it), and set param5 to true to fix the
 * end stop (cannot insert after it). Output the least-cost schedule (param6)
 * and the new least-cost route (param7).
 * O(|schedule|^2 * cost of route_through) */
DistInt sop_insert(
  const std::vector<Stop> &,  // schedule to insert into
  const Stop &,               // origin to insert
  const Stop &,               // dest to insert
        bool,                 // fix front stop
        bool,                 // fix end stop
        std::vector<Stop> &,  // augmented schedule
        std::vector<Wayp> &,  // route through augmented schedule
        GTree::G_Tree &       // for sp
);
DistInt sop_insert(  // use default g-tree
  const std::vector<Stop> &,
  const Stop &,
  const Stop &,
        bool,
        bool,
        std::vector<Stop> &,
        std::vector<Wayp> &
);
/* This version corrects the distances in the route (param4) by taking into
 * account vehicle's (param1) traveled distance. */
DistInt sop_insert(
  const Vehicle &,
  const Customer &,
        std::vector<Stop> &,
        std::vector<Wayp> &,
        GTree::G_Tree &
);
DistInt sop_insert(  // use default g-tree
  const Vehicle &,
  const Customer &,
        std::vector<Stop> &,
        std::vector<Wayp> &
);
/* This version is for convenience; vehicles returned by grid are
 * MutableVehicle pointers. */
DistInt sop_insert(
  const std::shared_ptr<MutableVehicle> &,
  const Customer &,
        std::vector<Stop> &,
        std::vector<Wayp> &
);
/* Replace customer (param2) with a new customer (param3)
 * (remove the old customer, then sop_insert the new customer) */
DistInt sop_replace(
  const std::shared_ptr<MutableVehicle> &,
  const CustId &,
  const Customer &,
        std::vector<Stop> &,
        std::vector<Wayp> &
);

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_FUNCTIONS_H_

