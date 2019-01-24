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
#ifndef CARGO_INCLUDE_LIBCARGO_DBSQL_H_
#define CARGO_INCLUDE_LIBCARGO_DBSQL_H_

#include "../sqlite3/sqlite3.h"
#include "types.h" /* SqliteQuery types */

/* -------
 * SUMMARY
 * -------
 * This file contains most SQL statements used by Cargo simulator.
 * The statements are:
 *
 *   CREATE STATEMENTS
 *   - create_cargo_tables  create database tables
 *
 *   SELECT STATEMENTS (s--)
 *   - sov_stmt  select one vehicle
 *   - sac_stmt  select all customers
 *   - stc_stmt  select customers to time out
 *   - sav_stmt  select all vehicles
 *   - sar_stmt  select all routes
 *   - ssv_stmt  select stepping vehicles
 *   - smv_stmt  select matchable vehicles
 *   - svs_stmt  select vehicle status
 *   - ssr_stmt  select single route
 *   - sss_stmt  select single schedule
 *   - swc_stmt  select waiting customers
 *   - sva_stmt  select visited at
 *   - cwc_stmt  count waiting customers
 *
 *   UPDATE STATEMENTS
 *   - ucs_stmt  update customer status
 *   - com_stmt  update customer assignment (commit)
 *   - tim_stmt  update timed out customers (set status)
 *   - pup_stmt  update vehl. load due to pickup (TODO: see below)
 *   - qud_stmt  update vehl. queued
 *   - drp_stmt  update vehl. load due to dropoff (TODO: see below)
 *   - dav_stmt  update vehl. status (deactivate)
 *   - uro_stmt  update vehl. route, lvn, nnd
 *   - sch_stmt  update vehl. schedule
 *   - lvn_stmt  update vehl. last-visited node idex
 *   - nnd_stmt  update vehl. next-node distance
 *   - usc_stmt  update vehl. schedule, lvn, nnd
 *   - mov_stmt  update vehicle position (MOVE VEHICLES)
 *   - vis_stmt  update visited at
 */

namespace cargo {

void prepare_stmt(SqliteQuery, sqlite3_stmt**);

namespace sql {

/* Create Cargo database tables. ---------------------------------------------*/
const SqliteQuery create_cargo_tables =
  "create table nodes("
    "id             int primary key,"
    "lng            real not null,"
    "lat            real not null,"
  "unique (lng, lat)"
  ") without rowid;"

  "create table vehicles("
    "id             int primary key,"     // col 0
    "origin_id      int not null,"        // col 1
    "destination_id int not null,"        // col 2
    "early          int not null,"        // col 3
    "late           int not null,"        // col 4
    "load           int not null,"        // col 5
    "queued         int not null,"        // col 6
    "status         int not null,"        // col 7
    "route          blob not null,"       // col 8
    "idx_last_visited_node int not null," // col 9
    "next_node_distance int not null,"    // col 10
    "schedule       blob not null,"       // col 11
  "foreign key (origin_id) references nodes(id),"
  "foreign key (destination_id) references nodes(id)"
  ") without rowid;"

  "create table customers("
    "id             int primary key,"     // col 0
    "origin_id      int not null,"        // col 1
    "destination_id int not null,"        // col 2
    "early          int not null,"        // col 3
    "late           int not null,"        // col 4
    "load           int not null,"        // col 5
    "status         int not null,"        // col 6
    "assignedTo     int,"                 // col 7
  "foreign key (origin_id) references nodes(id),"
  "foreign key (destination_id) references nodes(id),"
  "foreign key (assignedTo) references vehicles(id)"
  ") without rowid;"

  "create table stops("
    "owner          int not null,"
    "location       int not null,"
    "type           int not null,"
    "early          int not null,"
    "late           int not null,"
    "visitedAt      int,"
  "primary key (owner, location),"
  "foreign key (location) references nodes(id)"
  ") without rowid;";

/* Select statements. --------------------------------------------------------*/
const SqliteQuery sov_stmt =  // select one vehicle
  "select * from vehicles "
  "where"
  "  id = ?;";

const SqliteQuery sac_stmt =  // select all customers
  "select * from customers;";

const SqliteQuery stc_stmt =  // select customers to time out
  "select * from customers "
  "where"
  "  assignedTo is null"
  "  and ? > ? + early"  // param1: time now; param2: matching period
  "  and status != ?;";  // param3: CustStats::Canceled

const SqliteQuery sav_stmt =  // select all vehicles
  "select * from vehicles;";

const SqliteQuery sar_stmt =  // select all routes
  "select route from vehicles;";

const SqliteQuery ssv_stmt =  // select stepping vehicles
  "select * from vehicles "
  "where"
  "  ? >= early"       // param1: early (e_i)
  "  and ? != status"  // param2: VehlStatus::Arrived
  "  and next_node_distance <= 0;";

const SqliteQuery smv_stmt =  // select matchable vehicles
  "select * from vehicles "
  "where"
  "  ? >= early"       // param1: early (e_i)
  "  and ? != status"  // param2: VehlStatus::Arrived
  "  and load < 0;";   // TODO but we can add custs to a full vehicle after it makes a dropoff?

const SqliteQuery svs_stmt =  // select vehicle status
  "select status from vehicles "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery ssr_stmt =  // select route, lvn, nnd
  "select route, idx_last_visited_node, next_node_distance from vehicles "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery sss_stmt =  // select single schedule
  "select schedule from vehicles "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery swc_stmt =  // select waiting customers
  "select * from customers "
  "where"
  "  status = ?"        // param1: CustStatus::Waiting
  "  and ? >= early;";  // param2: time now

const SqliteQuery sva_stmt =  // select visited at
  "select visitedAt from stops "
  "where"
  "  owner = ?"       // stop owner id
  "  and type = ?;";  // StopType

const SqliteQuery cwc_stmt =  // count waiting customers
  "select count(id) from customers "
  "where"
  "  assignedTo is null"
  "  and status = ?"    // param1: CustStatus::Waiting
  "  and ? >= early;";  // param2: time now


/* Update Customers. ---------------------------------------------------------*/
const SqliteQuery ucs_stmt =  // update customer status
  "update customers set status = ? "  // param1: CustStatus
  "where"
  "  id = ?;";  // param2: CustId

const SqliteQuery com_stmt =  // update customer assignment
  "update customers set assignedTo = ? " // param1: VehlId
  "where"
  "  id = ?;";  // param2: CustId

const SqliteQuery tim_stmt =  // update timed out customers
  "update customers set status = ? "  // param1: CustStatus::Canceled
  "where"
  "  assignedTo is null"
  "  and ? > ? + early;";  // param2: time now; param3: matching period


/* Update Vehicles. ----------------------------------------------------------*/
const SqliteQuery pup_stmt =  // update load due to pickup
  "update vehicles set load = load+1 "  // TODO: add customer's load, NOT 1
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery qud_stmt =  // update queued
  "update vehicles set queued = queued+? "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery drp_stmt =  // update load and queued due to dropoff
  "update vehicles set load = load-1,"  // TODO: use customer's load, NOT 1
  "                    queued = queued-1 "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery dav_stmt =  // update vehicle status (deactivate)
  "update vehicles set status = ? "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery uro_stmt =  // update vehicle route, lvn, nnd
  "update vehicles set route = ?,"                  // param1: route blob
  "                    idx_last_visited_node = ?,"  // param2: lvn
  "                    next_node_distance = ? "     // param3: nnd
  "where"
  "  id = ?;";  // param4: VehlId

const SqliteQuery sch_stmt =  // update vehicle schedule
  "update vehicles set schedule = ? "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery lvn_stmt =  // update vehicle last-visited node
  "update vehicles set idx_last_visited_node = ? "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery nnd_stmt =  // update vehicle next-node distance
  "update vehicles set next_node_distance = ? "
  "where"
  "  id = ?;";  // param1: VehlId

const SqliteQuery usc_stmt =  // update vehicle schedule, lvn, nnd
  "update vehicles set schedule = ?,"               // param1: schedule blob
  "                    idx_last_visited_node = ?,"  // param2: lvn
  "                    next_node_distance = ? "     // param3: nnd
  "where"
  "  id = ?;"; // param4: VehlId

const SqliteQuery mov_stmt =  // MOVE VEHICLES
  "update vehicles set "
  "         next_node_distance = next_node_distance - ? "  // param1: speed
  "where"
  "  ? >= early"         // param2: time now
  "  and ? != status;";  // param3: VehlStatus:Arrived


/* Update Stops. -------------------------------------------------------------*/
const SqliteQuery vis_stmt =  // update visited at
  "update stops set visitedAt = ? "  // param1: time
  "where"
  "  owner = ?"           // param2: owner (VehlId or CustId)
  "  and location = ?;";  // param3: stop location

}  // namespace sql
}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_DBSQL_H_
