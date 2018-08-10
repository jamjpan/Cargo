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

#include "types.h" /* SqliteQuery types */

namespace cargo {
namespace sql {

const SqliteQuery create_cargo_tables =
    "create table nodes("
    "id             int primary key,"
    "lng            real not null,"
    "lat            real not null,"
    "unique (lng, lat)"
    ") without rowid;"
    "create table vehicles("
    "id             int primary key," // col 0
    "origin_id      int not null,"    // col 1
    "destination_id int not null,"    // col 2
    "early          int not null,"    // col 3
    "late           int not null,"    // col 4
    "load           int not null,"    // col 5
    "queued         int not null,"    // col 6
    "status         int not null,"    // col 7
    "route          blob not null,"   // col 8
    "idx_last_visited_node int not null," // col 9
    "next_node_distance int not null,"// col 10
    "schedule       blob not null,"   // col 11
    "foreign key (origin_id) references nodes(id),"
    "foreign key (destination_id) references nodes(id)"
    ") without rowid;"
    "create table customers("
    "id             int primary key,"
    "origin_id      int not null,"
    "destination_id int not null,"
    "early          int not null,"
    "late           int not null,"
    "load           int not null,"
    "status         int not null,"
    "assignedTo     int,"
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
    ") without rowid;"
    "create table schedules("
    "owner          int primary key,"
    "data           blob not null,"
    "foreign key (owner) references vehicles(id)"
    ") without rowid;"
    "create table routes("
    "owner          int primary key,"
    "data           blob not null,"
    "idx_last_visited_node int not null,"
    "next_node_distance int not null,"
    "foreign key (owner) references vehicles(id)"
    ") without rowid;";

/* Select statements */

// Select all customers
const SqliteQuery sac_stmt = "select * from customers;";

// Select customers to time out
const SqliteQuery stc_stmt = "select * from customers where assignedTo is null and ? > ? + early and status != ?;";

// Select all vehicles
const SqliteQuery sav_stmt2= "select * from vehicles;";
const SqliteQuery sav_stmt = // (DEPRECATED)
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner);";

// Select all routes
const SqliteQuery sar_stmt2= "select route from vehicles;";
const SqliteQuery sar_stmt = // (DEPRECATED)
    " select * from routes;";

// Select vehicles to step
const SqliteQuery ssv_stmt2= "select * from vehicles where ? >= early and ? != status and next_node_distance <= 0;";
const SqliteQuery ssv_stmt = // (DEPRECATED)
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner) "
    "where  ? >= vehicles.early and "
    "       ? != vehicles.status;";

// Select matchable vehicles
const SqliteQuery smv_stmt2= "select * from vehicles where ? >= early and ? != status and 0 > load;";
const SqliteQuery smv_stmt = // (DEPRECATED)
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner) "
    "where  ? >= vehicles.early and "
    "       ? != vehicles.status and "
    "       0 >  vehicles.load;";

// Select vehicle status
const SqliteQuery svs_stmt = "select status from vehicles where id = ?;";

// Select single route, lvn, nnd
const SqliteQuery ssr_stmt2= "select route, idx_last_visited_node, next_node_distance from vehicles where id = ?;";
const SqliteQuery ssr_stmt = // (DEPRECATED)
    "select * from routes where owner = ?;";

// Select single schedule
const SqliteQuery sss_stmt2= "select schedule from vehicles where id = ?;";
const SqliteQuery sss_stmt = // (DEPRECATED)
    "select * from schedules where owner = ?;";

// Select waiting customers
const SqliteQuery swc_stmt = "select * from customers where status = ? and ? >= early;";

/* Update customers */

// Update customer status
const SqliteQuery ucs_stmt = "update customers set status = ? where id = ?;";

// Assign customer
const SqliteQuery com_stmt = "update customers set assignedTo = ? where id = ?;";

// Time out customers
const SqliteQuery tim_stmt = "update customers set status = ? where assignedTo is null and ? > ? + early;";


/* Update vehicles */

// Pickup (increase load)
const SqliteQuery pup_stmt = "update vehicles set load = load+1 where id = ?; ";

// Increase queued
const SqliteQuery qud_stmt = "update vehicles set queued = queued+? where id = ?;";

// Dropoff (decrease load, queued)
const SqliteQuery drp_stmt =  "update vehicles set load = load-1, queued = queued-1 where id = ?; ";

// Deactivate vehicle
const SqliteQuery dav_stmt = "update vehicles set status = ? where id = ?;";

// Update route, last-visited node, next-node distance
const SqliteQuery uro_stmt2= "update vehicles set route = ?, idx_last_visited_node = ?, next_node_distance = ? where id = ?;";
const SqliteQuery uro_stmt = // (DEPRECATED)
    "update routes set data = ?, idx_last_visited_node = ?, next_node_distance "
    "= ? where owner = ?;";

// Update schedule
const SqliteQuery sch_stmt2= "update vehicles set schedule = ? where id = ?;";
const SqliteQuery sch_stmt = // (DEPRECATED)
    "update schedules set data = ? where owner = ?;";

// Update last-visited node
const SqliteQuery lvn_stmt2= "update vehicles set idx_last_visited_node = ? where id = ?;";
const SqliteQuery lvn_stmt = // (DEPRECATED)
    "update routes set idx_last_visited_node = ? where owner = ?;";

// Update next-node distance
const SqliteQuery nnd_stmt2= "update vehicles set next_node_distance = ? where id = ?;";
const SqliteQuery nnd_stmt = // (DEPRECATED)
    "update routes set next_node_distance = ? where owner = ?;";

// Update schedule, lvn, and nnd
const SqliteQuery usc_stmt = "update vehicles set schedule = ?, idx_last_visited_node = ?, next_node_distance = ? where id = ?;";

// Move vehicles (bulk-update nnd)
const SqliteQuery mov_stmt = "update vehicles set next_node_distance = next_node_distance - ? where ? >= early and ? != status;";

/* Update stops */

// Update visitedAt
const SqliteQuery vis_stmt = "update stops set visitedAt = ? where owner = ? and location = ?;";

}  // namespace sql
}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_DBSQL_H_
