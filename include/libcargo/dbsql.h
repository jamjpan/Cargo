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
    "id             int primary key,"
    "origin_id      int not null,"
    "destination_id int not null,"
    "early          int not null,"
    "late           int not null,"
    "load           int not null,"
    "queued         int not null,"
    "status         int not null,"
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
const SqliteQuery sac_stmt =  // select all customers
    "select * from customers;";

const SqliteQuery stc_stmt =  // select timeout customers
    "select * from customers where assignedTo is null and ? > ? + early "
    "and status != ?;";

const SqliteQuery sav_stmt =  // select all vehicles
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner);";

const SqliteQuery sar_stmt =  // select all routes
    " select * from routes;";

const SqliteQuery ssv_stmt =  // select stepping vehicles
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner) "
    "where  ? >= vehicles.early and "
    "       ? != vehicles.status;";

const SqliteQuery smv_stmt =  // select matchable vehicles
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner) "
    "where  ? >= vehicles.early and "
    "       ? != vehicles.status and "
    "       0 >  vehicles.load;";

const SqliteQuery ssr_stmt =  // select single route
    "select * from routes where owner = ?;";

const SqliteQuery sss_stmt =  // select single schedule
    "select * from schedules where owner = ?;";

const SqliteQuery swc_stmt =  // select waiting customers
    "select * from customers where status = ? and ? >= early;";

/* Update customers */
const SqliteQuery ucs_stmt =  // update customer status
    "update customers set status = ? where id = ?;";

const SqliteQuery com_stmt =  // assign customer
    "update customers set assignedTo = ? where id = ?;";

const SqliteQuery tim_stmt =  // timeout customers
    "update customers set status = ? where assignedTo is null and ? > ? + early;";

/* Update vehicles */
const SqliteQuery pup_stmt =  // increase load (pickup)
    "update vehicles set load = load+1 where id = ?; ";

const SqliteQuery qud_stmt =  // increase queued
    "update vehicles set queued = queued+? where id = ?;";

const SqliteQuery drp_stmt =  // decrease load, queued (dropoff)
    "update vehicles set load = load-1, queued = queued-1 where id = ?; ";

const SqliteQuery dav_stmt =  // deactivate vehicle
    "update vehicles set status = ? where id = ?;";

/* Other updates */
const SqliteQuery vis_stmt =  // update visited at
    "update stops set visitedAt = ? where owner = ? and location = ?;";

const SqliteQuery uro_stmt =  // update route, lvn, nnd
    "update routes set data = ?, idx_last_visited_node = ?, next_node_distance "
    "= ? where owner = ?;";

const SqliteQuery sch_stmt =  // update schedule
    "update schedules set data = ? where owner = ?;";

const SqliteQuery lvn_stmt =  // update last_visited_node
    "update routes set idx_last_visited_node = ? where owner = ?;";

const SqliteQuery nnd_stmt =  // update nearest_node_distance
    "update routes set next_node_distance = ? where owner = ?;";

}  // namespace sql
}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_DBSQL_H_
