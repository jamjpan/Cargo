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
#ifndef CARGO_INCLUDE_LIBCARGO_DBUTILS_H_
#define CARGO_INCLUDE_LIBCARGO_DBUTILS_H_

#include <iostream> // debug
#include <string>
#include <vector>

#include "cargo.h" /* static db() */
#include "classes.h"
#include "types.h"
#include "../sqlite3/sqlite3.h"

namespace cargo {

std::string serialize_route(const std::vector<Waypoint> &);
std::string serialize_schedule(const std::vector<Stop> &);
std::vector<Waypoint> deserialize_route(const std::string &);
std::vector<Stop> deserialize_schedule(const std::string &);
inline std::string stringify(const unsigned char* text) {
    return std::string(reinterpret_cast<const char*>(text));
}

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
        "data           text not null,"
        "foreign key (owner) references vehicles(id)"
    ") without rowid;"
    "create table routes("
        "owner          int primary key,"
        "data           text not null,"
        "idx_last_visited_node int not null,"
        "next_node_distance int not null,"
        "foreign key (owner) references vehicles(id)"
    ") without rowid;"
    "create table statistics("
        "run_id         int primary key,"
        "problem_name   text,"
        "n_customers    int,"
        "m_vehicles     int,"
        "base_cost      int,"
        "solution_name  text,"
        "solution_cost  int,"
        "matched_customers int,"
        "matched_vehicles int,"
        "sum_pickup     int,"
        "sum_delay      int"
    ") without rowid;";

const SqliteQuery select_vehicle =
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner) "
    "where  ? = vehicles.id;";

const SqliteQuery usn_stmt =
    "update statistics set solution_name = ? where run_id = ?;";

const SqliteQuery tim_stmt =
    "update customers set status = ? where assignedTo is null and ? > early+?;";

const SqliteQuery ssv_stmt =
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner) "
    "where  ? >= vehicles.early and "
    "       ? != vehicles.status;";

const SqliteQuery dav_stmt =
    "update vehicles set status = ? where id = ?;";

const SqliteQuery pup_stmt =
    "update vehicles set load = load+1 where id = ?; "
    "update customers set status = ? where id = ?;";

const SqliteQuery drp_stmt =
    "update vehicles set load = load-1 where id = ?; "
    "update customers set status = ? where id = ?;";

const SqliteQuery vis_stmt =
    "update stops set visitedAt = ? where owner = ? and location = ?;";

const SqliteQuery sch_stmt =
    "update schedules set data = ? where owner = ?;";

const SqliteQuery lvn_stmt =
    "update routes set idx_last_visited_node = ? where owner = ?;";

const SqliteQuery nnd_stmt =
    "update routes set next_node_distance = ? where owner = ?;";

SqliteReturnCode select_matchable_vehicles(std::vector<Vehicle> &, const SimTime &);
SqliteReturnCode select_waiting_customers(std::vector<Customer> &, const SimTime &);
SqliteReturnCode commit_assignment(const CustomerId &, const VehicleId &, std::vector<Waypoint> &, const std::vector<Stop> &);

} // namespace sql
} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_DBUTILS_H_

