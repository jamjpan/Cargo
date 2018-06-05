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

inline std::string stringify(const unsigned char* text) { // for sqlite3 text
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
    ") without rowid;";

const SqliteQuery select_step_vehicles =
    "select * "
    "from   (vehicles inner join routes on vehicles.id=routes.owner"
    "                 inner join schedules on vehicles.id=schedules.owner) "
    "where  ? >= vehicles.early and "
    "       ? != vehicles.status;";

inline SqliteReturnCode
selectall_active_vehicles(std::vector<Vehicle>& vec, SimTime now)
{
    std::string select_step_vehicles =
        "select * from (vehicles inner join routes on vehicles.id=routes.owner"
        " inner join schedules on vehicles.id=schedules.owner)"
        " where ? >= vehicles.early and vehicles.status != ?;";
    SqliteReturnCode rc;
    sqlite3_stmt* ssv_stmt;
    if ((rc = sqlite3_prepare_v2(Cargo::db(), select_step_vehicles.c_str(), -1,
                                 &ssv_stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(ssv_stmt, 1, now);
    sqlite3_bind_int(ssv_stmt, 2, (int)VehicleStatus::Arrived);
    while ((rc = sqlite3_step(ssv_stmt)) == SQLITE_ROW) {
        Route route(
                sqlite3_column_int(ssv_stmt, 0),
                deserialize_route(stringify(sqlite3_column_text(ssv_stmt, 8))));
        Schedule schedule(
                sqlite3_column_int(ssv_stmt, 0),
                deserialize_schedule(stringify(sqlite3_column_text(ssv_stmt, 12))));
        Vehicle vehicle(
                sqlite3_column_int(ssv_stmt, 0),
                sqlite3_column_int(ssv_stmt, 1),
                sqlite3_column_int(ssv_stmt, 2),
                sqlite3_column_int(ssv_stmt, 3),
                sqlite3_column_int(ssv_stmt, 4),
                sqlite3_column_int(ssv_stmt, 5),
                sqlite3_column_int(ssv_stmt, 10),
                route,
                schedule,
                sqlite3_column_int(ssv_stmt, 9),
                static_cast<VehicleStatus>(sqlite3_column_int(ssv_stmt, 6)));
        vec.push_back(vehicle);
    }
    if (rc != SQLITE_DONE)
        return rc;
    sqlite3_finalize(ssv_stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
selectall_waiting_customers(std::vector<Customer>& vec, SimTime now)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "select * from customers where status = ? and ? > early;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, (int)CustomerStatus::Waiting);
    sqlite3_bind_int(stmt, 2, now);
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        // Print columns
        // for (int i = 0; i < sqlite3_column_count(stmt); ++i)
        //    std::cout << "["<<i<<"] "<< sqlite3_column_name(stmt, i) << "\n";
        Customer customer(
            sqlite3_column_int(stmt, 0),
            sqlite3_column_int(stmt, 1),
            sqlite3_column_int(stmt, 2),
            sqlite3_column_int(stmt, 3),
            sqlite3_column_int(stmt, 4),
            sqlite3_column_int(stmt, 5),
            static_cast<CustomerStatus>(sqlite3_column_int(stmt, 6)),
            sqlite3_column_int(stmt, 7));
        vec.push_back(customer);
    }
    if (rc != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
deactivate_vehicle(VehicleId id)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update vehicles set status = ? where id = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, (int)VehicleStatus::Arrived);
    sqlite3_bind_int(stmt, 2, id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
assign_customer_to(CustomerId cust_id, VehicleId veh_id)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update customers set assignedTo = ? where id = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, veh_id);
    sqlite3_bind_int(stmt, 2, cust_id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
update_route(VehicleId veh_id, Route r, RouteIndex ri, DistanceInt nnd)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update routes set data = ?, idx_last_visited_node = ?, next_node_distance = ?  where owner = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    std::string route = serialize_route(r.data());
    sqlite3_bind_text(stmt, 1, route.c_str(), route.length(), SQLITE_STATIC);
    sqlite3_bind_int(stmt, 2, ri);
    sqlite3_bind_int(stmt, 3, nnd);
    sqlite3_bind_int(stmt, 4, veh_id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
replace_route(VehicleId veh_id, Route r)
{
    RouteIndex ri = 0;
    DistanceInt nnd = r.dist_at(1); // distance to next node
    return update_route(veh_id, r, ri, nnd);
}

inline SqliteReturnCode
replace_route(VehicleId veh_id, std::vector<Waypoint> r)
{
    Route route(veh_id, r);
    RouteIndex ri = 0;
    DistanceInt nnd = route.dist_at(1); // distance to next node
    return update_route(veh_id, route, ri, nnd);
}

inline SqliteReturnCode
update_schedule(VehicleId veh_id, Schedule s)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update schedules set data = ? where owner = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    std::string sch = serialize_schedule(s.data());
    sqlite3_bind_text(stmt, 1, sch.c_str(), sch.length(), SQLITE_STATIC);
    sqlite3_bind_int(stmt, 2, veh_id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
update_schedule(VehicleId veh_id, std::vector<Stop> s)
{
    Schedule sch(veh_id, s);
    return update_schedule(veh_id, sch);
}

inline SqliteReturnCode
pickup_customer(VehicleId veh_id, CustomerId cust_id)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update vehicles set load = load+1 where id = ?; update customers set status = ? where id = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, veh_id);
    sqlite3_bind_int(stmt, 2, (int)CustomerStatus::Onboard);
    sqlite3_bind_int(stmt, 3, cust_id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
dropoff_customer(VehicleId veh_id, CustomerId cust_id)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update vehicles set load = load-1 where id = ?; update customers set status = ? where id = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, veh_id);
    sqlite3_bind_int(stmt, 2, (int)CustomerStatus::Arrived);
    sqlite3_bind_int(stmt, 3, cust_id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}


inline SqliteReturnCode
update_visitedAt(TripId id, NodeId loc, SimTime t)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update stops set visitedAt = ? where owner = ? and location = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, t);
    sqlite3_bind_int(stmt, 2, id);
    sqlite3_bind_int(stmt, 3, loc);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
update_next_node_distance(VehicleId id,  DistanceInt nnd)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update routes set next_node_distance = ? where owner = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, nnd);
    sqlite3_bind_int(stmt, 2, id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
update_idx_last_visited_node(VehicleId id, RouteIndex lvn)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update routes set idx_last_visited_node = ? where owner = ?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, lvn);
    sqlite3_bind_int(stmt, 2, id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

inline SqliteReturnCode
timeout_customers(SimTime now, SimTime matching_period)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(
             Cargo::db(), "update customers set status = ? where assignedTo is null and ? > early+?;",
             -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, (int)CustomerStatus::Canceled);
    sqlite3_bind_int(stmt, 2, now);
    sqlite3_bind_int(stmt, 3, matching_period);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

} // namespace sql
} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_DBUTILS_H_

