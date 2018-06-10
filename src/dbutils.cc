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
#include <iostream> /* cout */
#include <iterator> /* istream_iterator */
#include <sstream>
#include <string>
#include <vector>

#include "libcargo/dbutils.h"
#include "libcargo/classes.h"
#include "libcargo/types.h"
#include "sqlite3/sqlite3.h"

namespace cargo {

// https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string
std::vector<Waypoint> deserialize_route(const std::string& str)
{
    std::istringstream iss(str);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    std::vector<Waypoint> result;
    for (const auto& token : tokens) {
        std::vector<int> temp;
        size_t start = 0, end = 0;
        while ((end = token.find("|", start)) != std::string::npos) {
            temp.push_back(std::stoi(token.substr(start, end - start)));
            start = end + 1;
        }
        temp.push_back(std::stoi(token.substr(start)));
        Waypoint wp(temp[0], temp[1]);
        result.push_back(wp);
    }
    return result;
}

std::vector<Stop> deserialize_schedule(const std::string& str)
{
    std::istringstream iss(str);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    std::vector<Stop> result;
    for (const auto& token : tokens) {
        std::vector<int> temp;
        size_t start = 0, end = 0;
        while ((end = token.find("|", start)) != std::string::npos) {
            temp.push_back(std::stoi(token.substr(start, end - start)));
            start = end + 1;
        }
        temp.push_back(std::stoi(token.substr(start)));
        Stop stop(temp[0], temp[1], static_cast<StopType>(temp[2]), temp[3],
                  temp[4], temp[5]);
        result.push_back(stop);
    }
    return result;
}

std::string serialize_route(const std::vector<Waypoint>& vec)
{
    std::string result = "";
    for (const auto& i : vec)
        result.append(std::to_string(i.first)
                + "|" + std::to_string(i.second)
                + " "); // whitespace delimiter
    return result;
}

std::string serialize_schedule(const std::vector<Stop>& vec)
{
    std::string result = "";
    for (const auto& stop : vec)
        result.append(std::to_string(stop.owner())
                    + "|" + std::to_string(stop.location())
                    + "|" + std::to_string((int)stop.type())
                    + "|" + std::to_string(stop.early())
                    + "|" + std::to_string(stop.late())
                    + "|" + std::to_string(stop.visitedAt())
                    + " "); // whitespace delimiter
    return result;
}

void commit(const CustomerId& cust_id, const VehicleId& veh_id,
            std::vector<cargo::Waypoint>& new_route,
            const std::vector<cargo::Stop>& new_schedule)
{

    if (cargo::sql::commit_assignment(cust_id, veh_id, new_route, new_schedule) != SQLITE_OK) {
        std::cerr << "Failed commit " << cust_id << "to " << veh_id << "\n";
        throw std::runtime_error(sqlite3_errmsg(cargo::Cargo::db()));
    }
}

namespace sql {

SqliteReturnCode select_matchable_vehicles(std::vector<Vehicle>& vec, const SimTime& now)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(Cargo::db(), ssv_stmt, -1,
                                 &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, now);
    sqlite3_bind_int(stmt, 2, (int)VehicleStatus::Arrived);
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        Route route(
                sqlite3_column_int(stmt, 0),
                deserialize_route(stringify(sqlite3_column_text(stmt, 8))));
        Schedule schedule(
                sqlite3_column_int(stmt, 0),
                deserialize_schedule(stringify(sqlite3_column_text(stmt, 12))));
        Vehicle vehicle(
                sqlite3_column_int(stmt, 0),
                sqlite3_column_int(stmt, 1),
                sqlite3_column_int(stmt, 2),
                sqlite3_column_int(stmt, 3),
                sqlite3_column_int(stmt, 4),
                sqlite3_column_int(stmt, 5),
                sqlite3_column_int(stmt, 10),
                route,
                schedule,
                sqlite3_column_int(stmt, 9),
                static_cast<VehicleStatus>(sqlite3_column_int(stmt, 6)));
        vec.push_back(vehicle);
    }
    if (rc != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

SqliteReturnCode selectall_waiting_customers(std::vector<Customer>& vec, const SimTime& now)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(Cargo::db(),
    "select * from customers where status = ? and ? > early;", -1, &stmt, NULL)) != SQLITE_OK)
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

SqliteReturnCode commit_assignment(const CustomerId& cust_id, const VehicleId& veh_id,
        std::vector<Waypoint>& new_route, const std::vector<Stop>& new_schedule)
{
    // Get existing route
    std::vector<Waypoint> existing_route;
    RouteIndex lvn;
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(Cargo::db(),
    "select * from routes where owner = ?;", -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, veh_id);
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        existing_route = deserialize_route(stringify(sqlite3_column_text(stmt, 1)));
        lvn = sqlite3_column_int(stmt, 2);
    }
    if (rc != SQLITE_DONE)
        return rc;
    sqlite3_reset(stmt);

    // Add current segment to head of the new route
    DistanceInt head = existing_route.at(lvn+1).first - existing_route.at(lvn).first;
    for (auto& wp : new_route)
        wp.first += head;
    new_route.insert(new_route.begin(), existing_route.at(lvn));

    // Commit the new route (current_node_idx, nnd are unchanged)
    if ((rc = sqlite3_prepare_v2(Cargo::db(),
    "update routes set data = ? where owner = ?;", -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    std::string new_route_str = serialize_route(new_route);
    sqlite3_bind_text(stmt, 1, new_route_str.c_str(), new_route_str.length(), SQLITE_STATIC);
    sqlite3_bind_int(stmt, 2, veh_id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_reset(stmt);

    // Commit the new schedule
    if ((rc = sqlite3_prepare_v2(Cargo::db(),
    "update schedules set data = ? where owner = ?;", -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    std::string new_schedule_str = serialize_schedule(new_schedule);
    sqlite3_bind_text(stmt, 1, new_schedule_str.c_str(), new_schedule_str.length(), SQLITE_STATIC);
    sqlite3_bind_int(stmt, 2, veh_id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_reset(stmt);

    // Commit the assignment
    if ((rc = sqlite3_prepare_v2(Cargo::db(),
    "update customers set assignedTo = ? where id = ?;", -1, &stmt, NULL)) != SQLITE_OK)
        return rc;
    sqlite3_bind_int(stmt, 1, veh_id);
    sqlite3_bind_int(stmt, 2, cust_id);
    if ((rc = sqlite3_step(stmt)) != SQLITE_DONE)
        return rc;
    sqlite3_finalize(stmt);

    return SQLITE_OK;
}

} // namespace sql
} // namespace cargo

