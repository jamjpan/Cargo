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

namespace sql {

SqliteReturnCode select_matchable_vehicles(std::vector<Vehicle>& vec, const SimTime& now)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(Cargo::db(), ssv_stmt, -1, &stmt, NULL)) != SQLITE_OK) {
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
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
    if (rc != SQLITE_DONE) {
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

SqliteReturnCode select_waiting_customers(std::vector<Customer>& vec, const SimTime& now)
{
    SqliteReturnCode rc;
    sqlite3_stmt* stmt;
    if ((rc = sqlite3_prepare_v2(Cargo::db(),
    "select * from customers where status = ? and ? > early;", -1, &stmt, NULL)) != SQLITE_OK) {
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
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
        customer.print();
        vec.push_back(customer);
    }
    if (rc != SQLITE_DONE) {
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
    sqlite3_finalize(stmt);
    return SQLITE_OK;
}

SqliteReturnCode commit_assignment(const Customer& cust, const Vehicle& veh,
        std::vector<Waypoint>& new_route, const std::vector<Stop>& new_schedule)
{
    SqliteReturnCode rc;
    sqlite3_stmt* uro_stmt; // update route
    sqlite3_stmt* sch_stmt; // update sched
    sqlite3_stmt* nnd_stmt;
    sqlite3_stmt* lvn_stmt;
    sqlite3_stmt* com_stmt; // assign cust to veh

    // Commit the new route (current_node_idx, nnd are unchanged)
    if ((rc = sqlite3_prepare_v2(Cargo::db(), sql::uro_stmt, -1, &uro_stmt, NULL)) != SQLITE_OK) {
        std::cout << "Error in prepare commit new route " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
    std::string new_route_str = serialize_route(new_route);
    sqlite3_bind_text(uro_stmt, 1, new_route_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(uro_stmt, 2, veh.id());
    if ((rc = sqlite3_step(uro_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit new route " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }

    // ENHANCEMENT check if assignment is valid
    //   If the vehicle has moved a lot during RSAlgorithm::match(), then the
    //   lvn will be different in the db compared to what it is here (because
    //   here was selected before the vehicle moved). How to handle this case?
    //   Reject the assignment, or "roll back" the vehicle?
    //
    //   We roll back the vehicle for now, consider adding the check and rejceting
    //   the assignment in the future.

    // Re-commit the nnd
    if ((rc = sqlite3_prepare_v2(Cargo::db(), sql::nnd_stmt, -1, &nnd_stmt, NULL)) != SQLITE_OK) {
        std::cout << "Error in prepare nnd " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
    sqlite3_bind_int(nnd_stmt, 1, veh.next_node_distance());
    sqlite3_bind_int(nnd_stmt, 2, veh.id());
    if ((rc = sqlite3_step(nnd_stmt)) != SQLITE_DONE) {
        std::cout << "Error in nnd " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }

    // Reset the lvn
    if ((rc = sqlite3_prepare_v2(Cargo::db(), sql::lvn_stmt, -1, &lvn_stmt, NULL)) != SQLITE_OK) {
        std::cout << "Error in prepare lvn " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
    sqlite3_bind_int(lvn_stmt, 1, 0);
    sqlite3_bind_int(lvn_stmt, 2, veh.id());
    if ((rc = sqlite3_step(lvn_stmt)) != SQLITE_DONE) {
        std::cout << "Error in lvn " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }

    // Commit the new schedule
    if ((rc = sqlite3_prepare_v2(Cargo::db(), sql::sch_stmt, -1, &sch_stmt, NULL)) != SQLITE_OK) {
        std::cout << "Error in prepare commit new shcedule " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
    sqlite3_bind_text(sch_stmt, 1, serialize_schedule(new_schedule).c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(sch_stmt, 2, veh.id());
    if ((rc = sqlite3_step(sch_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit new schedule " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }

    // Commit the assignment
    if ((rc = sqlite3_prepare_v2(Cargo::db(), sql::com_stmt, -1, &com_stmt, NULL)) != SQLITE_OK) {
        std::cout << "Error in prepare commit assignment " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }
    sqlite3_bind_int(com_stmt, 1, veh.id());
    sqlite3_bind_int(com_stmt, 2, cust.id());
    if ((rc = sqlite3_step(com_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit assignment " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
        return rc;
    }

    // sqlite3_finalize(sro_stmt);
    sqlite3_finalize(uro_stmt);
    sqlite3_finalize(sch_stmt);
    sqlite3_finalize(nnd_stmt);
    sqlite3_finalize(lvn_stmt);
    sqlite3_finalize(com_stmt);

    return SQLITE_OK;
}

} // namespace sql
} // namespace cargo

