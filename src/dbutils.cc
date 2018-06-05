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
#include <iostream>
#include <iterator> /* istream_iterator */
#include <sstream>
#include <string>
#include <vector>

#include "libcargo/dbutils.h"
#include "libcargo/classes.h"
#include "libcargo/types.h"
#include "sqlite3/sqlite3.h"

namespace cargo {

//int DA::UpdateLocation(VehicleId vid, int lv_node, int nnd) {
//    sqlite3_stmt *stmt;
//    rc = sqlite3_prepare_v2(
//        db, "UPDATE vehicle SET lv_node = ?, nnd = ? WHERE id = ?", -1, &stmt,
//        NULL);
//    if (rc != SQLITE_OK) {
//        std::cerr << "prepare error: " << sqlite3_errmsg(db) << std::endl;
//        return -1;
//    } else {
//        sqlite3_bind_int(stmt, 1, lv_node);
//        sqlite3_bind_int(stmt, 2, nnd);
//        sqlite3_bind_int(stmt, 3, vid);
//    }
//    // rc = sqlite3_step(stmt);
//    // if (rc != SQLITE_ROW) {
//    //     std::cerr << "step error: " << sqlite3_errmsg(db) << std::endl;
//    //     return -1;
//    // }
//    rc = sqlite3_step(stmt);
//
//    if (rc != SQLITE_DONE) {
//        std::cerr << "step error: " << sqlite3_errmsg(db) << std::endl;
//        return -1;
//    }
//    // std::cout << "veh " << vid << " move to " << lv_node << std::endl;
//
//    rc = sqlite3_finalize(stmt);
//    if (rc != SQLITE_OK) {
//        std::cerr << "finalize error: " << sqlite3_errmsg(db) << std::endl;
//        return -1;
//    }
//    return 0;
//}
//
//static int OneRequestCallback(void *request, int count, char **data,
//                              char **columns) {
//    request = new Vehicle();
//    for (int i = 0; i < count; ++i)
//        std::cout << columns[i] << " is " << data[i] << std::endl;
//    return 0;
//}
//
//int DA::GetOneRequest(Trip *request) {
//    request = NULL;
//    rc = sqlite3_exec(
//        db,
//        "SELECT * FROM request WHERE matched = 0 ORDER BY online_time "
//        "ASC LIMIT 1",
//        OneRequestCallback, (void *)request, &zErrMsg);
//    if (rc != SQLITE_OK) {
//        std::cerr << "exec error: " << zErrMsg << std::endl;
//        sqlite3_free(zErrMsg);
//        return -1;
//    }
//    if (request == NULL)
//        return 0;
//    else
//        return 1;
//}
//
//static int NRequestCallback(void *requests, int count, char **data,
//                            char **columns) {
//    Trip *request = new Trip();
//    ((std::vector<Trip *> *)requests)->push_back(request);
//    for (int i = 0; i < count; ++i)
//        std::cout << columns[i] << " is " << data[i] << std::endl;
//    return 0;
//}
//
//int DA::GetNRequest(std::vector<Trip *> &requests, int n) {
//    std::string sql =
//        "SELECT * FROM request WHERE matched = 0 ORDER BY online_time ASC ";
//    if (n >= 0)
//        sql += "LIMIT " + std::to_string(n);
//    rc = sqlite3_exec(db, sql.c_str(), NRequestCallback, (void *)(&requests),
//                      &zErrMsg);
//    if (rc != SQLITE_OK) {
//        std::cerr << "exec error: " << zErrMsg << std::endl;
//        sqlite3_free(zErrMsg);
//        return -1;
//    }
//    return requests.size();
//}


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

} // namespace cargo

