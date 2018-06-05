#include "catch.hpp"
#include "libcargo.h"

#include <ctime>
#include <iomanip>
#include <iostream>
// =============================================================================
// DB API
// =============================================================================
using namespace cargo;

TEST_CASE("dbutils", "[dbutils]") {
    Message msg("dbutils_test");

    SECTION("de/serialize works") {
        msg << "De/serialize works\n";
        std::vector<Waypoint> test_vec {{1,2},{3,4},{5,6},{7,8}};
        std::string test_str = serialize_route(test_vec);
        msg << "{{1,2},{3,4},{5,6},{7,8}} => " << test_str << "\n";

        std::vector<Waypoint> result = deserialize_route(test_str);
        msg << test_str << " => {";
        for (const auto& i : result)
            msg << "(" << i.first << ", " << i.second << ") ";
        msg << "}\n";

        Stop a(1,2,StopType::CustomerOrigin,3,4,5);
        Stop b(6,7,StopType::CustomerDest,8,9,10);
        test_str = serialize_schedule({a, b});
        msg << "stop a, stop b => " << test_str << "\n";
        std::vector<Stop> result2 = deserialize_schedule(test_str);
        for (const auto& stop : result2)
            msg << "Stop " << stop.owner() << "; " << stop.location() << "; "
                << std::to_string((int)stop.type()) << "; " << stop.early()
                << "; " << stop.late() << "; " << stop.visitedAt() << "\n";
    }

    SECTION("de/serialize vector of 1,000,000 waypoints") {
        msg << "Serialize vector of 1,000,000 waypoints\n";
        std::vector<Waypoint> test_vec;
        for (int i = 0; i < 1000000; ++i)
            test_vec.push_back({i,i});
        auto t_start = clock();
        std::string test_str = serialize_route(test_vec);
        auto t_end = clock();
        msg << std::fixed << std::setprecision(4) << "\tCPU time used: "
                  << 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC << " ms\n";

        msg << "Deserialize string of 1,000,000 waypoints\n";
        t_start = clock();
        deserialize_route(test_str);
        t_end = clock();
        msg << std::fixed << std::setprecision(4) << "\tCPU time used: "
                  << 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC << " ms\n";
    }

    SECTION("de/serialize vector of 1,000 stops") {
        msg << "Serialize vector of 1,000 stops\n";
        std::vector<Stop> test_vec;
        for (int i = 0; i < 1000; ++i) {
            Stop a(i,2,StopType::CustomerOrigin,3,4,5);
            test_vec.push_back(a);
        }
        auto t_start = clock();
        std::string test_str = serialize_schedule(test_vec);
        auto t_end = clock();
        msg << std::fixed << std::setprecision(4) << "\tCPU time used: "
                  << 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC << " ms\n";

        msg << "Deserialize string of 1,000 stops\n";
        t_start = clock();
        deserialize_schedule(test_str);
        t_end = clock();
        msg << std::fixed << std::setprecision(4) << "\tCPU time used: "
                  << 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC << " ms\n";
    }

}

