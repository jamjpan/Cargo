#include "catch.hpp"
#include "libcargo.h"

#include <ctime>
#include <iomanip>
#include <iostream>
// =============================================================================
// Functions
// =============================================================================
using namespace cargo;

TEST_CASE("functions", "[functions]") {
    Message msg("functions_test");
    msg << "Reading gtree...";
    GTree::load("../data/roadnetwork/tiny.gtree");
    Cargo::gtree() = GTree::get();
    Cargo::vspeed() = 1;
    msg << "Done\n";

    SECTION("route through") {
        msg << "route through works\n";
        Stop vo1(1, 0, StopType::VehicleOrigin,0,10);
        Stop vd1(1, 6, StopType::VehicleDest,0,10);
        Schedule s(1, {vo1, vd1});
        std::vector<Waypoint> route;
        DistanceInt cost = route_through(s, route);
        msg << "{0,6} => ";
        for (const auto& wp : route)
            msg << "(" << wp.first << ", " << wp.second << ") ";
        msg << "\n";
        REQUIRE(cost == 15);
    }

    SECTION("check precedence constr") {
        msg << "check precedence constr works\n";
        Stop vo1(1, 0, StopType::VehicleOrigin,0,10);
        Stop vd1(1, 6, StopType::VehicleDest,0,10);
        Stop vo2(2, 3, StopType::VehicleOrigin,0,10);
        Stop vd2(2, 7, StopType::VehicleDest,0,10);
        Stop A(3,1,StopType::CustomerOrigin,0,10);
        Stop a(3,5,StopType::CustomerDest,0,10);
        Stop B(4,4,StopType::CustomerOrigin,0,10);
        Stop b(4,8,StopType::CustomerDest,0,10);

        SECTION("vehicle origin appears somewhere not 0") {
            Schedule bad(1, {a, vo1, vd1});
            Schedule good(1, {vo1, a, vd1});
            REQUIRE(check_precedence_constr(bad) == false);
            REQUIRE(check_precedence_constr(good) == true);
        }

        SECTION("vehicle origin is at 0 but not this vehicle") {
            Schedule bad(1, {vo2, a, vd1});
            Schedule good(1, {vo1, a, vd1});
            REQUIRE(check_precedence_constr(bad) == false);
            REQUIRE(check_precedence_constr(good) == true);
        }

        SECTION("vehicle dest appears last but is not this vehicle") {
            Schedule bad(1, {vo1, a, vd2});
            Schedule good(1, {vo1, a, vd1});
            REQUIRE(check_precedence_constr(bad) == false);
            REQUIRE(check_precedence_constr(good) == true);
        }

        SECTION("vehicle dest is not last") {
            Schedule bad1(1, {A, vd1, a});
            Schedule bad2(1, {vd1, A, a});
            Schedule good(1, {A, a, vd1});
            REQUIRE(check_precedence_constr(bad1) == false);
            REQUIRE(check_precedence_constr(bad2) == false);
            REQUIRE(check_precedence_constr(good) == true);
        }

        SECTION("an origin is unpaired") {
            Schedule bad(1, {A, vd1});
            Schedule good(1, {A, a, vd1});
            REQUIRE(check_precedence_constr(bad) == false);
            REQUIRE(check_precedence_constr(good) == true);
        }

        SECTION("an origin appears after destination") {
            Schedule bad(1, {a, A, vd1});
            Schedule good(1, {A, a, vd1});
            REQUIRE(check_precedence_constr(bad) == false);
            REQUIRE(check_precedence_constr(good) == true);
        }

        SECTION("an unpaired destination is ok") {
            Schedule good(1, {a, vd1});
            REQUIRE(check_precedence_constr(good) == true);
        }
    }

    SECTION("check timewindow constr") {
        msg << "check timewindow constr works\n";
        Stop vo1(1, 0, StopType::VehicleOrigin,0,15);
        Stop vd1(1, 6, StopType::VehicleDest,0,15);
        Stop a(2, 5, StopType::CustomerDest,0,12);
        Schedule s(1, {vo1, a, vd1});
        std::vector<Waypoint> good = {{0,0}, {4,1}, {7,2}, {12,5}, {15,6}};
        std::vector<Waypoint> bad1 = {{0,0}, {4,1}, {7,2}, {12,5}, {17,6}};
        std::vector<Waypoint> bad2 = {{0,0}, {4,1}, {7,2}, {14,5}, {15,6}};
        Route rgood(1, good);
        Route rbad1(1, bad1);
        Route rbad2(1, bad2);
        REQUIRE(check_timewindow_constr(s, rgood) == true);
        REQUIRE(check_timewindow_constr(s, rbad1) == false);
        REQUIRE(check_timewindow_constr(s, rbad2) == false);
    }

    SECTION("check sop_insertion works") {
        msg << "sop_insertion works\n";
        Stop A(3,0,StopType::CustomerOrigin,0,10);
        Stop a(3,2,StopType::CustomerDest,0,10);
        Stop B(4,2,StopType::CustomerOrigin,0,10);
        Stop b(4,6,StopType::CustomerDest,0,10);
        Stop C(5,5,StopType::CustomerOrigin,0,10);
        Stop c(5,6,StopType::CustomerDest,0,10);

        Stop goo(1,8,StopType::CustomerOrigin,0,10);
        Stop bat(1,3,StopType::CustomerDest,0,10);
        Customer cust(1, 8, 3, 0, 10, 1, CustomerStatus::Waiting);

        std::vector<Waypoint> route;
        Schedule veh_sch(1, {A, a, B, b, C, c});
        route_through(veh_sch, route);
        Route veh_route(1, route);
        int nnd = route.at(1).first;
        Vehicle veh(1,0,6,0,10,-3,nnd,veh_route,veh_sch,0,VehicleStatus::Enroute);
        msg << "Vehicle route: ";
        veh.route().print();
        // Pretend the vehicle moved a little
        // (veh2 is at the fourth waypoint in its route, and headed toward AaBbCc)
        // (the route is wrong but it's ok, it will be replaced anyway)
        Vehicle veh2(2,0,6,0,10,-3,0,veh_route,veh_sch,4,VehicleStatus::Enroute);
        veh2.print();
        std::vector<Stop> best_schedule;
        std::vector<Waypoint> best_route;
        DistanceInt veh2_best_cost = sop_insert(veh2, cust, true, true, best_schedule, best_route);
        Schedule new_sch(1,best_schedule);
        Route new_route(1,best_route);
        msg << "best cost: " << veh2_best_cost << "\n";
        new_sch.print();
        new_route.print();
    }
}

