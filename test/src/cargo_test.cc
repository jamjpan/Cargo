#include <algorithm>

#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// Simulator
// =============================================================================
using namespace cargo;

TEST_CASE("Cargo::step()", "") {

    Options opt;
    opt.path_to_roadnet = "step_test.rnet";
    opt.path_to_gtree = "step_test.gtree";
    opt.path_to_edges = "step_test.edges";
    opt.path_to_problem = "step_test.instance";
    opt.time_multiplier = 1;
    opt.matching_period = 10;
    opt.vehicle_speed = 3;

    Cargo cargo(opt);
    RSAlgorithm rsalg;

    Vehicle vehl = rsalg.get_all_vehicles().front();
    std::vector<Customer> custs = rsalg.get_all_customers();
    Customer cust1 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 2; }));
    Customer cust2 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 3; }));

    cust1.print();
    cust2.print();

    Stop vehl_orig(vehl.id(), vehl.orig(), StopType::VehlOrig, vehl.early(), vehl.late());
    Stop vehl_dest(vehl.id(), vehl.dest(), StopType::VehlDest, vehl.early(), vehl.late());
    Stop cust1_orig(cust1.id(), cust1.orig(), StopType::CustOrig, cust1.early(), cust1.late());
    Stop cust1_dest(cust1.id(), cust1.dest(), StopType::CustDest, cust1.early(), cust1.late());
    Stop cust2_orig(cust2.id(), cust2.orig(), StopType::CustOrig, cust2.early(), cust2.late());
    Stop cust2_dest(cust2.id(), cust2.dest(), StopType::CustDest, cust2.early(), cust2.late());

    std::vector<Wayp> rte {{0,0}, {2,1}, {4,2}, {6,3}, {8,4}, {11,5}};
    std::vector<Stop> sch {vehl_orig, cust1_orig, cust2_orig, cust1_dest, cust2_dest, vehl_dest};
    REQUIRE(rsalg.commit({cust1, cust2}, {}, vehl, rte, sch) == true);

    SECTION("speed=3") {
      int stepped;
      int deactivated;

      { /* After 1 step */
        stepped = cargo.step(deactivated);
        REQUIRE(stepped == 1);
        REQUIRE(deactivated == 0);

        vehl = rsalg.get_all_vehicles().front();
        REQUIRE(vehl.last_visited_node() == 1);
        REQUIRE(vehl.next_node_distance() == 1);

        custs = rsalg.get_all_customers();
        cust1 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 2; }));
        cust2 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 3; }));
        REQUIRE(cust1.status() == CustStatus::Onboard);
        REQUIRE(cust2.status() == CustStatus::Waiting);
      }

      { /* After 2 step */
        stepped = cargo.step(deactivated);
        REQUIRE(stepped == 1);
        REQUIRE(deactivated == 0);

        vehl = rsalg.get_all_vehicles().front();
        REQUIRE(vehl.last_visited_node() == 3);
        REQUIRE(vehl.next_node_distance() == 2);

        custs = rsalg.get_all_customers();
        cust1 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 2; }));
        cust2 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 3; }));
        REQUIRE(cust1.status() == CustStatus::Arrived);
        REQUIRE(cust2.status() == CustStatus::Onboard);
      }

      { /* After 3 step */
        stepped = cargo.step(deactivated);
        REQUIRE(stepped == 1);
        REQUIRE(deactivated == 0);

        vehl = rsalg.get_all_vehicles().front();
        REQUIRE(vehl.last_visited_node() == 4);
        REQUIRE(vehl.next_node_distance() == 2);

        custs = rsalg.get_all_customers();
        cust1 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 2; }));
        cust2 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 3; }));
        REQUIRE(cust1.status() == CustStatus::Arrived);
        REQUIRE(cust2.status() == CustStatus::Arrived);
      }

      { /* After 4 step */
        stepped = cargo.step(deactivated);
        REQUIRE(stepped == 1);
        REQUIRE(deactivated == 1);

        vehl = rsalg.get_all_vehicles().front();
        // We don't actually record the vehicle's destination into the db when
        // the vehicle arrives; but we write it into the solution, so it's ok.
        // We also don't update next node distance; it's not needed anymore.
        // REQUIRE(vehl.last_visited_node() == 5);
        // REQUIRE(vehl.next_node_distance() == -1);
        REQUIRE(vehl.status() == VehlStatus::Arrived);

        custs = rsalg.get_all_customers();
        cust1 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 2; }));
        cust2 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 3; }));
        REQUIRE(cust1.status() == CustStatus::Arrived);
        REQUIRE(cust2.status() == CustStatus::Arrived);
      }
    }

    SECTION("speed=11") {
      cargo.vspeed() = 11;
      int stepped;
      int deactivated;

      { /* After 1 step */
        stepped = cargo.step(deactivated);
        REQUIRE(stepped == 1);
        REQUIRE(deactivated == 1);

        vehl = rsalg.get_all_vehicles().front();
        // Same as before.. the vehicle arrived, so we don't do any more
        // updating on it.
        //REQUIRE(vehl.last_visited_node() == 5);
        //REQUIRE(vehl.next_node_distance() == 0);
        REQUIRE(vehl.status() == VehlStatus::Arrived);

        custs = rsalg.get_all_customers();
        cust1 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 2; }));
        cust2 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 3; }));
        REQUIRE(cust1.status() == CustStatus::Arrived);
        REQUIRE(cust2.status() == CustStatus::Arrived);
      }
    }

    SECTION("speed=12") {
      cargo.vspeed() = 12;
      int stepped;
      int deactivated;

      { /* After 1 step */
        stepped = cargo.step(deactivated);
        REQUIRE(stepped == 1);
        REQUIRE(deactivated == 1);

        vehl = rsalg.get_all_vehicles().front();
        // Same as before.. the vehicle arrived, so we don't do any more
        // updating on it.
        //REQUIRE(vehl.last_visited_node() == 5);
        //REQUIRE(vehl.next_node_distance() == 0);
        REQUIRE(vehl.status() == VehlStatus::Arrived);

        custs = rsalg.get_all_customers();
        cust1 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 2; }));
        cust2 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 3; }));
        REQUIRE(cust1.status() == CustStatus::Arrived);
        REQUIRE(cust2.status() == CustStatus::Arrived);
      }
    }
}

