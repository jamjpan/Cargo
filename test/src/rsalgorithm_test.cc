#include <ctime>
#include <iomanip>
#include <iostream>

#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// RSAlgorithm
// =============================================================================
using namespace cargo;

TEST_CASE("RSAlgorithm::assign()", "") {

    Options opt;
    opt.path_to_roadnet = "step_test.rnet";
    opt.path_to_gtree = "step_test.gtree";
    opt.path_to_edges = "step_test.edges";
    opt.path_to_problem = "step_test.instance";
    opt.time_multiplier = 1;
    opt.matching_period = 10;
    opt.vehicle_speed = 3;

    Cargo cargo(opt);
    RSAlgorithm rsalg("rsalg_test");

    Vehicle vehl = rsalg.get_all_vehicles().front();
    std::vector<Customer> custs = rsalg.get_all_customers();
    Customer cust1 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 2; }));
    Customer cust2 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 3; }));

    Stop vehl_orig(vehl.id(), vehl.orig(), StopType::VehlOrig, vehl.early(), vehl.late());
    Stop vehl_dest(vehl.id(), vehl.dest(), StopType::VehlDest, vehl.early(), vehl.late());
    Stop cust1_orig(cust1.id(), cust1.orig(), StopType::CustOrig, cust1.early(), cust1.late());
    Stop cust1_dest(cust1.id(), cust1.dest(), StopType::CustDest, cust1.early(), cust1.late());
    Stop cust2_orig(cust2.id(), cust2.orig(), StopType::CustOrig, cust2.early(), cust2.late());
    Stop cust2_dest(cust2.id(), cust2.dest(), StopType::CustDest, cust2.early(), cust2.late());

    SECTION("non-strict assign") {
      int _; cargo.step(_);

      vehl = rsalg.get_all_vehicles().front();
      std::vector<Wayp> rte {{0,0}, {2,1}, {4,2}, {6,3}, {8,4}, {11,5}};
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};

      std::cout << "rsalg_test before assign: " << std::endl;
      vehl.print();

      MutableVehicle sync_vehl(vehl);
      sync_vehl.set_route(rte);
      sync_vehl.set_schedule(sch);
      REQUIRE(rsalg.assign({cust1}, {}, sync_vehl) == true);

      std::cout << "rsalg_test after assign: " << std::endl;
      sync_vehl.print();

      Stop vehl_x1(vehl.id(), 2, StopType::VehlOrig, vehl.early(), vehl.late());
      std::vector<Wayp> true_rte {{2,1}, {4,2}, {6,1}, {8,2}, {10,3}, {12,4}, {15,5}};
      std::vector<Stop> true_sch {vehl_x1, cust1_orig, cust1_dest, vehl_dest};
      REQUIRE(sync_vehl.route().data() == true_rte);
      REQUIRE(sync_vehl.schedule().data() == true_sch);
    }

    SECTION("non-strict assign fails timewindow") {
      Stop cust1_dest2(cust1.id(), cust1.dest(), StopType::CustDest, cust1.early(), 3);

      int _; cargo.step(_);

      vehl = rsalg.get_all_vehicles().front();
      std::vector<Wayp> rte {{0,0}, {2,1}, {4,2}, {6,3}, {8,4}, {11,5}};
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest2, vehl_dest};

      MutableVehicle sync_vehl(vehl);
      sync_vehl.set_route(rte);
      sync_vehl.set_schedule(sch);
      REQUIRE(rsalg.assign({cust1}, {}, sync_vehl) == false);
    }

    SECTION("strict assign") {
      int _; cargo.step(_);

      vehl = rsalg.get_all_vehicles().front();
      std::vector<Wayp> rte {{0,0}, {2,1}, {4,2}, {6,3}, {8,4}, {11,5}};
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};

      std::cout << "rsalg_test before assign: " << std::endl;
      vehl.print();

      MutableVehicle sync_vehl(vehl);
      sync_vehl.set_route(rte);
      sync_vehl.set_schedule(sch);
      REQUIRE(rsalg.assign_strict({cust1}, {}, sync_vehl) == false);
    }
}

