#include <ctime>
#include <iomanip>
#include <iostream>

#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// RSAlgorithm
// =============================================================================
using namespace cargo;

void VehiclesSynced(const MutableVehicle &local, const Vehicle &db) {
   REQUIRE(local.route().data() == db.route().data());
   REQUIRE(local.schedule().data() == db.schedule().data());
   REQUIRE(local.idx_last_visited_node() == db.idx_last_visited_node());
   REQUIRE(local.next_node_distance() == db.next_node_distance());
}

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
    Customer cust3 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 4; }));
    Customer cust4 = *(std::find_if(custs.begin(), custs.end(), [&](const Customer& a) { return a.id() == 5; }));

    Stop vehl_orig(vehl.id(), vehl.orig(), StopType::VehlOrig, vehl.early(), vehl.late());
    Stop vehl_dest(vehl.id(), vehl.dest(), StopType::VehlDest, vehl.early(), vehl.late());
    Stop cust1_orig(cust1.id(), cust1.orig(), StopType::CustOrig, cust1.early(), cust1.late());
    Stop cust1_dest(cust1.id(), cust1.dest(), StopType::CustDest, cust1.early(), cust1.late());
    Stop cust2_orig(cust2.id(), cust2.orig(), StopType::CustOrig, cust2.early(), cust2.late());
    Stop cust2_dest(cust2.id(), cust2.dest(), StopType::CustDest, cust2.early(), cust2.late());
    Stop cust3_orig(cust3.id(), cust3.orig(), StopType::CustOrig, cust3.early(), cust3.late());
    Stop cust3_dest(cust3.id(), cust3.dest(), StopType::CustDest, cust3.early(), cust3.late());
    Stop cust4_orig(cust4.id(), cust4.orig(), StopType::CustOrig, cust4.early(), cust4.late());
    Stop cust4_dest(cust4.id(), cust4.dest(), StopType::CustDest, cust4.early(), cust4.late());

    std::vector<Wayp> rte {{0,0}, {2,1}, {4,2}, {6,3}, {8,4}, {11,5}};
    std::vector<Wayp> rerte {{0,0}, {2,1}, {3,7}, {5,8}, {7,9}, {8,3}, {10,4}, {13,5}};

    SECTION("A11. cadd in prefix (fail)") {
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};
      //                             0,          1,          3,         5
      int _; cargo.step(_);
      MutableVehicle sync_vehl(rsalg.get_all_vehicles().front());
      std::cout << "A11. cadd in prefix (fail)" << std::endl;
      REQUIRE(rsalg.assign({cust1.id()}, {}, rte, sch, sync_vehl, true) == false);
    }

    SECTION("A11->A12. normal assign (1,3) (pass)") {
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};
      //                             0,          1,          3,         5
      MutableVehicle sync_vehl(rsalg.get_all_vehicles().front());
      REQUIRE(rsalg.assign({cust1.id()}, {}, rte, sch, sync_vehl, true) == true);
    }

    SECTION("A12. cdel in prefix (fail)") {
      /* 1) Assign */
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};
      //                             0,          1,          3,         5
      MutableVehicle sync_vehl(rsalg.get_all_vehicles().front());
      rsalg.assign({cust1.id()}, {}, rte, sch, sync_vehl, true);
      /* 2) Try to commit a delete */
      int _; cargo.step(_);
      sch = {vehl_orig, vehl_dest};
      //             0,         5
      sync_vehl = rsalg.get_all_vehicles().front();
      REQUIRE(rsalg.assign({}, {cust1.id()}, rte, sch, sync_vehl, true) == false);
    }

    SECTION("A14. pass existing stop (pass)") {
      /* 1) Assign */
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};
      //                             0,          1,          3,         5
      MutableVehicle sync_vehl(rsalg.get_all_vehicles().front());
      rsalg.assign({cust1.id()}, {}, rte, sch, sync_vehl, true);
      /* 2) Pass it */
      int _; cargo.step(_);
      sch = {vehl_orig, cust1_orig, cust1_dest, vehl_dest};
      //             0,          1,          3,         5
      sync_vehl = rsalg.get_all_vehicles().front();
      REQUIRE(rsalg.assign({}, {}, rte, sch, sync_vehl, true) == true);

      Stop nn(vehl.id(), 2, StopType::VehlOrig, vehl.early(), vehl.late());
      std::vector<Wayp> true_rte {{2,1}, {4,2}, {6,3}, {8,4}, {11,5}};
      std::vector<Stop> true_sch {nn, cust1_dest, vehl_dest};
      REQUIRE(sync_vehl.route().data() == true_rte);
      REQUIRE(sync_vehl.schedule().data() == true_sch);
      REQUIRE(sync_vehl.idx_last_visited_node() == 0);
      vehl = rsalg.get_all_vehicles().front();
      VehiclesSynced(sync_vehl, vehl);
    }

    SECTION("A1. curloc matches (pass)") {
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};
      //                             0,          1,          3,         5
      int _; cargo.step(_);
      MutableVehicle sync_vehl(rsalg.get_all_vehicles().front());
      REQUIRE(rsalg.assign({cust1.id()}, {}, rte, sch, sync_vehl) == true);
      print_rte(sync_vehl.route().data());

      Stop nn(vehl.id(), 2, StopType::VehlOrig, vehl.early(), vehl.late());
      std::vector<Wayp> true_rte {{2,1}, {4,2}, {6,1}, {8,2}, {10,3}, {12,4}, {15,5}};
      std::vector<Stop> true_sch {nn, cust1_orig, cust1_dest, vehl_dest};
      REQUIRE(sync_vehl.route().data() == true_rte);
      REQUIRE(sync_vehl.schedule().data() == true_sch);
      REQUIRE(sync_vehl.idx_last_visited_node() == 0);
      vehl = rsalg.get_all_vehicles().front();
      VehiclesSynced(sync_vehl, vehl);
    }

    SECTION("A2. cdel (fail)") {
      /* 1) Assign */
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};
      //                             0,          1,          3,         5
      MutableVehicle sync_vehl(rsalg.get_all_vehicles().front());
      rsalg.assign({cust1.id()}, {}, rte, sch, sync_vehl, true);
      /* 2) Try to commit a delete */
      int _; cargo.step(_);
      sch = {vehl_orig, cust4_orig, cust4_dest, vehl_dest};
      //             0,          7,         9,          5
      sync_vehl = rsalg.get_all_vehicles().front();
      REQUIRE(rsalg.assign({cust4.id()}, {cust1.id()}, rerte, sch, sync_vehl) == false);
    }

    SECTION("A3. pass existing (pass)") {
      /* 1) Assign */
      std::vector<Stop> sch {vehl_orig, cust1_orig, cust1_dest, vehl_dest};
      //                             0,          1,          3,         5
      MutableVehicle sync_vehl(rsalg.get_all_vehicles().front());
      rsalg.assign({cust1.id()}, {}, rte, sch, sync_vehl, true);
      /* 2) Pass it */
      int _; cargo.step(_);
      sch = {vehl_orig, cust1_orig, cust4_orig, cust4_dest, cust1_dest, vehl_dest};
      //             0,          1,          7,          9,          3,         5
      sync_vehl = rsalg.get_all_vehicles().front();
      REQUIRE(rsalg.assign({cust4.id()}, {}, rerte, sch, sync_vehl) == true);

      Stop nn(vehl.id(), 2, StopType::VehlOrig, vehl.early(), vehl.late());
      std::vector<Wayp> true_rte {{2,1}, {4,2}, {5,8}, {7,7}, {9,8}, {11,9}, {12,3}, {14,4}, {17,5}};
      std::vector<Stop> true_sch {nn, cust4_orig, cust4_dest, cust1_dest, vehl_dest};
      REQUIRE(sync_vehl.route().data() == true_rte);
      REQUIRE(sync_vehl.schedule().data() == true_sch);
      REQUIRE(sync_vehl.idx_last_visited_node() == 0);
      vehl = rsalg.get_all_vehicles().front();
      VehiclesSynced(sync_vehl, vehl);
    }
}

