#include "libcargo.h"
#include "catch.hpp"

using namespace cargo;

SCENARIO("print test-3 intro") {
  std::cout
    << "-----------------------------------------------------------\n"
    << " C A R G O -- Test Route Functions \n"
    << "-----------------------------------------------------------"
    << std::endl;
}

SCENARIO("sop_insert returns correctly distanced routes", "[functions.h]") {

  GIVEN("vehl1 and cust3 from small-rs-bj5-s10-x0.0.instance") {
    Options option;
    std::string path = "/home/jpan/devel/Cargo_benchmark/";
    option.path_to_roadnet = path + "road/bj5.rnet";
    option.path_to_problem = path + "problem/small-rs-bj5-s10-x0.0.instance";
    Cargo cargo(option);
    RSAlgorithm alg("", false);  // empty alg

    vec_t<Vehicle> vehicles = alg.get_all_vehicles();
    Vehicle* Vehl1= nullptr;
    for (Vehicle& vehl : vehicles)
      if (vehl.id() == 1) Vehl1 = &vehl;

    CHECK(Vehl1->id() == 1);
    CHECK(Vehl1->orig() == 42833);
    CHECK(Vehl1->dest() == 35518);

    vec_t<Customer> customers = alg.get_all_customers();
    Customer* Cust3 = nullptr;
    Customer* Cust4 = nullptr;
    for (Customer& cust : customers) {
      if (cust.id() == 3) Cust3 = &cust;
      if (cust.id() == 4) Cust4 = &cust;
    }

    CHECK(Cust3->id() == 3);
    CHECK(Cust3->orig() == 264377);
    CHECK(Cust3->dest() == 118317);
    CHECK(Cust4->id() == 4);
    CHECK(Cust4->orig() == 272191);
    CHECK(Cust4->dest() == 235398);

    std::cout << Vehl1->route() << std::endl;

    vec_t<Wayp> route;
    vec_t<Stop> schedule;

    THEN("t=0, sop_insert head=41") {
      sop_insert(*Vehl1, *Cust3, schedule, route, Cargo::gtree());
      REQUIRE(route.at(1).first == 41);

      AND_THEN("t=1, first sop_insert head=41") {
        int count_moved = 0;
        cargo.step(count_moved);  // t=1
        vehicles = alg.get_all_vehicles();  // TODO Cargo::get_vehicle(id)
        for (Vehicle& vehl : vehicles)
          if (vehl.id() == 1) Vehl1 = &vehl;
        CHECK(Vehl1->next_node_distance() == 31);
        CHECK(Vehl1->idx_last_visited_node() == 0);
        CHECK(Vehl1->last_visited_node() == 42833);
        sop_insert(*Vehl1, *Cust3, schedule, route, Cargo::gtree());
        REQUIRE(route.at(1).first == 41);

        AND_THEN("t=1, second sop_insert head=41") {
          MutableVehicle mutVehl1(*Vehl1);
          mutVehl1.set_rte(route);
          mutVehl1.set_sch(schedule);
          mutVehl1.reset_lvn();
          sop_insert(mutVehl1, *Cust4, schedule, route, Cargo::gtree());
          REQUIRE(route.at(1).first == 41);
        }
      }

      AND_THEN("t=5, first sop_insert head=103") {
        int count_moved = 0;
        cargo.step(count_moved);  // t=1
        cargo.step(count_moved);  // t=2
        cargo.step(count_moved);  // t=3
        cargo.step(count_moved);  // t=4
        cargo.step(count_moved);  // t=5  moved to 42832
        vehicles = alg.get_all_vehicles();  // TODO Cargo::get_vehicle(id)
        for (Vehicle& vehl : vehicles)
          if (vehl.id() == 1) Vehl1 = &vehl;
        CHECK(Vehl1->next_node_distance() == 53);
        CHECK(Vehl1->idx_last_visited_node() == 1);
        CHECK(Vehl1->last_visited_node() == 42832);
        sop_insert(*Vehl1, *Cust3, schedule, route, Cargo::gtree());
        REQUIRE(route.at(1).first == 103);

        AND_THEN("t=5, second sop_insert head=103") {
          MutableVehicle mutVehl1(*Vehl1);
          mutVehl1.set_rte(route);
          mutVehl1.set_sch(schedule);
          mutVehl1.reset_lvn();
          sop_insert(mutVehl1, *Cust4, schedule, route, Cargo::gtree());
          REQUIRE(route.at(1).first == 103);
        }
      }
    }
  }

  GIVEN("vehl1 and cust3 from small-rh-bj5-s10-x0.0.instance") {
    Options option;
    std::string path = "/home/jpan/devel/Cargo_benchmark/";
    option.path_to_roadnet = path + "road/bj5.rnet";
    option.path_to_problem = path + "problem/small-rh-bj5-s10-x0.0.instance";
    Cargo cargo(option);
    RSAlgorithm alg("", false);  // empty alg

    vec_t<Vehicle> vehicles = alg.get_all_vehicles();
    Vehicle* Vehl1= nullptr;
    for (Vehicle& vehl : vehicles)
      if (vehl.id() == 1) Vehl1 = &vehl;

    CHECK(Vehl1->id() == 1);
    CHECK(Vehl1->orig() == 42833);
    CHECK(Vehl1->dest() == -1);

    vec_t<Customer> customers = alg.get_all_customers();
    Customer* Cust3 = nullptr;
    Customer* Cust4 = nullptr;
    for (Customer& cust : customers) {
      if (cust.id() == 3) Cust3 = &cust;
      if (cust.id() == 4) Cust4 = &cust;
    }

    CHECK(Cust3->id() == 3);
    CHECK(Cust3->orig() == 264377);
    CHECK(Cust3->dest() == 118317);
    CHECK(Cust4->id() == 4);
    CHECK(Cust4->orig() == 272191);
    CHECK(Cust4->dest() == 235398);

    std::cout << Vehl1->route() << std::endl;

    vec_t<Wayp> route;
    vec_t<Stop> schedule;

    THEN("t=0, sop_insert head=0") {
      sop_insert(*Vehl1, *Cust3, schedule, route, Cargo::gtree());
      REQUIRE(route.at(1).first == 0);

      MutableVehicle mutVehl1(*Vehl1);
      mutVehl1.set_rte(route);
      mutVehl1.set_sch(schedule);
      mutVehl1.reset_lvn();

      alg.assign({Cust3->id()}, {}, route, schedule, mutVehl1);

      AND_THEN("t=0, second sop_insert head=0") {
        // Checking against the local
        sop_insert(mutVehl1, *Cust4, schedule, route, Cargo::gtree());
        REQUIRE(route.at(1).first == 0);
      }

      AND_THEN("t=5, first sop_insert head=60") {
        int count_moved = 0;
        cargo.step(count_moved);  // t=1
        cargo.step(count_moved);  // t=2
        cargo.step(count_moved);  // t=3
        cargo.step(count_moved);  // t=4
        cargo.step(count_moved);  // t=5  moved to 277564
        vehicles = alg.get_all_vehicles();  // TODO Cargo::get_vehicle(id)
        for (Vehicle& vehl : vehicles)
          if (vehl.id() == 1) Vehl1 = &vehl;
        // Checking against the DB
        CHECK(Vehl1->next_node_distance() == 10);
        CHECK(Vehl1->idx_last_visited_node() == 7);
        CHECK(Vehl1->last_visited_node() == 277564);
        sop_insert(*Vehl1, *Cust4, schedule, route, Cargo::gtree());
        REQUIRE(route.at(1).first == 60);
      }

      AND_THEN("t=238, first sop_insert head=2387") {
        int count_moved = 0;
        for (int i = 0; i < 238; ++i)
          cargo.step(count_moved);
        vehicles = alg.get_all_vehicles();  // TODO Cargo::get_vehicle(id)
        for (Vehicle& vehl : vehicles)
          if (vehl.id() == 1) Vehl1 = &vehl;
        // Checking against the DB
        CHECK(Vehl1->next_node_distance() == 7);
        sop_insert(*Vehl1, *Cust4, schedule, route, Cargo::gtree());
        REQUIRE(route.at(1).first == 2387);
      }

      AND_THEN("t=239, first sop_insert head=2387") {
        int count_moved = 0;
        for (int i = 0; i < 239; ++i)
          cargo.step(count_moved);
        vehicles = alg.get_all_vehicles();  // TODO Cargo::get_vehicle(id)
        for (Vehicle& vehl : vehicles)
          if (vehl.id() == 1) Vehl1 = &vehl;
        // Checking against the DB
        CHECK(Vehl1->next_node_distance() == 0);
        sop_insert(*Vehl1, *Cust4, schedule, route, Cargo::gtree());
        REQUIRE(route.at(1).first == 2387);
      }
    }
  }
}

