#include "libcargo.h"
#include "catch.hpp"

using namespace cargo;

SCENARIO("print test-2 intro") {
  std::cout
    << "-----------------------------------------------------------\n"
    << " C A R G O -- Test Route Commit Integrity \n"
    << "-----------------------------------------------------------"
    << std::endl;
}

SCENARIO("route must pass integrity check before commit", "[rsalgorithm.h]") {

  GIVEN("vehicle following route 0>1>194917>114843>114844") {
    Options option;
    std::string path = "/home/jpan/devel/Cargo_benchmark/";
    option.path_to_roadnet = path + "road/bj5.rnet";
    option.path_to_problem = path + "problem/rs-bj5-m5k-c3-d6-s10-x1.0.instance";
    Cargo cargo(option);

    Vehicle VehlA(1, 0, 114844, 0, 500, -3, Cargo::gtree());

    THEN("RSAlgorithm::assign correctly commits an identical route") {
      vec_t<Wayp> good_route = {};
      Wayp a = std::make_pair( 0,      0);
      Wayp b = std::make_pair( 8,      1);
      Wayp c = std::make_pair(15, 194917);
      Wayp d = std::make_pair(22, 114843);
      Wayp e = std::make_pair(27, 114844);
      good_route = { a, b, c, d, e };

      vec_t<Stop> schedule = {};
      Stop A(1,      0, StopType::VehlOrig, 0, 500);
      Stop B(1, 114844, StopType::VehlDest, 0, 500);
      schedule = { A, B };

      MutableVehicle copy(VehlA);
      RSAlgorithm alg("", false);  // empty alg
      REQUIRE(alg.assign({}, {}, good_route, schedule, copy) == true);
    }

    THEN("RSAlgorithm::assign rejects new route with incorrect distances") {
      vec_t<Wayp> bad_route = {};
      // 10 meters have been added to the route.
      Wayp a = std::make_pair(10,      0);
      Wayp b = std::make_pair(18,      1);
      Wayp c = std::make_pair(25, 194917);
      Wayp d = std::make_pair(32, 114843);
      Wayp e = std::make_pair(37, 114844);
      bad_route = { a, b, c, d, e };

      vec_t<Stop> schedule = {};
      Stop A(1,      0, StopType::VehlOrig, 0, 500);
      Stop B(1, 114844, StopType::VehlDest, 0, 500);
      schedule = { A, B };

      MutableVehicle copy(VehlA);
      RSAlgorithm alg("", false);  // empty alg
      REQUIRE(alg.assign({}, {}, bad_route, schedule, copy) == false);
    }
  }
}

