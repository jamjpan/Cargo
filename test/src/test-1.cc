#include "libcargo.h"
#include "catch.hpp"

using namespace cargo;

SCENARIO("print test-1 intro") {
  std::cout
    << "-----------------------------------------------------------\n"
    << " C A R G O -- Distance and Route Tests \n"
    << "-----------------------------------------------------------"
    << std::endl;
}

SCENARIO("distance and route functions give correct results", "[distance.h, functions.h]") {

  GIVEN("bj5 and standard instance") {
    Options option;
    std::string path = "/home/jpan/devel/Cargo_benchmark/";
    option.path_to_roadnet = path + "road/bj5.rnet";
    option.path_to_problem = path + "problem/rs-bj5-m5k-c3-d6-s10-x1.0.instance";
    Cargo cargo(option);

    CHECK(Cargo::node2pt(0).lng == 116.303837);
    CHECK(Cargo::node2pt(0).lat == 39.985677);
    CHECK(Cargo::node2pt(114844).lng == 116.304096);
    CHECK(Cargo::node2pt(114844).lat == 39.985827);

    THEN("haversine from 0~114844 is within 1m") {
      // http://www.movable-type.co.uk/scripts/latlong.html
      REQUIRE(haversine(0, 114844) == haversine(114844, 0));
      REQUIRE(haversine(0, 114844) == Approx(27.66).margin(1));
    }

    THEN("get_shortest_path from 0~0 is 0>0") {
      vec_t<Wayp> path;
      DistInt cost = get_shortest_path(0, 0, path);
      REQUIRE(path.size() == 2);
      REQUIRE(path.at(0).first  == 0);
      REQUIRE(path.at(0).second == 0);
      REQUIRE(path.at(1).first  == 0);
      REQUIRE(path.at(1).second == 0);
      REQUIRE(cost == 0);

      AND_THEN("route_through from 0~0 gives same route and cost") {
        Stop A(1, 0, StopType::VehlOrig, 0, 100);
        Stop B(1, 0, StopType::VehlDest, 0, 100);
        vec_t<Stop> schedule = { A, B };
        vec_t<Wayp> route = {};
        DistInt route_cost = route_through(schedule, route);
        REQUIRE(route == path);
        REQUIRE(route_cost == cost);
      }
    }

    // Cannot capture the following case, but just to document that it happens
    // due to gtree invalid array access
    // THEN("get_shortest_path from 0~-1 throws SIGSEGV") {}
    // THEN("get_shortest_path from 0~999999 throws find_path error") {}

    THEN("get_shortest_path from 0~114844 is 0>1>194917>114843>114844") {
      vec_t<Wayp> real_route = {};
      Wayp a = std::make_pair( 0,      0);
      Wayp b = std::make_pair( 8,      1);
      Wayp c = std::make_pair(15, 194917);
      Wayp d = std::make_pair(22, 114843);
      Wayp e = std::make_pair(27, 114844);
      real_route = { a, b, c, d, e };

      vec_t<Wayp> path;
      get_shortest_path(0, 114844, path);
      REQUIRE(path.size() == 5);
      REQUIRE(path == real_route);

      AND_THEN("get_shortest_path from 0~114844 is within 5m") {
        // G-tree uses integer distances but haversine result has some precision.
        // I think g-tree rounding is to nearest meter, thus should be off by at
        // most 1m per segment.
        vec_t<Wayp> path;
        DistInt cost = get_shortest_path(0, 114844, path);
        float real_cost = 0;
        real_cost += haversine(path.at(0).second, path.at(1).second);
        real_cost += haversine(path.at(1).second, path.at(2).second);
        real_cost += haversine(path.at(2).second, path.at(3).second);
        real_cost += haversine(path.at(3).second, path.at(4).second);
        REQUIRE(cost == Approx(real_cost).margin(5));

        AND_THEN("route_through from 0~194917~114844 gives same route and cost") {
          Stop A(1,      0, StopType::VehlOrig, 0, 100);
          Stop B(1, 194917, StopType::CustDest, 0, 100);
          Stop C(1, 114844, StopType::VehlDest, 0, 100);
          vec_t<Stop> schedule = { A, B, C };
          vec_t<Wayp> route = {};
          DistInt route_cost = route_through(schedule, route);
          REQUIRE(route == path);
          REQUIRE(route_cost == cost);

          AND_THEN("route_through from 0~194917~194917~114844 gives 0>1>194917>194917>114843>114844 and same cost") {
            Stop D(1, 194917, StopType::CustOrig, 0, 100);
            schedule = { A, D, B, C};
            route = {};
            route_cost = route_through(schedule, route);
            vec_t<Wayp> this_route = { a, b, c, c, d, e };
            REQUIRE(route == this_route);
            REQUIRE(route_cost == cost);
          }

          AND_THEN("route_through from 0~0~194917~114844 gives 0>0>1>194917>114843>114844 and same cost") {
            Stop D(1, 0, StopType::CustOrig, 0, 100);
            schedule = { A, D, B, C};
            route = {};
            route_cost = route_through(schedule, route);
            vec_t<Wayp> this_route = { a, a, b, c, d, e };
            REQUIRE(route == this_route);
            REQUIRE(route_cost == cost);
          }

          AND_THEN("route_through from 0~194917~114844~114844 gives 0>1>194917>114843>114844>114844 and same cost") {
            Stop B(1, 194917, StopType::CustOrig, 0, 100);
            Stop D(1, 114844, StopType::CustDest, 0, 100);
            schedule = { A, B, D, C};
            route = {};
            route_cost = route_through(schedule, route);
            vec_t<Wayp> this_route = { a, b, c, d, e, e };
            REQUIRE(route == this_route);
            REQUIRE(route_cost == cost);
          }
        }

      }
    }

  }
}

