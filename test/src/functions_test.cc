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
  Stop a1(1, 0, StopType::VehlOrig, 0, 0);
  Stop a2(1, 0, StopType::VehlDest, 0, 0);
  Stop b1(2, 1, StopType::CustOrig, 0, 0);
  Stop b2(2, 1, StopType::CustDest, 0, 0);

  SECTION("randcust") {
    std::vector<Stop> sch1 {a1, a2};
    REQUIRE(randcust(sch1) == -1);
    std::vector<Stop> sch2 {a1, b1, b2, a2};
    REQUIRE(randcust(sch2) == 2);
  }

  SECTION("pickup_range") { /* TODO */ }
  SECTION("route_through") { /* TODO */ }
  SECTION("chkpc") { /* TODO */ }
  SECTION("chktw") { /* TODO */ }
  SECTION("sop_insert") { /* TODO */ }
  SECTION("sop_replace") { /* TODO */ }
}

