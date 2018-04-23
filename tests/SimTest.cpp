#include "catch.hpp"

#include "../src/Sim.h"

TEST_CASE("hello world") {
    Sim sim("../data/roadnetworks/cd1.gtree",
            "../data/benchmarks/cd1/cd1-SR-n10m5-0",
            "../data/roadnetworks/cd1.rnet",
            1, 12, 1);
    sim.Init();
    REQUIRE(1==1);
}

