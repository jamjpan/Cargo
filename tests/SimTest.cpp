#include "catch.hpp"

#include "../src/Sim.h"

TEST_CASE("sim size cd1-SR-n10m5-0 is 15 trips") {
    Sim sim("../data/roadnetworks/cd1.gtree",
            "../data/benchmarks/cd1/cd1-SR-n10m5-0",
            "../data/roadnetworks/cd1.rnet",
            1, 12, 1);
    sim.Init();
    REQUIRE(sim.Size()==15);
}

