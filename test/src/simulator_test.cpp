#include "catch.hpp"

#include "libcargo.h"

using namespace cargo;
using opts::Options;

TEST_CASE("Simulator can be initialized and executed", "[Simulator]") {
    Options op;
    op.RoadNetworkPath = "../data/roadnetworks/cd1.rnet";
    op.GTreePath = "../data/roadnetworks/cd1.gtree";
    op.EdgeFilePath = "../data/roadnetworks/cd1.edges";
    op.ProblemInstancePath = "../data/benchmarks/cd1/cd1-SR-n10m5-0";

    Simulator sim;
    sim.SetOptions(op);
    sim.Initialize();
}

