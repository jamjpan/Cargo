#include "catch.hpp"

#include "libcargo.h"

using namespace cargo;

using opts::Options;

// =============================================================================
// Simulator
// =============================================================================
TEST_CASE("Simulator can be initialized and executed", "[simulator]") {
    Options op;
    op.RoadNetworkPath = "../data/roadnetworks/cd1.rnet";
    op.GTreePath = "../data/roadnetworks/cd1.gtree";
    op.EdgeFilePath = "../data/roadnetworks/cd1.edges";
    op.ProblemInstancePath = "../data/benchmarks/cd1/cd1-SR-n10m5-0";
    op.Scale = 1;
    op.VehicleSpeed = 10;

    Simulator sim;
    sim.SetOptions(op);
    sim.Initialize();
    //sim.Run();
}

