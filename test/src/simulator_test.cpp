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
    op.ProblemInstancePath = "../data/benchmarks/a_1.instance";
    op.Scale = 10;
    op.VehicleSpeed = 10;
    op.GPSTiming = 1;

    Simulator *sim = new Simulator();
    Solution *solution = new Solution(sim);
    sim->SetOptions(op);
    sim->SetSolution(solution);
    sim->Initialize();
//    sim->Run();
}

