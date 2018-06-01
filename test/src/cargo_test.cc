#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// Simulator
// =============================================================================
using namespace cargo;

TEST_CASE("Simulator can be initialized and executed", "[simulator]") {
    Options opt;
    opt.path_to_roadnet = "../data/roadnetwork/cd1.rnet";
    opt.path_to_gtree = "../data/roadnetwork/cd1.gtree";
    opt.path_to_edges = "../data/roadnetwork/cd1.edges";
    opt.path_to_problem = "../data/benchmark/tiny/tiny-n1m2.instance";
    opt.time_multiplier = 10;
    opt.vehicle_speed = 10;
    opt.gps_frequency = 1;

    Cargo cargo(opt);
    cargo.run();
}

