#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// Simulator
// =============================================================================
using namespace cargo;

TEST_CASE("Simulator can be initialized and executed", "[simulator]") {

    Options opt;
    opt.path_to_roadnet = "../data/roadnetwork/tiny.rnet";
    opt.path_to_gtree = "../data/roadnetwork/tiny.gtree";
    opt.path_to_edges = "../data/roadnetwork/tiny.edges";
    opt.path_to_problem = "../data/benchmark/tiny/tiny-n1m2.instance";
    opt.time_multiplier = 1;
    opt.vehicle_speed = 4;
    opt.matching_period = 60;

    Cargo cargo(opt);
    cargo.start();
}

