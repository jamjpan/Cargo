#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// Simulator
// =============================================================================
using namespace cargo;

// ENHANCEMENT: testing cases
//   There is no rigorous test case. A human has to analyze the output of the
//   cargo thread for problems.
//
// POTENTIAL IMPLEMENTATION:
//   Design a test case; dump the db at every time step; assert the dumped
//   states are correct against the test case.
TEST_CASE("Simulator can be initialized and executed", "[simulator]") {

    Options opt;
    // opt.path_to_roadnet = "../data/roadnetwork/tiny.rnet";
    // opt.path_to_gtree = "../data/roadnetwork/tiny.gtree";
    // opt.path_to_edges = "../data/roadnetwork/tiny.edges";
    // opt.path_to_problem = "../data/benchmark/tiny/tiny-n1m2.instance";
    // opt.time_multiplier = 1;
    // opt.vehicle_speed = 4;
    // opt.matching_period = 10;

    opt.path_to_roadnet = "../data/roadnetwork/bj5.rnet";
    opt.path_to_gtree = "../data/roadnetwork/bj5.gtree";
    opt.path_to_edges = "../data/roadnetwork/bj5.edges";
    opt.path_to_problem = "../data/benchmark/rs-lg-4.instance";
    opt.time_multiplier = 10;
    opt.vehicle_speed = 10;
    opt.matching_period = 60;

    Cargo cargo(opt);
    cargo.start();
}

