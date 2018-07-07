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
    Message msg("functions_test");
    msg << "Reading gtree...";
    GTree::load("../data/roadnetwork/tiny.gtree");
    Cargo::gtree() = GTree::get();
    Cargo::vspeed() = 1;
    msg << "Done\n";
}

