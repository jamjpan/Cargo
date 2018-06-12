#include "catch.hpp"
#include "libcargo.h"

#include <ctime>
#include <iomanip>
#include <iostream>
// =============================================================================
// Grid TODO
// =============================================================================
using namespace cargo;

TEST_CASE("grid", "[grid]") {
    Message msg("grid test");
    msg << "Reading gtree...";
    GTree::load("../data/roadnetwork/cd1.gtree");
    Cargo::gtree() = GTree::get();
    Cargo::vspeed() = 1;
    msg << "Done\n";
}

