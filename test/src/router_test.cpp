#include "catch.hpp"
#include "libcargo.h"

using namespace cargo;

using msg::Message;
using msg::MessageType;

TEST_CASE("Router works", "[router]") {
    GTree::load("../data/roadnetworks/tiny.gtree");

    WHEN("tiny.gtree is loaded") {
        auto g_ = GTree::getG();
        REQUIRE(g_.n == 9);
        REQUIRE(g_.m == 9);
    }

    WHEN("shortest path works") {
        auto gtree_ = GTree::get();
        Stop o{1, 0, StopType::CUSTOMER_ORIGIN, 0, 0};
        Stop d{1, 6, StopType::CUSTOMER_DEST, 0, 10};
        Schedulel s{o, d};
        Router sr(gtree_);
        Routel r_;
        Routel r{0, 1, 2, 5, 6};
        int c = sr.RouteThrough(s, r_);
        REQUIRE(c == 15);
        REQUIRE(r_ == r);
    }
}
