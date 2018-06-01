#include "catch.hpp"
#include "libcargo.h"

using namespace cargo;

using msg::Message;
using msg::MessageType;

TEST_CASE("Inserter works", "[inserter]") {
    Message m(MessageType::DEFAULT);
    m << "Inserter test" << std::endl;

    GTree::load("../data/roadnetworks/tiny.gtree");

    SECTION("tiny.gtree is loaded") {
        auto g_ = GTree::getG();
        REQUIRE(g_.n == 9);
        REQUIRE(g_.m == 9);
    }

    SECTION("inserter works") {
        auto gtree_ = GTree::get();
        Inserter ins(gtree_);
        Routel r_;
        Stop o{1, 0, StopType::CUSTOMER_ORIGIN, 0, 0};
        Stop d{1, 6, StopType::CUSTOMER_DEST, 0, 10};
        Stop x{2, 8, StopType::CUSTOMER_ORIGIN, 0, 0};
        Stop y{2, 4, StopType::CUSTOMER_DEST, 0, 10};
        Schedulel s{o, d};
        Schedulel s_new{};
        s_new = ins.Inserter_jaw(s, x, y, r_);

        Routel r {0, 1, 8, 5, 6, 5, 2, 4};
        REQUIRE(r_ == r);

        // Schedulel s_true {o, x, d, y};
        // REQUIRE(s_new == s_true);

        m << "Old schedule: ";
        for (auto i : s)
            m << i.node_id << " ";
        m << "\n";

        m << "New schedule: ";
        for (auto i : s_new)
            m << i.node_id << " ";
        m << "\n";
    }
}
