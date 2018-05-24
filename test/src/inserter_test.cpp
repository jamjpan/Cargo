#include "catch.hpp"
#include "libcargo.h"

using namespace cargo;

using msg::Message;
using msg::MessageType;

TEST_CASE("Inserter works", "[inserter]") {
    Message m(MessageType::DEFAULT);
    m << "Inserter test" << std::endl;

    GTree::load("../data/roadnetworks/tiny.gtree");
    auto g_ = GTree::getG();
    m << g_.n << " " << g_.m << std::endl;
    auto gtree_ = GTree::get();


    Stop o {1, 0, StopType::CUSTOMER_ORIGIN, 0, 0};
    Stop d {1, 6, StopType::CUSTOMER_DEST, 0, 10};
    Schedulel s {o, d};

    Inserter ins(gtree_);
    Routel r;

    Stop x {2, 8, StopType::CUSTOMER_ORIGIN, 0, 0};
    Stop y {2, 4, StopType::CUSTOMER_DEST, 0, 10};
    Schedulel s_new;
    s_new = ins.Inserter_jaw(s, x, y, r);

    m << "Old schedule: ";
    for (auto i : s)
        m << i.node_id << " ";
    m << "\n";

    m << "New schedule: ";
    for (auto i : s_new)
        m << i.node_id << " ";
    m << "\n";

    m << "Route: ";
    for (auto i : r)
        m << i << " ";
    m << std::endl;
}
