#include "catch.hpp"

#include "libcargo.h"

using namespace cargo;

using  msg::Message;
using  msg::MessageType;

TEST_CASE("Router works", "[router]") {
    Message m(MessageType::DEFAULT);
    m << "Router Test Begin" << std::endl;

    GTree::load("../data/roadnetworks/tiny.gtree");
    auto g_ = GTree::getG();
    m << g_.n << " " << g_.m << std::endl;
    auto gtree_ = GTree::get();

    Stop o {1, 0, StopType::CUSTOMER_ORIGIN, 0, 0};
    Stop d {1, 6, StopType::CUSTOMER_DEST, 0, 10};
    Schedule s {o, d};

    Router sr(gtree_);
    Route r;

    int c = sr.RouteThrough(s, r);
    m << "Cost through (0,6) on tiny == 15: " << c << "\nRoute: ";

    for (auto i = r.begin(); i != r.end(); ++i)
        m << *i << " ";
    m << std::endl;

    REQUIRE(c == 15);
}
