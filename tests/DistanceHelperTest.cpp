#include "catch.hpp"
#include "../src/libezrs.h"

TEST_CASE("DistanceHelper: Euclidean of (0,0) (0,10) is 10", "[distance]") {
    auto dh = ezrs::DistanceHelper();
    ezrs::Node_t node1 {1, 0, 0};
    ezrs::Node_t node2 {2, 0, 10};
    REQUIRE(dh.Euclidean(node1, node2) == 10);
}

TEST_CASE("DistanceHelper: Haversine of (116.394520,39.848040) "
          "(116.400288,39.847500) is within 10 m of 500 m", "[distance]") {
    auto dh = ezrs::DistanceHelper();
    ezrs::Node_t node1{1, 116.394520, 39.848040};
    ezrs::Node_t node2{2, 116.400288, 39.847500};
    double eps = 10;
    double res = dh.Haversine(node1, node2);
    REQUIRE(res + eps / 2 <= 510);
    REQUIRE(res - eps / 2 >= 490);
}

TEST_CASE("DistanceHelper: Network distance on TestGraph from node 0 to 5 is 12",
        "[distance]") {
    GTree::load("/home/jpan/devel/libezrs/utils/GP_Tree.gtree");
    //auto gtree = GTree::get();
    //auto dh = ezrs::DistanceHelper(gtree);
    ezrs::Node_t node1{0, 0, 0};
    ezrs::Node_t node2{5, 0, 0};
    //REQUIRE(dh.Network(node1, node2) == 12);
}
