#include "catch.hpp"

#include "libcargo.h"
#include <chrono>
#include <iostream>

using namespace cargo;

TEST_CASE("DA works", "[DA]") {
    GTree::load("../data/gtree/cd1.gtree");
    GTree::G_Tree gtree = GTree::get();

    DA da;
    ProblemInstance pi;
    file::ReadProblemInstance("../data/benchmark/rs-lg-2.instance", pi);

    auto t_start = std::chrono::high_resolution_clock::now();

    for (const auto &kv : pi.trips) {
        for (const auto &trip : kv.second) {
            if (trip.demand < 1) {
                Vehicle *v = new Vehicle(trip);
                gtree.find_path(v->oid, v->did, v->route);
                Stop o = {v->id, v->oid, StopType::VEHICLE_ORIGIN, -1,
                          trip.early};
                Stop d = {v->id, v->did, StopType::VEHICLE_DEST, -1, trip.late};
                v->sched.push_back(o);
                v->sched.push_back(d);
                da.AddVehicle(v);
            } else {
            }
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout
        << "time cost: "
        << std::chrono::duration<double, std::milli>(t_end - t_start).count()
        << "ms" << std::endl;

    // std::string test_string = "1,2,3,4,5,";
    // std::vector<long long int> v = da.StringToVector(test_string);
    // for (long long int i : v) {
    //     std::cout << i << std::endl;
    // }
    // std::cout << da.VectorToString(v) << std::endl;
}
