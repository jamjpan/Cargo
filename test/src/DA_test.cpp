#include "catch.hpp"

#include <chrono>
#include <iostream>
#include "libcargo.h"

using namespace cargo;

TEST_CASE("DA works", "[DA]")
{
  DA da;
  ProblemInstance pi;
  file::ReadProblemInstance("../data/dataset_500+1000_0", pi);

  auto t_start = std::chrono::high_resolution_clock::now();

  for (const auto &kv : pi.trips) {
    for (const auto &trip : kv.second) {
      if (trip.demand < 1) {
        Vehicle *v = new Vehicle(trip);
        da.AddVehicle(v);
      }
    }
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  std::cout
      << "time cost: "
      << std::chrono::duration<double, std::milli>(t_end - t_start).count()
      << "ms" << std::endl;

  std::string test_string = "1,2,3,4,5,";
  std::vector<long long int> v = da.StringToVector(test_string);
  for (long long int i : v) {
    std::cout << i << std::endl;
  }
  std::cout << da.VectorToString(v) << std::endl;
}
