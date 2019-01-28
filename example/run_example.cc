#include "libcargo.h"
#include "bilateral+/bilateral+.h"
#include "bilateral_arrangement/bilateral_arrangement.h"
#include "grabby/grabby.h"

int main() {
  Options option;
  option.path_to_roadnet  = "/home/jpan/devel/Cargo_benchmark/road/bj5_directed.rnet";
  option.path_to_problem  = "/home/jpan/devel/Cargo_benchmark/problem/rs-bj5-m5k-c3-d6-s10-x1.0.instance";
  option.vehicle_speed    = 10;
  // TODO: auto parse road network and speed from problem
  Cargo cargo(option);
  Grabby gr;
  cargo.start(gr);
}

