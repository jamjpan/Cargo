// Copyright (c) 2019 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <iostream>

#include "libcargo.h"
#include "bilateral+/bilateral+.h"
#include "bilateral_arrangement/bilateral_arrangement.h"
#include "grabby/grabby.h"
#include "grasp/grasp.h"
#include "greedy/greedy.h"
#include "kinetic_tree/kinetic_tree.h"
#include "nearest_neighbor/nearest_neighbor.h"
#include "nearest_road/nearest_road.h"
#include "simulated_annealing/simulated_annealing.h"
#include "trip_vehicle_grouping/trip_vehicle_grouping.h"

void print_header() {
  std::cout
    << "-----------------------------------------------------------\n"
    << " C A R G O -- Ridesharing Algorithms Benchmarking Platform \n"
    << "-----------------------------------------------------------"
    << std::endl;
}

void print_usage() {
  std::cout
    << "Interactive:  ./launcher\n"
    << "Command-line: ./launcher selection(1-12) *.rnet *.instance [static(0-1)] [strict(0-1)] \n"
    << std::endl;
}

int main(int argc, char** argv) {
  print_header();
  print_usage();
  Options op;
  std::string selection, roadnetwork, instance;
  bool staticmode = false;
  bool strictmode = false;
  if (argc >= 4 && argc <= 6) {
    vec_t<std::string> args(argv, argv + argc);
    selection = args.at(1);
    roadnetwork = args.at(2);
    instance = args.at(3);
    if (argc >= 5) staticmode = (args.at(4) == "0" ? false : true);
    if (argc == 6) strictmode = (args.at(5) == "0" ? false : true);
  } else {
    if (argc > 6) {
      std::cout << "Too many arguments!" << std::endl;
      print_usage();
    }
    std::cout
      << "Make a selection.\n"
      << "  Search Algorithms:\n"
      << "    1) bilateral+               4) greedy\n"
      << "    2) bilateral_arrangement    5) kinetic_tree\n"
      << "    3) grabby                   6) nearest_neighbor\n"
      << "  Join Algorithms:\n"
      << "    7) grasp4                  10) sa100\n"
      << "    8) grasp16                 11) trip_vehicle_grouping\n"
      << "    9) sa50\n"
      << "  Other:\n"
      << "    12) nearest_road\n"
      << "\n"
      << "Your selection (1-12): ";
    std::cin >> selection;
    std::cout << "Path to rnet (*.rnet): ";
    std::cin >> roadnetwork;
    std::cout << "Path to instance (*.instance): ";
    std::cin >> instance;
  }

  op.path_to_roadnet = roadnetwork;
  op.path_to_problem = instance;
  op.static_mode = staticmode;
  op.strict_mode = strictmode;
  Cargo cargo(op);

  if (selection == "1") {
    BilateralPlus alg("bp_"+cargo.name()); cargo.start(alg);
  } else if (selection == "2") {
    BilateralArrangement alg("ba_"+cargo.name()); cargo.start(alg);
  } else if (selection == "3") {
    Grabby alg("gb_"+cargo.name()); cargo.start(alg);
  } else if (selection == "4") {
    Greedy alg("gr_"+cargo.name()); cargo.start(alg);
  } else if (selection == "5") {
    KineticTrees alg("kt_"+cargo.name()); cargo.start(alg);
  } else if (selection == "6") {
    NearestNeighbor alg("nn_"+cargo.name()); cargo.start(alg);
  } else if (selection == "7") {
    GRASP alg("gp4_"+cargo.name(), 4); cargo.start(alg);
  } else if (selection == "8") {
    GRASP alg("gp16_"+cargo.name(), 16); cargo.start(alg);
  } else if (selection == "9") {
    SimulatedAnnealing alg("sa50_"+cargo.name(), 50); cargo.start(alg);
  } else if (selection == "10") {
    SimulatedAnnealing alg("sa100_"+cargo.name(), 100); cargo.start(alg);
  } else if (selection == "11") {
    TripVehicleGrouping alg("tg_"+cargo.name()); cargo.start(alg);
  } else if (selection == "12") {
    NearestRoad alg("nr_"+cargo.name()); cargo.start(alg);
  }
}

