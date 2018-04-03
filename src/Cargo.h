// author @J.T. Hu

#pragma once

#include <string>
#include <map>
#include <vector>
#include "common.h"
#include "Solution.h"
#include "Trip.h"

namespace cargo {

  class Cargo
  {
  public:
    std::string gtree_path_ = BJ_GTREE;
    std::string RoadNetPath;
    std::string trips_path_ = BJ_TRIPS;
    int number_of_vehicles = 4000;

    Cargo(int argc, const char* argv[]);
    RoadNet& road_net() { return road_net_; };
    void print_usage();
    void run(Solution* solution);

  private:
    time_t start_;
    int duration_;
    RoadNet road_net_;
    std::map<time_t, std::vector<Trip> > trips_;
  };
}
