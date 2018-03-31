// author @J.T. Hu

#pragma once

#include <string>
#include <map>
#include <vector>
#include "Solution.h"
#include "common.h"

namespace cargo {

  class Cargo
  {
  public:
    std::string GTreePath = BJ_GTREE;
    std::string RoadNetPath;
    std::string TripsPath = BJ_TPIPS;
    static int NumberOfVehicles = 4000;

    Cargo(int argc, const char* argv[]);
    RoadNet& RoadNet() { return mRoadNet; };
    void printUsage();
    void run(Solution* solution);

  private:
    RoadNet mRoadNet;
    std::map<time_t, vector<Trip>> mTrips;
  };
}