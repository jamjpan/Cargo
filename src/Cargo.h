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
    std::string GTreePath = BJ_GTREE;
    std::string RoadNetPath;
    std::string TripsPath = BJ_TRIPS;
    int NumberOfVehicles = 4000;

    Cargo(int argc, const char* argv[]);
    RoadNet& getRoadNet() { return mRoadNet; };
    void printUsage();
    void run(Solution* solution);

  private:
    time_t mStart;
    int mDuration;
    RoadNet mRoadNet;
    std::map<time_t, std::vector<Trip> > mTrips;
  };
}
