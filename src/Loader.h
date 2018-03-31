// author @J.T. Hu

#pragma once

#include <map>
#include <vector>
#include "Trip.h"

namespace cargo {

  class Loader
  {
  public:
    Loader();
    static void loadTrips(std::map<time_t, vector<Trip>>& trips);

  private:
    std::string mRoadNetPath;
    std::string mTripsPath;
  };
}