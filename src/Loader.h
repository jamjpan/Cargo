// author @J.T. Hu

// maybe loader class is not needed

#pragma once

#include <map>
#include <vector>
#include "Trip.h"

namespace cargo {

  class Loader
  {
  public:
    Loader();
    static void loadTrips(std::map<time_t, std::vector<Trip>>& trips);

  private:
    std::string mRoadNetPath;
    std::string mTripsPath;
  };
}