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
    Loader(std::string road_net_path, std::string tripsPath);
    void loadTrips(std::map<time_t, std::vector<Trip> >& trips);

  private:
    std::string road_net_path_;
    std::string trips_path_;
  };
}
