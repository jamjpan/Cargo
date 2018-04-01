// author @J.T. Hu

#include <fstream>
#include <iomanip>
#include "Loader.h"

namespace cargo {

  Loader::Loader(std::string roadNetPath, std::string tripsPath)
    : mRoadNetPath(roadNetPath), mTripsPath(tripsPath) {

  }

  void Loader::loadTrips(std::map<time_t, std::vector<Trips>>& trips) {
    std::ifstream ifs(mTripsPath);
    int id, origin, dest;
    std::string time;
    std::tm tm = {};
    time_t dt;
    while (ifs >> time >> id >> origin >> dest) {
      std::istringstream ss("2015-01-31" + time);
      ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
      dt = std::mktime(&tm);
      trips[time_t].push_back(Trip());
    }
    ifs.close();
  }
}
