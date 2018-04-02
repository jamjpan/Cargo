// author @J.T. Hu

#include <fstream>
#include <sstream>
#include <iomanip>
#include "Loader.h"

namespace cargo {

  Loader::Loader(std::string roadNetPath, std::string tripsPath)
    : mRoadNetPath(roadNetPath), mTripsPath(tripsPath) {

  }

  void Loader::loadTrips(std::map<time_t, std::vector<Trip>>& trips) {
    std::ifstream ifs(mTripsPath);
    int id, origin, dest;
    std::string time;
    std::tm tm = {};
    time_t dt;
    while (ifs >> time >> id >> origin >> dest) {
      std::string fullTime = "2015-01-31" + time;
      std::istringstream ss(fullTime);
      ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
      dt = std::mktime(&tm);
      trips[dt].push_back(Trip());
    }
    ifs.close();
  }
}
