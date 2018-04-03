// author @J.T. Hu

#include <fstream>
#include <sstream>
#include <iomanip>
#include "Loader.h"

namespace cargo {

  Loader::Loader(std::string road_net_path, std::string trips_path)
    : road_net_path_(road_net_path), trips_path_(trips_path) {

  }

  void Loader::loadTrips(std::map<time_t, std::vector<Trip>>& trips) {
    std::ifstream ifs(trips_path_);
    int id, origin, dest;
    std::string time;
    std::tm tm = {};
    time_t dt;
    while (ifs >> time >> id >> origin >> dest) {
      std::string full_time = "2015-01-31" + time;
      std::istringstream ss(full_time);
      ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
      dt = std::mktime(&tm);
      trips[dt].push_back(Trip());
    }
    ifs.close();
  }
}
