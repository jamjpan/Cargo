// author @J.T. Hu

#include <fstream>
#include <stdexcept>
#include <algorithm>
#include "RoadNet.h"
#include "Vehicle.h"
#include "common.h"

namespace cargo {

  auto key_selector = [](std::pair<int, Vehicle> pair) { return pair.second.current; };

  RoadNet::RoadNet(std::string road_net_path, std::string gtree_path) {
    // init nodes and edges
    std::ifstream ifs(road_net_path);
    int id, o, d;
    double ox, oy, dx, dy;
    while (ifs >> id >> o >> d >> ox >> oy >> dx >> dy) {
        nodes_[o]  = node_t { ox, oy };
        nodes_[d]  = node_t { dx, dy };
        edges_[id] = edge_t { o, d , haversine(ox, oy, dx, dy) };
    }
    std::printf("Nodes: %lu; Edges: %lu\n", nodes_.size(), edges_.size());
    ifs.close();
    // init gtree
    GTree::load(gtree_path);
    mGTree = GTree::get();
  }

  Vehicle RoadNet::KthVehicle(int origin, int K) {
    // get current locations of vehicles
    std::vector<int> locations(vehicles_.size());
    transform(vehicles_.begin(), vehicles_.end(), locations.begin(), key_selector);

    std::vector<int> knn_vehicles = gtree.KNN(origin, K, locations);
    
    if (knn_vehicles.size() < K)
      throw std::runtime_error("no enough vehicles for knn search");
    return vehicles_[knn_vehicles[K-1]];
  }
}
