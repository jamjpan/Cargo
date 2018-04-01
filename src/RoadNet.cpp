// author @J.T. Hu

#include <fstream>
#include <stdexcept>
#include <algorithm>
#include "RoadNetwork.h"
#include "common.h"

namespace cargo {

  auto keySelector = [](auto veh) { return veh.current; };

  RoadNet::RoadNet(std::string roadNetPath, std::string gTreePath) {
    // init nodes and edges
    std::ifstream ifs(roadNetPath);
    int id, o, d;
    double ox, oy, dx, dy;
    while (ifs >> id >> o >> d >> ox >> oy >> dx >> dy) {
        mNodes[o]  = node_t(ox, oy);
        mNodes[d]  = node_t(dx, dy);
        mEdges[id] = { edge_t(o, d), haversine(Point(ox, oy), Point(dx, dy)) };
    }
    std::printf("Nodes: %lu; Edges: %lu\n", mNodes.size(), mEdges.size());
    ifs.close();
    // init gtree
    GTree::load(gTreePath);
    mGTree = GTree::get();
  }

  Vehicle& RoadNet::getKthVehicle(int origin, int K) {
    // get current locations of vehicles
    std::vector<int> locations(mVehicles.size());
    transform(mVehicles.begin(), mVehicles.end(), locations.begin(), keySelector);

    std::vector<int> knnVehicles = mGTree.KNN(origin, K, locations);
    
    if (knnVehicles.size() < K)
      throw std::runtime_error("no enough vehicles for knn search");
    return mVehicles[knnVehicles[K-1]];
  }
}
