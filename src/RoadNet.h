// author @J.T. Hu

#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include "gtree/GTree.h"
#include "common.h"
#include "Vehicle.h"

namespace cargo {

  class RoadNet
  {
  public:
    RoadNet(std::string roadNetPath, std::string gTreePath);
    Vehicle getKthVehicle(int origin, int K);
    // gtree only return integer
    double getDistance(int origin, int dest);

  private:
    GTree::G_Tree mGTree;
    std::unordered_map<int, node_t> mNodes;
    std::unordered_map<int, edge_t> mEdges;
    std::unordered_map<int, Vehicle> mVehicles;
    std::vector<int> mVehicleIds;
  };
}
