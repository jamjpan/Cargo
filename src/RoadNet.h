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
    RoadNet(std::string road_net_path, std::string gtree_path);
    Vehicle KthVehicle(int origin, int K);
    // gtree only return integer
    double Distance(int origin, int dest);

  private:
    GTree::G_Tree gtree_;
    std::unordered_map<int, node_t> nodes_;
    std::unordered_map<int, edge_t> edges_;
    std::unordered_map<int, Vehicle> vehicles_;
  };
}
