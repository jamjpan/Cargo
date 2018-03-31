// author @J.T. Hu

#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include "gtree/GTree.h"
#include "common.h"

namespace cargo {

  class RoadNet
  {
  public:
    RoadNet(std::string roadNetPath, std::string gTreePath);
    
  private:
    GTree::G_Tree mGTree;
    std::unordered_map<int, node_t> mNodes;
    std::unordered_map<int, edge_t> mEdges;
  };
}