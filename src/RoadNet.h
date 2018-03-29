// author @J.T. Hu

#pragma once

#include <vector>
#include <string>
#include "gtree/GTree.h"

namespace cargo {

  class RoadNet
  {
  public:
    RoadNet(std::string roadNetPath, std::string gTreePath);
    ~RoadNet();
    
  private:
    GTree::G_Tree mGTree;
    vector<int, node_t> mNodes;
    vecotr<int, edge_t> mEdges;
  };
}