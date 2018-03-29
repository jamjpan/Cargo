// author @J.T. Hu

#include "RoadNetwork.h"

namespace cargo {

  RoadNet::RoadNet(std::string roadNetPath, std::string gTreePath) {
    // init nodes
    // init edges
    // init gtree
    GTree::load(gTreePath);
    mGTree = GTree::get();
  }
}