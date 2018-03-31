// author @J.T. Hu

#include <fstream>
#include "RoadNetwork.h"
#include "common.h"

namespace cargo {

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
}