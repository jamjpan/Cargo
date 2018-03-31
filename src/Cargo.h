// author @J.T. Hu

#pragma once

#include <string>
#include "Solution.h"

namespace cargo {

  class Cargo
  {
  public:
    std::string GTreePath;
    std::string RoadNetPath;
    static int NumberOfVehicles = 4000;

    Cargo(int argc, const char* argv[]);
    RoadNet& RoadNet() { return mRoadNet; };
    void printUsage();
    void run(Solution* solution);

  private:
    RoadNet mRoadNet;
  };
}