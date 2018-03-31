// author @J.T. Hu

#pragma once

#include <vector>
#include "RoadNet.h"

namespace cargo {

  class Solution
  {
  public:
    Solution(RoadNet& roadNet);
  
  protected:
    RoadNet& mRoadNet;
  };
}