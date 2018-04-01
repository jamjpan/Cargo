// author @J.T. Hu

#pragma once

#include <vector>
#include <unordered_map>
#include "RoadNet.h"
#include "Vehicle.h"
#include "Customer.h"

namespace cargo {

  class Solution
  {
  public:
    Solution(RoadNet& roadNet);

    bool isValid(std::vector<stop_t> stops);

    virtual int assign(const Customer& customer);

  protected:
    RoadNet& mRoadNet;
    std::unordered_map<int, Vehicle> mVehicles;
  };
}
