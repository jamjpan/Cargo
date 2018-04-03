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
    Solution(RoadNet& road_net);

    bool IsValid(std::vector<stop_t> stops);

    virtual int Assign(const Customer& customer);

  protected:
    RoadNet& road_net_;
    std::unordered_map<int, Vehicle> vehicles_;
  };
}
