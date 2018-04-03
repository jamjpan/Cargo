// author @J.T. Hu

#pragma once

#include <map>

namespace cargo {

  class Schedule
  {
  public:
    Schedule();
    bool isValid();


  private:
    std::map<double, stop_t> mRoute; // stops sorted by distance
  };
}
