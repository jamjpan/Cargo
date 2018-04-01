// author @J.T. Hu

#pragma once

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
