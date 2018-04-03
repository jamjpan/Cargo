// authro @J.T. Hu

#pragma once

#include "TimeWindow.h"
#include "Schedule.h"

namespace cargo {

  class Vehicle
  {
  public:
    // for /usr/include/c++/5/tuple:1172:70: error: no matching function for call to ‘cargo::Vehicle::Vehicle()’
    Vehicle();
    Vehicle(int origin, int destination, time_t leaving, time_t arrival, int capacity);

    int origin const () { return origin_; };
    int destination const () { return destination_; };
    int capacity const () { return capacity_; };
    const TimeWindow& time_window() const { return time_window_; };
    const Schedule& schedule() const { return schedule_; };

    // current node id
    int current_;

  private:
    // node index of origin
    int origin_;
    // node index of destination
    int destination_;
    // vehicle's time window
    TimeWindow time_window_;
    // capacity
    int capacity_;
    // schedule
    Schedule schedule_;
  };
}
