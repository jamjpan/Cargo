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

    int& Origin();
    int& Destination();
    const TimeWindow& getTimeWindow() const { return mTimeWindow; };
    int& Capacity();
    const Schedule& getSchedule() const { return mSchedule; };

    // current node id
    int current;

  private:
    // node index of origin
    int mOrigin;
    // node index of destination
    int mDestination;
    // vehicle's time window
    TimeWindow mTimeWindow;
    // capacity
    int mCapacity;
    // schedule
    Schedule mSchedule;
  };
}
