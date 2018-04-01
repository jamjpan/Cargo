// authro @J.T. Hu

#pragma once

#include "TimeWindow.h"
#include "Schedule.h"

namespace cargo {

  class Vehicle
  {
  public:
    Vehicle(int origin, int destination, time_t leaving, time_t arrival, int capacity);
    ~Vehicle();
    int& Origin();
    int& Destination();
    const TimeWindow& getTimeWindow() const { return mTimeWindow; };
    int& Capacity();
    const Schedule& getSchedule() const { return mSchedule; };

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
