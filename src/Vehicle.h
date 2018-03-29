// authro @J.T. Hu

#pragma once

#include "TimeWindow.h"

namespace cargo {

  class Vehicle
  {
  public:
    Vehicle(int origin, int destination, time_t leaving, time_t arrival, int capacity);
    ~Vehicle();
    int& Origin();
    int& Destination();
    TimeWindow& mTimeWindow();
    int& Capacity();
    
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