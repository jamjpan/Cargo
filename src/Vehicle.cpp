// author @J.T. Hu

#include "Vehicle.h"

namespace cargo {

  Vehicle::Vehicle(int origin, int destination, time_t leaving, time_t arrival, int capacity) {
    mOrigin = origin;
    mDestination = destination;
    mTimeWindow.StartTime() = leaving;
    mTimeWindow.EndTime() = arrival;
    mCapacity = capacity;
  }
  
}