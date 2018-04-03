// author @J.T. Hu

#include "Vehicle.h"

namespace cargo {

  Vehicle::Vehicle(int origin, int destination, time_t leaving, time_t arrival, int capacity)
    : time_window_(TimeWindow(leaving, arrival)) {
    origin_ = origin;
    destination_ = destination;
    capacity_ = capacity;
  }
  
}