// author @J.T. Hu

#include "Customer.h"

namespace cargo {

  // travel time is computed in cargo run class
  Customer::Customer(int id, int origin, int destination, time_t created, time_t travel, int demand)
    : mTimeWindow(created, travel) {
    mId = id;
    mOrigin = origin;
    mDestination = destination;
    mDemand = demand;
  }
}
