// author @J.T. Hu

#include "Customer.h"

namespace cargo {

  // travel time is computed in cargo run class
  Customer::Customer(int origin, int destination, time_t created, time_t travel, int demand) {
    mOrigin = origin;
    mDestination = destination;
    mTimeWindow.StartTime() = createTime;
    mTimeWindow.EndTime() = travelTime;
    mDemand = demand;
  }
}