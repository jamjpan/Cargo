// author @J.T. Hu

#pragma once

#include "TimeWindow.h"

namespace cargo {

  class Customer
  {
  public:
    Customer(int origin, int destination, time_t created, time_t travel, int demand);
    ~Customer();
    int& Origin();
    int& Destination();
    TimeWindow& TimeWindow();
    int& Demand() const;
    
  private:
    // node index of origin
    int mOrigin;
    // node index of destination
    int mDestination;
    // customer's time window
    TimeWindow mTimeWindow;
    // seat demand
    int mDemand;
  };
}