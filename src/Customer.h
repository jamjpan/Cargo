// author @J.T. Hu

#pragma once

#include "TimeWindow.h"

namespace cargo {

  class Customer
  {
  public:
    Customer(int id, int origin, int destination, time_t created, time_t travel, int demand);
    const int& Id() const { return mId; };
    const int& Origin() const { return mOrigin; };
    const int& Destination() const { return mDestination; };
    const TimeWindow& getTimeWindow() const { return mTimeWindow; };
    const int& Demand() const { return mDemand; };

  private:
    int mId;
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
