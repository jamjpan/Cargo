// author @J.T. Hu

#pragma once

#include "TimeWindow.h"

namespace cargo {

  class Customer
  {
  public:
    Customer(int id, int origin, int destination, time_t created, time_t travel, int demand);
    int id() const { return id_; };
    int origin() const { return origin_; };
    int destination() const { return destination_; };
    int demand() const { return mDemand; };
    const TimeWindow& time_window() const { return time_window_; };

  private:
    int id_;
    // node index of origin
    int origin_;
    // node index of destination
    int destination_;
    // customer's time window
    TimeWindow time_window_;
    // seat demand
    int demand_;
  };
}
