// author @J.T. Hu

#pragma once

#include <ctime>

namespace cargo {

  class TimeWindow
  {
  public:
    // TimeWindow();
    TimeWindow(time_t, time_t);
    ~TimeWindow();
    time_t start_time const () { return start_time_; };
    time_t end_time const () { return end_time_; };

  private:
    // time window start
    time_t start_time_;
    // time window end
    time_t end_time_;
    // maximum waiting time
    time_t max_wait_;
    // travel time along the shortest path
    time_t travel_time_;
  };
}