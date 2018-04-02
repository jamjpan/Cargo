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
    const time_t& StartTime();
    time_t& EndTime();

  private:
    // time window start
    time_t mStartTime;
    // time window end
    time_t mEndTime;
    // maximum waiting time
    time_t mMaxWait;
    // travel time along the shortest path
    time_t mTravelTime;
  };
}