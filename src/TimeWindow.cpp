#include "TimeWindow.h"
#include "Cargo.h"

namespace cargo {

  TimeWindow::TimeWindow(time_t createTime, time_t travelTime) {
    mStartTime = createTime;
    mEndTime = travelTime + Cargo::MaxWait;
  }
}