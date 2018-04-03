#include "TimeWindow.h"
#include "Cargo.h"

namespace cargo {

  TimeWindow::TimeWindow(time_t created, time_t travel) {
    start_time_ = created;
    end_time_ = created + travel; // need another param accpetable dropoff
  }
}