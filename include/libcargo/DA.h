#pragma once
#include "libcargo/types.h"
#include "sqlite3/sqlite3.h"

namespace cargo
{
class DA
{
 public:
  DA();
  ~DA();
  int AddVehicle(Vehicle *);
  int UpdateLocation(VehicleId, int, int);
  int UpdateSchedule(VehicleId, Schedule *, Route *);
  int UpdateStop(VehicleId, int, SimTime);

 private:
  sqlite3 *db;
  char *zErrMsg;
  int rc;
};
}  // namespace cargo