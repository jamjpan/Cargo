#pragma once
#include <vector>
#include "libcargo/types.h"
#include "sqlite3/sqlite3.h"
#include <string>

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
  std::vector<long long int> StringToVector(std::string);
  template<typename T>
  std::string VectorToString(std::vector<T>&);

 private:
  sqlite3 *db;
  char *zErrMsg;
  int rc;
};
}  // namespace cargo
