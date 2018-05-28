#pragma once
#include "libcargo/types.h"
#include "sqlite3/sqlite3.h"
#include <string>
#include <vector>

namespace cargo {
class DA {
public:
    DA();
    ~DA();
    int AddVehicle(Vehicle *);
    // vid lv_node nnd
    int UpdateLocation(VehicleId, int, int);
    int UpdateSchedule(VehicleId, Schedule *, Route *);
    int UpdateStop(VehicleId, int, SimTime);
    int InsertRequest(Trip *);
    int GetOneRequest(Trip *);
    int GetNRequest(std::vector<Trip *> &, int);
    std::vector<long long int> StringToVector(std::string);
    template <typename T> std::string VectorToString(std::vector<T> &);

private:
    sqlite3 *db;
    char *zErrMsg;
    int rc;
};
} // namespace cargo
