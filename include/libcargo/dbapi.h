// MIT License
//
// Copyright (c) 2018 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef CARGO_INCLUDE_LIBCARGO_DBAPI_H_
#define CARGO_INCLUDE_LIBCARGO_DBAPI_H_

#include <string>
#include <vector>

#include "types.h"

#include "../sqlite3/sqlite3.h"

namespace cargo {
namespace db {




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

} // namespace db
} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_DBAPI_H_

