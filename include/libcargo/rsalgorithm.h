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
#ifndef CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_
#define CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "classes.h"
#include "message.h"
#include "types.h"
#include "../sqlite3/sqlite3.h"

namespace cargo {

// RSAlgorithm is a base ridesharing algorithm and demonstrates how algorithms
// can be implemented in Cargo. The listen() method is called continuously in a
// loop. It first retrieves customers and vehicles from the db, then calls
// handle_customer() and handle_vehicle() for each of them. The
// handle_customer() method calls match(). The handle_vehicle() method updates
// the local vehicles_ vector with the new vehicle information. The local
// vehicles_ is a complete copy of the vehicles table in the db. The default
// match() method does nothing and is meant to be overridden.
//
// Batch processing is sometimes desired. To implement it, override listen()
// and add a sleep time corresponding to the batch time.
class RSAlgorithm {
public:
    RSAlgorithm(const std::string&);

    const std::string&          name()                  const;
    bool                        done()                  const;
    void                        kill();
    int&                        batch_time();
    std::vector<Customer>&      waiting_customers();
    std::vector<Vehicle>&       vehicles();
    virtual void                handle_customer(const Customer&);
    virtual void                handle_vehicle(const Vehicle&);
    virtual void                match();

    // The base listen() method will
    //   - poll for active vehicles, then call handle_vehicle() on each
    //     retrieved vehicle
    //   - poll for customers where t_ > early and status is waiting, then call
    //     handle_customer() on each retrieved customer
    virtual void                listen();

    Message print_out;
    Message print_info;
    Message print_warning;
    Message print_error;
    Message print_success;

private:
    std::string name_;
    bool done_;
    int batch_time_; // seconds

    std::vector<Customer> waiting_customers_;
    std::vector<Vehicle> vehicles_;
    std::unordered_set<VehicleId> appeared_vehicles_;

};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_

