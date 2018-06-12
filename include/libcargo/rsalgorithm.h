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

// RSAlgorithm is a base ridesharing algorithm.
//   * listen() is called continuously. It retrieves customers and vehicles from
//     the db, calls handle_customer() and handle_vehicle(), match(), then sleeps.
//   * Sleep time (batch_time()) is overridable.
//   * vehicles() and waiting_customers() gives access to vehicles and customers
//     at each simulation step.
//       - sql::select_matchable_vehicles selects vehicles
//       - sql::select_waiting_customers selects customers
//       - both can be overriden with custom sql; implement the custom sql
//         against the global Cargo::db() database object, and call the sql in
//         a custom listen()
class RSAlgorithm {
public:
    RSAlgorithm(const std::string&);

    const std::string&          name()                  const;
    bool                        done()                  const;
    void commit(const Customer, const Vehicle, std::vector<cargo::Waypoint>,
                const std::vector<cargo::Stop>)         const;
    static bool                 committing()            { return committing_; }
    void                        kill();
    /* Set batch_time() = 0 for streaming mode */
    int&                        batch_time();
    /* Retrieve customers/vehicles */
    std::vector<Customer>&      waiting_customers();
    std::vector<Vehicle>&       vehicles();
    /* Overridable */
    virtual void                handle_customer(const Customer);
    virtual void                handle_vehicle(const Vehicle);
    virtual void                match();
    virtual void                end();
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
    static bool committing_; // "lock"

    std::vector<Customer> waiting_customers_;
    std::vector<Vehicle> vehicles_;
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_

