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

// The abstract interface for ridesharing algorithms. Users can implement the
// handle_customer(), handle_vehicle(), match(), end(), and listen() methods.
//
// Only listen() has a default behavior. The behavior is to select all active
// vehicles into vehicles_ (retrievable with vehicles()), select all waiting
// customers into waiting_customers_ (retrievable with waiting_customers()),
// then sleep for batch_time_ (settable with batch_time()). The method is
// called continuously inside Cargo::step().
class RSAlgorithm {
public:
    // Pass along a name string to your RSAlgorithm
    RSAlgorithm(const std::string&);

    virtual void                handle_customer(const Customer &);
    virtual void                handle_vehicle(const Vehicle &);
    virtual void                match();
    virtual void                end();
    virtual void                listen();

    // Write assignment to the db
    void commit(const Customer &,
                const MutableVehicle &)                 const;
    void commit(const Customer &, const Vehicle &,
                const std::vector<Waypoint> &,
                const std::vector<Stop> &)       const;

    const std::string&          name()                  const;
    bool                        done()                  const;
    static bool                 committing()            { return committing_; }
    void                        kill();         // <-- sets done_ to true
    int&                        batch_time();   // <-- set to 0 for streaming

    // Customers and vehicles are refreshed whenever listen() is called
    std::vector<Customer>&      waiting_customers();
    std::vector<Vehicle>&       vehicles();

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

