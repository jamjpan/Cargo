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
#ifndef CARGO_INCLUDE_LIBCARGO_CARGO_H_
#define CARGO_INCLUDE_LIBCARGO_CARGO_H_

#include "classes.h"
#include "file.h"
#include "functions.h"
#include "message.h"
#include "options.h"
#include "rsalgorithm.h"
#include "types.h"
#include "../gtree/gtree.h"
#include "../sqlite3/sqlite3.h"

// Cargo is a simulator and requests generator. Its two functionalities are to
// simulate the movement of vehicles, and to generate customer requests and
// vehicles in "real time". These are generated from a "problem instance", a
// special text file that lists customers, vehicles, and their properties.
//
// Vehicle movement is simulated based on vehicle speed, a road network,
// vehicle routes, and time. Speed determines how much distance is covered at
// every simulation timestep. Routes are sequences of nodes through the road
// network. By default, all vehicles travel along the shortest path from their
// origin to their destination.
//
// Cargo will poll an internal sqlite3 database for new vehicle routes. These
// routes can be updated through an RSAlgorithm.
namespace cargo {

class Cargo {
public:
    Cargo(const Options&);
    ~Cargo();
    const BoundingBox&          bbox()                  const;
    const SimTime&              final_request_time()    const;
    const SimTime&              final_arrival_time()    const;
    size_t                      active_vehicles()       const;
    const std::string&          name();
    const std::string&          road_network();
    static Speed&               vspeed()                { return speed_; }
    static SimTime              now()                   { return t_; }
    static GTree::G_Tree&       gtree()                 { return gtree_;}
    static sqlite3*             db()                    { return db_; }
    void                        start(RSAlgorithm&);
    void                        start();

private:
    Message print_out;
    Message print_info;
    Message print_warning;
    Message print_error;
    Message print_success;

    KeyValueNodes nodes_;
    KeyValueEdges edges_; // usage: edges_[from_id][to_id] = weight
    BoundingBox bbox_;
    ProblemSet probset_;

    /* Globally accessible vars */
    static GTree::G_Tree gtree_;
    static sqlite3* db_;
    static Speed speed_;
    static SimTime t_; // current sim time

    SimTime tmin_; // minimum sim duration (max trip.early)
    SimTime tmax_; // maximum sim duration (max vehicle.late)
    SimTime matching_period_;

    size_t active_vehicles_;
    int sleep_interval_;

    SqliteReturnCode rc;
    SqliteErrorMessage err;
    sqlite3_stmt* usn_stmt; // update solution name
    sqlite3_stmt* tim_stmt; // timeout customers
    sqlite3_stmt* ssv_stmt; // select step vehicles
    sqlite3_stmt* dav_stmt; // deactivate vehicle
    sqlite3_stmt* pup_stmt; // pickup
    sqlite3_stmt* drp_stmt; // dropoff
    sqlite3_stmt* vis_stmt; // visitedAt
    sqlite3_stmt* sch_stmt; // schedule
    sqlite3_stmt* lvn_stmt; // last-visited node
    sqlite3_stmt* nnd_stmt; // nearest-node distance

    void initialize(const Options &);

    // Steps active vehicles by their speed.
    // Returns number of stepped vehicles.
    // Outputs number of deactivated vehicles.
    int step(int&);
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_CARGO_H_

