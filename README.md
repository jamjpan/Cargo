# Cargo - Ridesharing Simulation & Algorithms Library

Dynamic ridesharing is a type of vehicle routing problem (VRP) closely related
to the variants known as PDPTW (or VRPPDTW) and DARP (dial-a-ride).

Cargo is a C++11 library that provides a set of abstractions to make it easy to
develop and test ridesharing algorithms.

### Develop algorithms using built-in classes and functions
* Built-in classes: Stop, Schedule, Route, Trip, Customer, Vehicle, etc.
* Built-in functions: constraints checking, schedule insertion, schedule routing,
  shortest-paths, etc.
* Five overridable methods:
  - `RSAlgorithm::handle_customer()`: called for every customer request
  - `RSAlgorithm::handle_vehicle()`: called whenever a new vehicle appears
  - `RSAlgorithm::match()`: called at some configurable frequency
  - `RSAlgorithm::listen()`: fine-tune an algorithm's polling (batch vs streaming)
  - `RSAlgorithm::end()`: called at the end of the simulation

### Test algorithms on the Cargo real-time simulation platform
* Simulation state is stored in an in-memory Sqlite3 database
* `Cargo::step()` runs every second (configurable) to update locations of vehicles
* `RSAlgorithm::listen()` polls every second (configurable) for new vehicles/customers
* Three carefully prepared road networks are provided (`data/roadnetwork`)
* Several problem instances are provided (`data/benchmark`)

Here's an example road network, `cd1`:
<img src="data/roadnetwork/fig/cd1.png" alt="Chengdu, China" style="width:480px;"/>

### Comes with extra tools
* `tool/gtreebuilder` - build a GTree spatial index (Zhong 2015) for fast
  shortest-path finding
* `tool/probplot` - plot road networks and problem sets using matplotlib
* `tool/rspgen` - generate ridesharing problem instances
* `tool/rspoptsol` - solve problem instances optimally using a mixed-integer
  mathematical program model

### Example
This example implements a greedy-insertion matching strategy. The example
overrides handle_customer() in order to match each customer as it arrives.
Check the `example` folder for more.
```cpp
void GreedyInsertion::handle_customer(const cargo::Customer cust)
{
    // Customers that are already assigned, but not yet picked up, still
    // trigger a handle_customer(). Here, we skip those that are already
    // assigned.
    if (cust.assigned())
        return;

    cargo::DistanceInt cost;
    cargo::DistanceInt best_cost = cargo::InfinityInt;
    std::vector<cargo::Stop> schedule;
    std::vector<cargo::Stop> best_schedule;
    std::vector<cargo::Waypoint> route;
    std::vector<cargo::Waypoint> best_route;
    cargo::Vehicle best_vehicle;
    bool matched = false;

    // vehicles() provides all available vehicles. Loop through them, and
    // assign this customer to the vehicle with the least-cost after inserting
    // the customer into the vehicle's schedule using the "cheap insertion"
    // heuristic (Jaw 1986) (sop_insert()).
    // TODO: use index to narrow the candidates
    for (const auto& veh : vehicles()) {
        cost = cargo::sop_insert(veh, cust, schedule, route);
        bool within_time = cargo::check_timewindow_constr(schedule, route);
        if ((cost < best_cost) && within_time) {
            best_schedule = schedule;
            best_route = route;
            best_vehicle = veh;
            best_cost = cost;
            matched = true;
        }
    }
    // Commit the match to the database so that Cargo::step() will move the
    // vehicle along its new route during the next simulation time step.
    if (matched) {
        commit(cust, best_vehicle, best_route, best_schedule);
        print_success << "Match (cust" << cust.id() << ", veh" << best_vehicle.id() << ")\n";
        nmatches++;
    }
}
```

### Similar projects

* [Open-VRP](https://github.com/mck-/Open-VRP)
* [jsprit](https://github.com/graphhopper/jsprit)
* [VRPH](https://projects.coin-or.org/VRPH)

## Building and usage

So far, Cargo has only been tested on Linux. Build using
```
make
```
This command will output the static library file into `lib/libcargo.a`.

To use the library in your own project, copy `include/*` and `libcargo.a`
into someplace where your compiler can find. For example:
```
myproject/
  include/libcargo.a
  include/*
  myalgorithm.cc
```
Then, compile using `-Iinclude`  and link the library using `-Linclude -lcargo`.
Cargo depends on gtree, which depends on [METIS](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview). Cargo also uses pthreads.

Look at the Makefiles in the examples for guidance.

Here is what myalgorithm.cc might look like:
```cpp
#include "libcargo.h"
#include "myalgorithm.h" // <-- MyAlgorithm should inherit from RSAlgorithm
int main()
{
    cargo::Options opt;
    opt.path_to_roadnet = "my.rnet";
    opt.path_to_gtree   = "my.gtree";
    opt.path_to_edges   = "my.edges";
    opt.path_to_problem = "some.instance";
    opt.time_multiplier = 1;  // 1=real-time, 2=double-time, etc.
    opt.vehicle_speed   = 10; // meters per second
    opt.matching_period = 60; // seconds

    MyAlgorithm alg;
    cargo::Cargo cargo(opt);
    cargo.start(alg); // <-- cargo takes care of starting up threads,
                      //     calling alg.listen(), etc.
}
```

## Schema
```
                                 ┌──────────────────┐
   ┌──────────────────┐          │ stops            │
   │ nodes            │          ├──────────────────┤
   ├──────────────────┤          │ *owner (int)     │
┌─>│ *id (int)        │<─────────| *location (int)  │
│  |  lng (real)      │          |  type (int)      │
│  |  lat (real)      │          │  early (int)     │
│  └──────────────────┘          │  late (int)      │
│                                │  visitedAt (int) │
│  ┌───────────────────────┐     └──────────────────┘
│  │ vehicles              │
│  ├───────────────────────┤
│  │ *id (int)             │<─┐  ┌──────────────────┐
└──│  origin_id (int)      │  │  │ schedules        │
└──│  destination_id (int) │  │  ├──────────────────┤
│  │  early (int)          │  ├──│ *owner (int)     │
│  │  late (int)           │  │  |  data (text)     │
│  │  load (int)           │  │  └──────────────────┘
│  │  status (int)         │  │
│  └───────────────────────┘  │  ┌──────────────────────────────┐
│                             │  │ routes                       │
│  ┌───────────────────────┐  │  ├──────────────────────────────┤
│  │ customers             │  ├──│ *owner (int)                 │
│  ├───────────────────────┤  │  |  data (text)                 │
│  │ *id (int)             │  │  |  idx_last_visited_node (int) │
└──│  origin_id (int)      │  │  │  next_node_distance (int)    │
└──│  destination_id (int) │  │  └──────────────────────────────┘
   │  early (int)          │  │
   │  late (int)           │  │
   │  load (int)           │  │
   │  status (int)         │  │
   │  assignedTo (int)     │──┘
   └───────────────────────┘
```

## Coming soon:
* Simulation statistics (number of matches, avg. trip delay, etc.)
* Plotting and animation
* Performance improvements
* Better problem instances, with matching models for rspoptsol
* Optimal solutions to the instances
* More examples

If you discover a bug, please [Submit an Issue](https://github.com/jamjpan/Cargo/issues/new)
Better yet, fix it and submit a pull request.

