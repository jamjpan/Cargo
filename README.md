(We are working on a publication comparing different ridesharing algorithms
using this system, but we could use some help and expertise! If you would like
to find out more or possibly collaborate, please reach out to me at
jamesjpan@outlook.com)

# Cargo - Ridesharing Simulation & Algorithms Library
**(under development)**

Dynamic ridesharing is a type of vehicle routing problem (VRP) closely related
to the variants known as PDPTW (or VRPPDTW) and DARP (dial-a-ride). Consider
customers and vehicles appearing over time on a road network. The objective is
to route the vehicles to service the customers while minimizing total travel
distance.

Cargo: a C++11 library that aims to provide a set of abstractions for easily
developing and testing online ridesharing algorithms.

### Quick Links
* [Road networks](data/roadnetwork)
* [Benchmark instances](data/benchmark)
* [Algorithm examples](example)
* [Optimal MIP solver](tool/rspoptsol)

### Similar projects
There are a few similar projects.

* [RinSim](https://github.com/rinde/RinSim) - very nice project; users implement
  behavior at the individual vehicles level whereas in Cargo, users implement
  central decision handlers
* [Open-VRP](https://github.com/mck-/Open-VRP) - static solver, Common Lisp
* [jsprit](https://github.com/graphhopper/jsprit) - static solver, part of GraphHopper
* [VRPH](https://projects.coin-or.org/VRPH) - static solver
* [VROOM](https://github.com/VROOM-Project/vroom) - static solver
* [MATSim](https://github.com/matsim-org/matsim) - multi-agent transportation simulator
* See this [Lyft article](https://eng.lyft.com/https-medium-com-adamgreenhall-simulating-a-ridesharing-marketplace-36007a8a31f2)
  about simulation
* Upcoming keynote by Peter Frazier, Data Science Manager at Uber, about
  simulation during the [Winter Simulation Conference '18](http://meetings2.informs.org/wordpress/wsc2018/keynotes/)

Cargo's distinguishing features:
* Real-time (vehicles move in real-time; a simulation can take minutes,
  hours, as long as you like)
* Customers and vehicles are bound to road networks
* Easy interface for implementing ridesharing algorithms
* High performance -- can simulate 17,000 vehicles per second on a i5-6200 CPU @ 2.30 Ghz
  machine

### Demo
In the demo, the simulator (bottom pane) waits until the user attaches a
listener. Here the listener is `cat` (top pane). When the listener is attached,
Cargo starts the algorithm (here, greedy) and begins simulating the vehicles in
real time. The logger outputs results at every simulation step (one second).
<p align="center">
<img src="example/greedy_insertion/greedy_demo.svg" alt="Greedy" width="800"/>
</p>

Supports:
* Dynamically arriving vehicles and customers
* Vehicles with different capacities, customers with different loads
* Multiple depots
* Time windows on origins and destinations

Does not (currently) support:
* Variable-cost roads (traffic)
* Directed road networks (<-- probably supported but needs testing)
* Vehicles "waiting" at a stop (*possible* but needs some work)
* Service-time at stops (for later)

### Built-ins:
*(Documentation is upcoming)*
* Classes: Stop, Schedule, Route, Trip, Customer, Vehicle, etc.
* Functions: constraints checking, schedule insertion, schedule routing,
  shortest-paths, Haversine, etc.
* Abstract RSAlgorithm class (you must implement these methods):
  - `RSAlgorithm::handle_customer()`: called for every customer request
  - `RSAlgorithm::handle_vehicle()`: called whenever a new vehicle appears
  - `RSAlgorithm::match()`: called at some configurable frequency
  - `RSAlgorithm::end()`: called at the end of the simulation
  - `RSAlgorithm::listen()`: fine-tune an algorithm's polling (batch vs streaming)
    (default listen() is provided)

### Simulation platform:
* Simulation state is stored in an in-memory Sqlite3 database
* `Cargo::step()` runs every second to update locations of vehicles
* `RSAlgorithm::listen()` polls every second (configurable) for new vehicles/customers
* Three cleaned road networks are provided (`data/roadnetwork`)
* Several problem instances are provided (`data/benchmark`)

Provided road networks:
(bj5 and cd1 are in GCJ coordinates; mny is in WGS)

<p align="center">
<img src="data/roadnetwork/fig/mny.png" alt="Manhattan" width="280"/>
<img src="data/roadnetwork/fig/cd1.png" alt="Chengdu, China" width="280"/>
<img src="data/roadnetwork/fig/bj5.png" alt="Beijing, China" width="280"/>
</p>

### Options
* Time multiplier (run the simulation in 1x, 2x, etc. speed)
* Matching period (time before customer requests timeout)
* Vehicle speed (meters/second)

### Extra tools
* `tool/gtreebuilder` - build a GTree spatial index (Zhong 2015) for fast
  shortest-path finding
* `tool/probplot` - plot road networks and problem sets using matplotlib
* `tool/rspgen` - generate ridesharing problem instances
* `tool/rspoptsol` - solve problem instances using a mixed-integer
  mathematical program model to find their offline optimal solutions

### Example
This example implements a greedy-insertion matching strategy. The example
overrides handle_customer() in order to match each customer as it arrives.
Check the `example` folder for more.
```cpp
void GreedyInsertion::handle_customer(const cargo::Customer cust)
{
    /* Don't consider customers that are assigned but not yet picked up */
    if (cust.assigned())
        return;

    /* Containers for storing outputs */
    cargo::DistanceInt cost;
    cargo::DistanceInt best_cost = cargo::InfinityInt;
    std::vector<cargo::Stop> schedule, best_schedule;
    std::vector<cargo::Waypoint> route, best_route;

    /* best_vehicle will point to an underlying MutableVehicle in our grid */
    std::shared_ptr<cargo::MutableVehicle> best_vehicle;
    bool matched = false;

    /* Get candidates from the local grid index
     * (the grid is refreshed during listen()) */
    cargo::DistanceInt range = cargo::pickup_range(cust, cargo::Cargo::now());
    auto candidates = grid_.within_about(range, cust.origin());

    /* Loop through candidates and check which is the greedy match */
    for (const auto& cand : candidates) {

        // Don't consider vehicles that are already queued to capacity
        if (cand->queued() == cand->capacity())
            continue;

        cost = cargo::sop_insert(cand, cust, schedule, route);
        bool within_time = cargo::check_timewindow_constr(schedule, route);
        if ((cost < best_cost) && within_time) {
            best_cost = cost;
            best_schedule = schedule;
            best_route = route;
            best_vehicle = cand; // copy the pointer
            matched = true;
        }
    }

    /* Commit match to the db. Also refresh our local grid index, so data is
     * fresh for other handle_customers that occur before the next listen(). */
    if (matched) {
        grid_.commit(best_vehicle, best_route, best_schedule); // <-- update local
        commit(cust, *best_vehicle, best_route, best_schedule); // <-- write to the db TODO make commit accept pointer as 2nd arg
        print_success << "Match (cust" << cust.id() << ", veh" << best_vehicle->id() << ")\n";
        nmatches++;
    }
}
```

## Building and usage

### Building
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

### Usage
Include the library, implement an algorithm using the base RSAlgorithm class,
then call `cargo.start(alg)`. Here is what myalgorithm.cc might look like:
```cpp
#include "libcargo.h"
#include "myalgorithm.h" // <-- MyAlgorithm should inherit from RSAlgorithm
MyAlgorithm::handle_customer() { /* Custom code here */ }
MyAlgorithm::handle_vehicle() { /*...*/ }
MyAlgorithm::match() {}
MyAlgorithm::listen() {}
MyAlgorithm::end() {}
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
│  │  queued (int)         │  │
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

## To do:
* Simulation statistics (number of matches, avg. trip delay, etc.)
* Plotting and animation
* Better problem instances, with matching models for rspoptsol
* Optimal solutions to the instances
* More examples

If you discover a bug, or have a suggestion, please
[Submit an Issue](https://github.com/jamjpan/Cargo/issues/new)!

Better yet, fix/implement it and submit a pull request.

