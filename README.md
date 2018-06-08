# Cargo - A Ridesharing Algorithms Library

## Introduction

Dynamic ridesharing is a type of vehicle routing problem (VRP) closely related
to the variants known as PDPTW (or VRPPDTW) and DARP (dial-a-ride).

Cargo aims to provide a set of abstractions to make it easy to develop
ridesharing algorithms. The customer/vehicle simulator and ridesharing
algorithms run on separate threads to mimic the real world. Features:
* simulate vehicles moving on a road network in real time
* broadcast customers and vehicles in real time
* provides a generic RSAlgorithm class; users override three method:
  handle_customer(), handle_vehicle(), and match()
* provides shortest-path route finding and vehicle scheduling algorithms
* find candidate vehicles using rtree
* provides time-window and precedence-constraint checking
* (todo) provides a generic logger interface to record statistics of a session
* uses an in-memory Sqlite3 db to keep track of state

### Example

```cpp
void GreedyInsertion::match() {
    for (const auto& customer : waiting_customers()) {
        if (customer.assignedTo != 0) // customer already assigned
            continue;
        for (const auto& vehicle : vehicles()) {
            cost = cargo::sop_insert(
                vehicle.schedule(), customer,
                true, true, schedule, route);
        
        }
    }
}

```

### Simulator

The purposes of the simulator are to
- broadcast requests and vehicles in real time, and
- simulate the ground-truth movement of the vehicles.

### Solution

todo

### Logger

todo

### Similar projects

- [Open-VRP](https://github.com/mck-/Open-VRP)
- [jsprit](https://github.com/graphhopper/jsprit)
- [VRPH](https://projects.coin-or.org/VRPH)

## Prerequisites

Users should have pthreads. The simulator's Run() loop blocks an
entire thread. The recommended usage is to execute the loop on a separate
thread, and execute a Solution on the main thread.

Users must have the [METIS graph partitioning
library](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview) installed.
Cargo provides a gtree (Zhong et al 2015)  spatial index for computing shortest
paths, and METIS is required to build this dependency.

## Schema

Schema for the ground-truth simulation state tables is below. The tables
are represented as `std::unordered_map`s.
```
┌──────────────────┐                             ┌───────────────────┐
│ VEHICLES         │                             │ ASSIGNMENTS       │
├──────────────────┤                             ├───────────────────┤
│ id (int)         │                             │ customer_id (int) │
│ load (int)       │                             │ vehicle_id (int)  │
│ nnd (double)     │ // next-node distance       └───────────────────┘
│ early (int)      │ // in sim time units
│ late (int)       │ // in sim time units
│ origin (int)     │
│ dest (int)       │
│ route (vec)      │ // route, vector<int>
│ sched (vec)      │ // vector<int> of stop IDs <─────┐
│ lv_node (itr)    │ // route::itr last-visited node  │
│ lv_stop (itr)    │ // sched::itr last-visited stop  │
└──────────────────┘                                  │
                                                      │
┌──────────────────┐                                  │
│ STOPS            │                                  │
├──────────────────┤                                  │
│ id (int)         │ ─────────────────────────────────┘
│ trip_id (int)    │ // corresponds to cust/veh
│ node_id (int)    │
│ type (int)       │ // 1=cust-origin;2=cust-dest;3=veh-origin;4=veh-dest
│ visit_time (int) │ // defaults to early if type=1,3; late if type=2,4
└──────────────────┘
```
### State updates:

todo

### Grid index:

todo

## Usage

Build the library using `make`. The library will be placed into `lib/libcargo.a`
after building. Then, in your own project, include the header to access the
libcargo API. The `include/` folder should be placed somewhere your compiler
can access.
```cpp
// myproj.cpp
#include "libcargo.h"
int main() {
    cargo::opts::Options myOpts;
    myOpts.RoadNetworkPath = "...";
    ... // set options

    cargo::Simulator mySim;
    mySim.SetOptions(myOpts);
    mySim.Initialize();

    mySim.Run(); // this should be on a separate thread
}
```

To compile, link the library (and don't forget to link METIS).
`g++ myproj.cpp -L/path/to/cargo -lcargo -L/path/to/metis -lmetis`

## To do (check means passed tests):

- [x] Distance functions (Euclidean and Haversine)
- [x] Message class provides four levels of colored terminal output
- [ ] Semantic types match the ridesharing problem's vocabulary and entities
- [ ] Simulator tracks the state of the simulation at every time step
- [ ] Simulator broadcasts new vehicle and customer arrivals to matching algs
- [ ] Implement some matching algs
- [ ] Logging class logs statistics to disk

Help wanted! Please feel free to throw pull requests at me.
