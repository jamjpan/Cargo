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
This example implements a greedy-insertion matching strategy. The example
overrides only the match() function, and uses the default handle_customer(),
handle_vehicle(), and listen() functions.
```cpp
void GreedyInsertion::match()
{
    for (const auto& cust : waiting_customers()) {
        if (cust.assigned())
            continue; // <-- skip assigned (but not yet picked up)

        cargo::DistanceInt cost;
        cargo::DistanceInt best_cost = cargo::InfinityInt;
        std::vector<cargo::Stop> schedule;
        std::vector<cargo::Stop> best_schedule;
        std::vector<cargo::Waypoint> route;
        std::vector<cargo::Waypoint> best_route;
        cargo::VehicleId best_vehicle = 0;
        bool matched = false;

        // TODO: use index to narrow the candidates
        for (const auto& veh : vehicles()) {
            cost = cargo::sop_insert(veh.schedule(), cust, schedule, route);
            if (cost < best_cost
                    && cargo::check_timewindow_constr(schedule, route)) {
                best_schedule = schedule;
                best_route = route;
                best_vehicle = veh.id();
                matched = true;
            }
        }
        if (matched) {
            cargo::commit(cust.id(), best_vehicle, best_route, best_schedule);
            nmatches++;
            print_success << "Match (Customer " << cust.id() << ", Vehicle "
                          << best_vehicle << ")" << std::endl;
        }
    }
    print_out << "Matches: " << nmatches << std::endl;
}
```

### Similar projects

- [Open-VRP](https://github.com/mck-/Open-VRP)
- [jsprit](https://github.com/graphhopper/jsprit)
- [VRPH](https://projects.coin-or.org/VRPH)

## Prerequisites

Users should have pthreads. Cargo runs on the main thread while a
user-implemented RSAlgorithm, passed as a parameter to Cargo::start, runs
on a separate thread.

Users must have the [METIS graph partitioning
library](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview) installed.
Cargo provides a gtree (Zhong et al 2015)  spatial index for computing shortest
paths, and METIS is required to build this dependency.

## Schema

Schema for the ground-truth simulation state tables is below.
```
                                 ┌──────────────────┐
   ┌──────────────────┐          │ stops            │
   │ nodes            │          ├──────────────────┤
   ├──────────────────┤          │ *owner (int)     │
┌─→│ *id (int)        │←─────────| *location (int)  │
│  |  lng (real)      │          |  type (int)      │
│  |  lat (real)      │          │  early (int)     │
│  └──────────────────┘          │  late (int)      │
│                                │  visitedAt (int) │
│  ┌───────────────────────┐     └──────────────────┘
│  │ vehicles              │
│  ├───────────────────────┤
│  │ *id (int)             │←─┐  ┌──────────────────┐
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
## Usage

Build the library using `make`. The library will be placed into `lib/libcargo.a`
after building. Then, in your own project, include the header to access the
libcargo API. The `include/` folder should be placed somewhere your compiler
can access.
```cpp
#include "libcargo.h"
#include "myalgorithm.h"
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
    cargo.start(alg);
}
```
To compile, link the library (and don't forget to link METIS).
`g++ myproj.cpp -L/path/to/cargo -lcargo -L/path/to/metis -lmetis`

## To do (check means passed tests):

- [x] Distance functions (Euclidean and Haversine)
- [x] Message class provides four levels of colored terminal output
- [x] Semantic types match the ridesharing problem's vocabulary and entities
- [ ] Simulator tracks the state of the simulation at every time step
- [x] Simulator broadcasts new vehicle and customer arrivals to matching algs
- [ ] Implement some matching algs
- [ ] Logging class logs statistics to disk

Help wanted! Please feel free to throw pull requests at me.

