(We are working on a publication comparing different ridesharing algorithms
using this system, but we could use some help and expertise! If you would like
to find out more or possibly collaborate, please reach out to me at
jamesjpan@outlook.com)

# Cargo - Ridesharing Simulator Library
**(under development)**

Ridesharing is a type of vehicle routing problem (VRP).  Consider customers and
vehicles appearing over time on a road network. The objective is to route the
vehicles to service the customers while minimizing total travel distance.

Cargo is a C++11 library that provides tools for developing and evaluating
ridesharing algorithms through real-time simulation. Users can develop
algorithms by extending the `RSAlgorithm` class and evaluate their algorithms
on provided benchmarks. Each instance comprises a road network and a "problem"
file consisting of vehicles and customers on the network.

Users develop ridesharing algorithms by implementing five `RSAlgorithm`
virtual methods: `handle_vehicle`, `handle_customer`, `listen`, `match`, and
`end`.

See the docs (TODO) for more about the `RSAlgorithm` class.

In this example, an algorithm `MyAlgorithm` stores new vehicles in a local
index and matches customers as they arrive. The class extends `RSAlgorithm` and
imports a custom spatial index:
```cpp
#include "libcargo.h"
#include "myindex.h"
class MyAlgorithm : public RSAlgorithm {
 public:
  // ...
 private:
  MyIndex myidx;  // a custom spatial index
};
```

The algorithm adds each active vehicle to the index.  The method
`cargo::Vehicle::id()` returns the ID of a vehicle.
```cpp
MyAlgorithm::handle_vehicle(const cargo::Vehicle vehl) {
    my_index.add(vehl);
}
```

To keep the vehicles fresh, the algorithm clears the index each time new
vehicles and customers are polled:
```cpp
MyAlgorithm::listen() {
    my_index.clear();
    RSAlgorithm::listen();  // call the base listen()
}
```

The algorithm tries to match a customer to the nearest vehicle. Here, the index
returns a `MutableVehicle` in order to modify the vehicle's schedule with the
customer's stops. The method `cargo::Customer::orig()` returns a customer's
origin node. Types `Stop` and `Wayp` represent physical stops and waypoints.  A
sequence of waypoints represents a route through the road network. The function
`sop_insert` performs Jaw's insertion heuristic to insert the customer's stops
into the vehicle's schedule (`new_schedule` and `new_route` are the outputs).
The function `chktw` checks if the new schedule and route meet time window
constraints; finally `assign` attempts to commit the assignment to the
database. The flag `true` indicates to perform a "strict assign".
```
MyAlgorithm::handle_customer(const cargo::Customer cust) {
    MutableVehicle closest_vehl = my_index.find_nearest_to(cust.orig());
    std::vector<Stop> new_schedule;
    std::vector<Wayp> new_route;
    sop_insert(closest_vehl, cust, new_schedule, new_route);
    if (chktw(new_schedule, new_route) {
        assign({cust.id()}, {}, *closest_vehl, true);
    }
}
```

See the `examples` folder for complete examples. See the docs (TODO) for more
about the classes and functions.


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

## Log file:

Log file is orgnized by several lines indicating events during runtime, which are in the formats shown below. `[var]` is a placeholder for a variable.

```
[T] R [VID] [NODEID] [NODEID] ...
[T] V [VID] [NODEID] [VID] [NODEID] ...
[T] P [CID] [CID] ...
[T] D [CID] [CID] ...
[T] T [CID] [CID] ...
[T] A [VID] [VID] ...
[T] M [VID] [CID] [CID] ...
```

[T] indicates the simulation time when the event happens.

[VID] indicates vehicle id.

[NODEID] indicates node id in the roadnetwork.

[CID] indicates customer id.

R means route update, followed by one vehicle id and several node ids, representing a new route from current position.

V means vehicle position update, followed by pairs of vehicle id and node id.

P means pick up customers, followed by customer ids which are picked up at [T] time.

D means drop off customers, followed by customer ids which are dropped off at [T] time.

T means customers timeout, followed by customer ids whose waiting time exceeds the matching period.

A means vehicle arrive update, followed by vehicles ids which arrive at the destination.

M means match update, followed by one vehicle id and several customer ids who are matched to this vehicle.

## To do:
* Simulation statistics (number of matches, avg. trip delay, etc.)
* Plotting and animation
* Better problem instances, with matching models for rspoptsol
* Optimal solutions to the instances
* More examples

If you discover a bug, or have a suggestion, please
[Submit an Issue](https://github.com/jamjpan/Cargo/issues/new)!

Better yet, fix/implement it and submit a pull request.

