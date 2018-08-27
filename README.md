(We are working on a publication comparing different ridesharing algorithms
using this system, but we could use some help and expertise! If you would like
to find out more or possibly collaborate, please reach out to me at
jamesjpan@outlook.com)

# Cargo
**(under development)**

*A C++11 static library plus benchmarks for implementing ridesharing algorithms
and evaluating them through simulation.*

Ridesharing is a type of vehicle routing problem (VRP). The objective is to
route vehicles to service customers as they continually appear on a road
network, while minimizing travel distance.

## What it does
Cargo provides abstractions of key components for implementing and evaluating
ridesharing algorithms and includes commonly used functions. Users build their
algorithms using the abstractions, then evaluate their algorithms on the
included benchmarks. The evaluation is through simulation on real-world road
networks using historic real taxi customers data and in real time.

## How to install

Dependencies: [METIS - Serial Graph Partitioning and
Fill-reducing Matrix
Ordering](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview); other
dependencies (POSIX Threads (`pthread`), Dynamic Loader (`dl`), and
Shared Memory Objects, Clock, and Timers (`rt`)) should be included in any
base Linux installation.

### Building the library

Clone the repository (78.82 MB), then use `make`:
```
git clone https://github.com/jamjpan/Cargo.git && cd Cargo && make
```
The library will be located in `lib/libcargo.a`.

### Compiling your code

Place the contents of the `include/` directory somewhere your compiler can
access.  Using `g++`, you can compiled your code with the `-I` flag to point
the compiler to the location of these includes, if necessary. For example:
```
g++ -std=c++11 -g -c -Iinclude mycode.cc -o mycode.o
```

### Linking

Place `libcargo.a` somewhere your compiler can access. Using `g++`, link your
code using `-lcargo`. Also link the dependencies using `-pthread -lrt -lmetis
-ldl`. Use the `-L` flag to locate the libraries, if necessary. For example:
```
g++ -Llib -lcargo -pthread -lrt -L/usr/local/lib -lmetis -ldl mycode.o -o mycode
```

## Usage

### Classes

#### cargo::Stop(TripId, NodeId, StopType, ErlyTime, LateTime, SimlTime=-1)

Represents an origin or destination that a vehicle must visit.

* TripId(param1): ID for owner of the stop
* NodeId(param2): location of the stop in the road network
* StopType(param3): its type, one of `CustOrig, CustDest, VehlOrig, VehlDest`
* ErlyTime(param4): early bound on the time window of the stop
* LateTime(param5): late bound on the time window of the stop
* SimlTime(param6): time when stop is first visited

#### cargo::Schedule(VehlId, std::vector\<Stop\>)

Sequence of stops that a vehicle follows.

* VehlId(param1): ID for owner of the schedule
* std::vector\<Stop\>(param2): sequence of stops to follow, in order

#### cargo::Route(VehlId, std::vector\<Wayp\>)

Sequence of nodes that a vehicle follows.

* VehlId(param1): ID for owner of the route
* std::vector\<Wayp\>(param2): sequence of nodes (waypoints) to follow, in order

#### cargo::Trip(TripId, OrigId, DestId, ErlyTime, LateTime, Load)

Base class for representing vehicles and customers.

* TripId(param1): ID of the trip (vehicle or customer)
* OrigId(param2): location of the trip origin in the road network
* DestId(param3): location of the trip destination in the road network
* ErlyTime(param4): early bound on the time window of the trip
* LateTime(param5): late bound on the time window of the trip
* Load(param6): trip load (negative means this trip can accept load, i.e. a vehicle)

#### cargo::Customer(TripId, OrigId, DestId, ErlyTime, LateTime, Load, CustStatus, VehlId=-1)

Extends Trip; represents a customer.

* (params1-6: same as Trip)
* CustStatus(param7): one of `Waiting, Onboard, Arrived, Canceled`
* VehlId(param8): Vehicle customer is assigned to

#### cargo::Vehicle(TripId, OrigId, DestId, ErlyTime, LateTime, Load, GTree::G_Tree)

Extends Trip; represents a vehicle.

* (params1-6: same as Trip)
* GTree::G\_Tree(param7): a specific G-tree to allow parallel construction,
  used for constructing default route

#### cargo::MutableVehicle(Vehicle)

Extends Vehicle; mutable version

* Vehicle(param1): the vehicle to make a mutable copy of

#### cargo::ProblemSet()

Holds details of the problem to evaluation (properties of the trips)

#### cargo::Cargo(Options)

Simulator. Manages the ground-truth state database; in-memory road network
and problem set; G-tree and shortest-paths cache; Logger.

* Options(param1): options for configuring Cargo

#### cargo::RSAlgorithm(std::string, bool)

Base class for representing ridesharing algorithms.

* std::string(param1): algorithm name, e.g. "greedy"
* bool(param2): set to true to print messages alongside standard out in a
  separate stream

Algorithms are implemented by overriding five virtual methods:

* listen(): poll for vehicles and customers at an interval
* handle\_vehicle(Vehicle): triggered for every polled vehicle
* handle\_customer(Customer): triggered for every polled customer
* match(): triggered at the end of `listen`
* end(): triggered at the end of simulation

#### cargo::Logger(Filepath)

Writes state of the simulation to disk.

* Filepath(param1): location to write to

#### cargo::Grid(int)

Two-dimensional spatial grid index.

* int(param1): number of cells in one dimension

### Full documentation

See the docs (TODO).

## Example

Here is a complete example found in `example/simple`:

```cpp
#include <iostream>
#include <string>
#include <vector>
#include "libcargo.h"
class MyAlgorithm : public cargo::RSAlgorithm {
 public:
  MyAlgorithm();
  virtual void handle_customer(const cargo::Customer &);
  virtual void end();
 private:
  int nmatches;
};

MyAlgorithm::MyAlgorithm() : cargo::RSAlgorithm("myalg") {
  this->nmatches = 0;
}

void MyAlgorithm::handle_customer(const cargo::Customer &cust)
{
s00:  if (cust.assigned()) return;
s01:  for (const cargo::Vehicle &vehl : this->vehicles())
      {
s02:    cargo::MutableVehicle matched_vehicle(vehl);
s03:    std::vector<cargo::Stop> augmented_schedule;
s04:    std::vector<cargo::Wayp> augmented_route;
s05:    cargo::sop_insert(vehl, cust, augmented_schedule, augmented_route);
s06:    if (chktw(augmented_schedule, augmented_route) == true)
        {
s07:      std::vector<cargo::CustId> to_assign {cust.id()};
s08:      std::vector<cargo::CustId> to_remove {};
s09:      if (this->assign(to_assign, to_remove,
                augmented_route, augmented_schedule, matched_vehicle) == true)
          {
s10:        this->print(cargo::MessageType::Success) << "Matched!" << std::endl;
s11:        this->nmatches++;
s12:        break;
          }
        }
      }
}

void MyAlgorithm::end() {
  this->print(cargo::MessageType::Info)
    << "Matches: " << this->nmatches << std::endl;
}

int main()
{
    cargo::Options opt;
    opt.path_to_roadnet = "../../data/roadnetwork/bj5.rnet";
    opt.path_to_gtree   = "../../data/roadnetwork/bj5.gtree";
    opt.path_to_edges   = "../../data/roadnetwork/bj5.edges";
    opt.path_to_problem = "../../data/benchmark/rs-sm-1.instance";
    opt.time_multiplier = 1;
    opt.vehicle_speed   = 10;
    opt.matching_period = 60;

    cargo::Cargo cargo(opt);  // must be initialized before algorithm
    MyAlgorithm alg;
    cargo.start(alg);
}
```
Explanation:

* s00: Do nothing to customers that are already assigned. The method
  handle\_customer is triggered on all waiting customers, including those that
  are assigned but not yet picked up.
* s01: Loop through all active vehicles, returned by method vehicles().
* s02: Make a MutableVehicle copy so its schedule and route can be updated by
  assign() if the assignment succeeds.
* s03, s04: Initialize containers for storing new schedule and route
* s05: Insert the customer into the vehicle and output the new schedule and route
* s06: Check if the new schedule and route meet time window constraints
* s07, s08: Initialize the vectors to pass along to assign()
* s09: Try the assignment. If it succeeds, assign() returns true, and
  matched\_vehicle will be mutated with a synchronized schedule and route.
  The synchronization is with the vehicle's current ground-truth state.
* s10: Print a message.
* s11: Increment counter.
* s12: Break out of the vehicles loop; we're done.

See the `examples` folder for more examples.

## Outputs

### Solution file (algorithm_name.sol)

```
rs-md-8                       # name of the problem instance
cd1                           # name of the road network
VEHICLES 5000                 # number vehicles in the instance
CUSTOMERS 11357               # number customers in the instance
base cost 62939700            # sum cost of the customers and ridesharing vehicles (not taxis)
solution cost 196928551       # sum total cost, plus penalty for unmatched customers
matches 11345                 # number of matches
out-of-sync rejected 0        # number rejected due to out-of-sync
avg. cust. handling time 1ms  # customer handling time
avg. pickup delay 46 sec      # pickup delay (time to customer origin)
avg. trip delay 319 sec       # trip delay (extra time due to shared ride)
```
Explanation:

* base cost: the sum of the shortest-path distances from each customer's
  origin to its destination, plus the cost of the shortest-path distances for
  each ridesharing vehicle. A ridesharing vehicle is one that has its own
  destination, as opposed to shared taxis which do not.
* solution cost: the sum of the total traveled distance of all the
  vehicles, plust the base cost of each unmatched customer.
* out-of-sync rejected: the number of customers rejected due to
  synchronization failure. Synchronization failure happens when an assignment is
  attempted but the assigned schedule or route is not valid for the vehicle, for
  example the assignment removes a particular customer that the vehicle has already
  picked up.

### Data file (*.dat)

The data file is not meant for human consumption, but is human readable. It
comprises seven types of lines:

```
t [R, V, P, D, T, A, M] [data]
```

* t is the time step of the simulation
* R, V, P, D, T, A, M are each event codes
    * R indicates new route; data = [VehlId] [sequence of NodeIds]
    * V indicates new location; data = [VelId1] [NodeId] [VehlId2] [NodeId] ...
    * P indicates pickup; data = [CustId1] [CustId2] ...
    * D indicates dropoff; data = [CustId1] [CustId2] ...
    * T indicates timeout; data = [CustId1] [CustId2] ...
    * A indicates arrival; data = [VehlId1] [VehlId2] ...
    * M indicates match; data = [VehlId] [CustId1] [CustId2] ...

The data file can be fed into `tool/solplot/solplot.py` to generated an mp4
visualization.

## Tools

### gtreebuilder (tool/gtreebuilder)

Build a G-tree spatial index from an \*.edges file.

### probplot (tool/probplot)

Plot a problem instance \*.instance file.

### rspoptsol (tool/rspoptsol)

Includes GMPL model and data for optimally solving a ridesharing instance.
Only the small instances (`rs-sm-*`) are included. Model is based on
Cordeau's formulation for the Dial-a-Ride problem.

### solplot (tool/solplot)

Visualize a data file.

## Bugs and Contributing

If you discover a bug, or have a suggestion, please
[Submit an Issue](https://github.com/jamjpan/Cargo/issues/new)!

You can also submit a pull request, preferrably against your own branch or
against the dev branch.

## License

Cargo is distributed under the MIT License. Our use of SQLite does not
require a license. The lrucache is copyright 2014 lamerman and the license
can be found in `include/lrucache`. For licensing issues please contact me
jamesjpan@outlook.com.

