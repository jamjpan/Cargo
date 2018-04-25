# Cargo - A Ridesharing Algorithms Library

Dynamic ridesharing is a type of vehicle routing problem (VRP) closely related
to the variants known as PDPTW (or VRPPDTW) and DARP (dial-a-ride).

The problem for academics studying ridesharing algorithms is that these
algorithms are hard to implement. Even something as simple as a "nearest
neighbor" strategy takes much time to develop because there is a lot of ground
work that needs to be built first, e.g. the road network, the requests, the
vehicles, the request handler, moving the vehicles around the road network,
etc.

A set of pre-built abstractions would make it easier to develop these
algorithms and hence to study them. The goal of the Cargo library is to provide
these abstractions. Because ridesharing in the real world is an online problem,
Cargo is targeted at online algorithms. Hence, a simulator is included that
submits simulated customer requests in real time to a solution implementation.
The architecture is below.

### Simulator

The purpose of the simulator is to
- broadcast requests and vehicles in real, and
- simulate the ground-truth movement of the vehicles.

The simulator uses an in-memory sqlite3 db to capture the ground-truth state.
The db is created per problem instance, and is destroyed when the simulation
ends. It is never saved to disk.

### Solution

Cargo provides a generic Solution class for implementing online ridesharing
algorithms.

### Logger

The logger captures statistics of the simulation and writes them to disk.

### Similar projects

- [Open-VRP](https://github.com/mck-/Open-VRP)
- [jsprit](https://github.com/graphhopper/jsprit)
- [VRPH](https://projects.coin-or.org/VRPH)

## Prerequisites

Users should have pthreads installed because the simulator's Run() loop blocks
an entire thread. The recommended usage is to execute the loop on a separate
thread, and execute a Solution on the main thread.

Users must also have the [METIS graph partitioning library](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview)
installed. Cargo provides a gtree spatial index for computing shortest paths,
and METIS is required to build this dependency.

Cargo also relies on sqlite3, included in the library.

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
