# Cargo

Simple ridesharing library for testing matching algorithms.

## Motivation

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

The purpose of the simulator is to (1) generate requests in real time, and (2)
simulate the ground-truth movement of the vehicles.

### Solution

Cargo provides a generic Solution class for implementing online ridesharing
algorithms.

### Logger

The logger captures statistics of the simulation.

### Similar projects

- [Open-VRP](https://github.com/mck-/Open-VRP)
- [jsprit](https://github.com/graphhopper/jsprit)
- [VRPH](https://projects.coin-or.org/VRPH)

## Requirements

- pthreads
- [METIS graph partitioning library](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview)

## Usage

Build the library using `make`. The library will be placed into `lib/cargo.a`
after building.

## To do (check means passed tests):

- [ ] Road network class
-- RoadNetwork has methods knn, gtreesp, dijkstra (, haversine?)
-- Cargo will construct a RoadNetwork but solutions can implement their own RN
- [ ] Base Trip class
-- Trip members: id, origin, destination, early, late, demand (demand > 0 is customer, < 0 is vehicle)
-- Vehicle extends Trip, has some helper methods get_all_assignments, get_current_passengers, others?
-- Customer extends Trip, has some helper methods, get_current_assignment, is_picked_up, etc.
- [ ] Routes, Schedules, Stops
-- Stop is a base type, it has node_id, customer_id, type
-- Route is needed to internally simulate the moving vehicles; vector of nodes?
-- Schedule is a vector of stops
- [ ] Movement, positioning of vehicles (traffic?)
-- Vehicles are always at a node (nearest node), move at constant speed (runtime parameter?)
-- Traffic will impact the road network edge weights, for later
- [ ] Simulating request submissions
- [ ] Logging
- [ ] Useful hooks for writing output and plotting

Help wanted! Please feel free to throw pull requests at me.
