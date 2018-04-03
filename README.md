Cargo
=======
Simple ridesharing server for testing matching algorithms.

### Reason:

Ridesharing algorithms are hard to implement. Even something as simple as a
"nearest neighbor" strategy still takes time to develop because there is a
lot of ground work that needs to be built first, e.g. the road network, the
requests, the vehicles, the request handler, moving the vehicles around the
road network, etc. A nice set of pre-built abstractions would make it easier
to develop these algorithms and hence to study them.

### Vision:

The library is general enough for people to build their own vehicle searching
and matching algorithms but powerful enough to provide a lot of out-of-the-box
functionality, like moving the vehicles around the road network, supporting
fast shortest-path searches, and supporting visualization.

### To do (check means passed tests):

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
