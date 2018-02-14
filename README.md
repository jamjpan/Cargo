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

- [ ] Road network class (or bring-your-own?)
- [ ] Calculating distances
- [ ] Base Trip class
- [ ] Requests
- [ ] Vehicles, convenience classes
- [ ] Routes, Schedules, Stops
- [ ] Movement, positioning of vehicles (traffic?)
- [ ] Simulating request submissions
- [ ] Logging
- [ ] Useful hooks for writing output and plotting

Help wanted! Please feel free to throw pull requests at me.
