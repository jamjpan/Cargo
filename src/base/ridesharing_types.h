// author @J.T. Hu

#ifndef CARGO_RIDESHARING_TYPES_H_
#define CARGO_RIDESHARING_TYPES_H_

#include <vector>
#include <map>
#include <unordered_map>

#include "basic_types.h"

namespace cargo {

// Corresponds to nodes in the road network.
struct Node {
    NodeId id;
    Point coordinates;
};

// A lookup table keyed by node id. We use unordered_map due to the faster
// amortized/average access and insertion, plus there is no need for us to
// require sorting.
typedef std::unordered_map<NodeId, Node> NodeMap;

// An ordered sequence of nodes.
typedef std::vector<Node> Route;

// Weighted edge in the road network. EdgeA < EdgeB if EdgeA has less weight.
// Ties are broken by edge id.
struct Edge {
    EdgeId id;
    NodeId from_id;
    NodeId to_id;
    Distance weight;
};

// A lookup table for edges. The table is keyed by the from node; then by
// the to node. The final value is the weight. This way, edge weights can
// be quickly retrieved. The table is "undirected"; that is, from-to and
// to-from key combinations both exist in the table.
typedef std::unordered_map<NodeId, std::unordered_map<NodeId, Distance>> EdgeMap;

// All customers and vehicles are represented as raw "trips". The difference
// between them is largely semantic. The one logical difference is that
// vehicles have negative demand, representing their capacity in the real
// world. In the ridesharing problem, there is the demand constraint: only
// vehicles with enough capacity can accept new customer requests. Hence
// customers can never "accept" a vehicle because their demands are already
// positive.
struct Trip {
    TripId id;
    NodeId oid;
    NodeId did;

    // The time window is expressed as SimTimes. Early tells the simulator when
    // to broadcast this trip. Late tells the solver the constraint for when
    // the trip should arrive at destination.
    SimTime early;
    SimTime late;

    // Positive demand corresponds to a customer request; negative demand
    // corresponds to vehicle capacity.
    Demand demand;
};

// A set of trips is a TripGroup. In case order is important, TripGroup is
// represented as a vector.
typedef std::vector<Trip> TripGroup;

// A problem instance is the set of trips keyed by their early time. When the
// simulator time reaches SimTime, all the trips in the group are broadcasted.
// Map is used here to sort the trip groups by simtime. Then, we can easily
// find the largest trip.early to set the minimum simulation time.
struct ProblemInstance {
    std::string name;
    std::map<SimTime, TripGroup> trips;
};

// Purely semantic definitions
typedef Trip Customer;
typedef Trip Vehicle;

// Vehicle speed, in m/s
typedef float Speed;

// A stop corresponds to one single trip.
struct Stop {
    TripId trip_id;
    NodeId destination;
    StopType type;
};

// An ordered sequence of stops. There can be consecutive stops with the same
// destination, but different trip_ids.
typedef std::vector<Stop> Schedule;

} // namespace cargo

#endif // CARGO_RIDESHARING_TYPES_H_
