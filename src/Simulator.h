#ifndef CARGO_SIMULATOR_H_
#define CARGO_SIMULATOR_H_

#include <unordered_map>

#include "base/basic_types.h"
#include "base/ridesharing_types.h"
#include "gtree/GTree.h"

namespace cargo {

class Simulator {
  public:
    Simulator();

    // Getters/setters to all parameters
    Filepath &ParamRoadNetworkPath();
    Filepath &ParamGTreePath();
    Filepath &ParamEdgePath();
    Filepath &ParamProblemInstancePath();
    std::string &ParamProblemInstanceName();
    std::string &ParamRoadNetworkName();
    size_t &ParamNumberOfVehicles();
    size_t &ParamNumberOfCustomers();
    size_t &ParamNumberOfNodes();
    size_t &ParamNumberOfEdges();
    Speed &ParamVehicleSpeed();

    // The scale is a multiplier for the ratio between the SimTime and real
    // time. A scale=2, for example, will set one SimTime to be equal to
    // approximately 1/2 real seconds. The scale is unitless and has no
    // real-world semantic meaning, hence it has a generic data type, float
    float &ParamSimTimeScale();

    // Attempt to load the nodes, edges, gtree, and problem instance from the
    // path parameters.
    void Initialize();

    // Start the simulation. Call Run() within its own thread, otherwise it
    // will block!
    void Run();

  private:
    // GTree index, used for fast shortest-path computation and knn search of
    // objects on the road network. The index is built from a .edges file,
    // hence the weights are all integer.
    GTree::G_Tree gtree_;

    // Lookup tables for the nodes and edges.
    NodeMap nodes_;
    EdgeMap edges_;

    // A problem instance.
    ProblemInstance pi_;

    // These mutable tables store the ground truth state of the vehicles.
    // Only the Simulator should have access to them!
    std::unordered_map<TripId, Route> routes_;
    std::unordered_map<TripId, Schedule> schedules_;
    //
    // The positions_ table stores the current position of every vehicle in the
    // simulation, using iterators to point to an element in the vehicle's
    // route. The iterator is constant, so it can be dereferenced to get the
    // node, but a new node cannot be assigned to it.
    //
    // Care must be taken here. Iterators become invalidated after several
    // operations. To be safe, after performing _any_ insertion, erasure, or
    // resizing, rediscover the iterator and update this table.
    std::unordered_map<TripId, Route::const_iterator> positions_;
    //
    // This table keeps track of how much distance each vehicle has remaining
    // until it reaches the next node in its route.
    std::unordered_map<TripId, Distance> residuals_;
    //
    // This table keeps track of the current capacities of each vehicle.
    // TODO: Should the capacity be reduced only after picking up a reqest, or
    // should it be reduced as soon as a request is assigned? If the former,
    // should the capacity table include information about the future capacity
    // of the vehicle?
    std::unordered_map<TripId, Demand> capacities_;

    // Holds the status of the simulator.
    SimulatorStatus status_;

    // Used to keep track of the time in the simulation world.
    SimTime t_;

    // The minimum length of the simulation, equal to the latest trip.early
    SimTime tmin_;

    // Holds the count of active vehicles.
    size_t count_active_;

    // This interval sets the simulation time with respect to real time. The
    // time in the problem instances is in seconds; hence set the sleep
    // interval to be equal to 1000 ms in order to approximate real time.
    // The unit is milliseconds.
    int sleep_;

    // All the parameters of the simulator
    // Maybe move these into a Params class
    Filepath path_rn_;
    Filepath path_gtree_;
    Filepath path_edges_;
    Filepath path_trips_;
    std::string name_instance_;
    std::string name_rn_;
    size_t count_vehicles_;
    size_t count_customers_;
    size_t count_nodes_;
    size_t count_edges_;
    float scale_;
    Speed speed_;

    // Update the ground-truth tables by moving the vehicles, recomputing
    // positions, residuals, capacities.
    void AdvanceSimulationState();

    // Insert a new vehicle into the ground-truth tables
    //     routes_
    //     schedules_
    //     positions_
    //     residuals_
    //     capacities_
    void InsertVehicle(const Trip &);

    // Update the ground-truth routes by copying the Route parameter into
    // the routes_ table, replacing the existing route. To be safe, followup
    // with a call to UpdatePosition to refresh the position iterator!
    void UpdateRoute(const TripId &, const Route);
    // Update other ground-truth tables.
    void UpdateSchedule(const TripId &, const Schedule);
    void UpdatePosition(const TripId &, Route::const_iterator);
    void UpdateResidual(const TripId &, const Distance);
    void UpdateCapacity(const TripId &, const Demand);
};

} // namespace cargo

#endif // CARGO_SIMULATOR_H_
