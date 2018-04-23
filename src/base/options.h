#ifndef CARGO_OPTIONS_H_
#define CARGO_OPTIONS_H_

#include "basic_types.h"
#include "ridesharing_types.h"

namespace cargo {
namespace opts {

class Options {
  public:
    Options() {};

    // Path to the road network *.rnet file
    Filepath &RoadNetworkPath() {
        return path_rn_;
    }

    // Path to the gtree
    Filepath &GTreePath() {
        return path_gtree_;
    }

    // Path to the *.edge file used to build the gtree
    Filepath &EdgePath() {
        return path_edges_;
    }

    // Path to the problem instance
    Filepath &ProblemInstancePath() {
        return path_trips_;
    }

    // Name of the problem instance
    std::string &ProblemInstanceName() {
        return name_instance_;
    }

    // Name of the road network
    std::string &RoadNetworkName() {
        return name_rn_;
    }

    // Number of vehicles, determined from the problem instance
    size_t &NumberOfVehicles() {
        return count_vehicles_;
    }

    // Number of customers, determined from the problem instance
    size_t &NumberOfCustomers() {
        return count_customers_;
    }

    // Number of nodes, determined from the road network
    size_t &NumberOfNodes() {
        return count_nodes_;
    }

    // Number of edges, determined from the road network
    size_t &NumberOfEdges() {
        return count_edges_;
    }

    // The scale is a multiplier for the ratio between the SimTime and real
    // time. A scale=2, for example, will set one SimTime to be equal to
    // approximately 1/2 real seconds. The scale is unitless and has no
    // real-world semantic meaning, hence it has a generic data type, float
    float &SimTimeScale() {
        return scale_;
    }

    // Set this, in m/s
    Speed &VehicleSpeed() {
        return speed_;
    }

  private:
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
};

} // namespace opts
} // namespace cargo

#endif // CARGO_OPTIONS_H_
