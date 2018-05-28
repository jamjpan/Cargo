#include "libcargo.h"

using namespace cargo;

class NearestNeighbor : public Solution {
public:
    NearestNeighbor(Simulator *, opts::Options);
    void VehicleOnline(const Trip &);
    void RequestOnline(const Trip &);
    void UpdateVehicle(const Vehicle &, const SimTime);
    void Run();

private:
    opts::Options opts_;
    // maybe wrap gtree as a util
    GTree::G_Tree gtree_;
    std::map<VehicleId, Vehicle> vehicles_;
    std::unordered_map<VehicleId, SimTime> updates_;
    // std::unordered_map<VehicleId, NodeId> locations_;
    // std::unordered_map<VehicleId, Route> routes_;
    // std::unordered_map<VehicleId, Schedule> schedules_;

    bool InsertSchedule(const Trip &, const Vehicle &, Schedule &, float &,
                        Route &);
    void Receive();
};