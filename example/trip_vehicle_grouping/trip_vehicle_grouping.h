#include "libcargo.h"

using namespace cargo;

class PNAS : public Solution
{
  public:
    PNAS(Simulator *, int);
    void VehicleOnline(const Trip &);
    void RequestOnline(const Trip &);
    void UpdateVehicle(const Vehicle &, const SimTime);
    void Run();

  private:
    int request_count_;
    int batch_time_;
    // maybe wrap gtree as a util
    GTree::G_Tree gtree_;
    std::map<VehicleId, Vehicle> vehicles_;
    std::unordered_map<VehicleId, SimTime> updates_;
    // std::unordered_map<VehicleId, NodeId> locations_;
    // std::unordered_map<VehicleId, Route> routes_;
    // std::unordered_map<VehicleId, Schedule> schedules_;

    bool RRPairValid(const Trip &, const Trip &);
    bool RVPairValid(const Trip &, const Vehicle &);

    bool InsertSchedule(const Trip &, const Vehicle &, Schedule &, float &, Route &);
    void Receive();
};