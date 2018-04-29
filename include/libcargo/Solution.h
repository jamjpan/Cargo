// #include "common.h"
// #include "gtree/GTree.h"
#include "libcargo/types.h"
#include "libcargo/Simulator.h"
#include "queue/readerwriterqueue.h"

namespace cargo
{
class Solution
{
  public:
    Solution(Simulator *);

    virtual void RequestOnline(const Trip &){};
    virtual void VehicleOnline(const Trip &){};
    virtual void UpdateVehicle(const Vehicle &, const SimTime){};
    virtual void Run(){};
    void Terminate() { done_ = true; };

  protected:
    // GTree::G_Tree &gtree_;
    Simulator *sim_;
    moodycamel::ReaderWriterQueue<Trip> request_queue_;
    moodycamel::ReaderWriterQueue<Trip> replay_queue_;
    bool done_ = false;
    Speed speed_;
};
}