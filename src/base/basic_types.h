#ifndef CARGO_BASIC_TYPES_H_
#define CARGO_BASIC_TYPES_H_

#include <string>
#include <limits>

namespace cargo {

// We use many "logical" numerical types such as node IDs, edge IDs,
// trip IDs, etc. Unfortunately the possibility exists for these types to
// get "mingled" in the code. For example, consider, where TripId and NodeId
// are int types:
//     TripId tid;
//     NodeId nid;
//
// The following assignment is allowed by C++ even though the two vars are
// logically different:
//     nid = tid;
//     tid = nid;
//
// There are other similar issues in type conversion.
//
// Google or-tools has a nice template class in int_type.h to prevent these
// kinds of issues. We can consider using their template IntType class in the
// future. But for now, using typedefs to at least provide some semantic
// difference is better than nothing.

// ints are guaranteed at least 32-bits ~> 2 billion values
typedef int NodeId;
typedef int EdgeId;
typedef int TripId;

// Double to minimize rounding errors; unit is meters
typedef double Distance;

// No need for double precision because these will never be operated on. Float
// gives us 7 decimal digits. For lng/lat coordinates, the 6th digit corresponds
// to a precision of roughly 110 centimeters(!), so 7 is more than enough.
typedef float Longitude;
typedef float Latitude;

// Spatial coordinates
typedef struct {
    Longitude lng;
    Latitude lat;
} Point;

// The following are "small" types; their values will never be too large. We
// could use a small type such as short or char, but on modern architectures
// there is no real benefit to using these types compared to int. Using short
// or char would be semantically more pleasing, but we're redefining semantic
// types anyways, so why bother.
//
// Used as the internal simulation time; one SimTime is roughly equivalent to
// one real second. Time windows are expressed as SimTime, with 0 being the
// start of the simulation. Travel time is also expressed as SimTime, computed
// as the real (haversine) distance divided by the real speed, in m/s.
typedef int SimTime;
//
// A positive demand corresponds to the number of passengers per trip, while a
// negative demand corresponds to the available capacity of a trip. In other
// words, a negative demand indicates a "vehicle".
typedef int Demand;

// Different stop types
enum class StopType : int {
    CUSTOMER_ORIGIN,
    CUSTOMER_DESTINATION,
    VEHICLE_ORIGIN,
    VEHICLE_DESTINATION,
};

// Simulator status flags
enum class SimulatorStatus : int {
    // Default state of the simulator.
    RUNNING,

    // The simulator reaches the FINISHED state if two conditions are met:
    // (1) all trips from the problem instance have been broadcasted,
    // (2) all vehicles have arrived at their destinations.
    FINISHED,
};

// Filepath
typedef std::string Filepath;

// Infinity
const double kInfinity = std::numeric_limits<double>::infinity();

// Math PI
const double kPI = 3.141592653589793238462643383279502884L;

} // namespace cargo

#endif // CARGO_BASIC_TYPES_H_
