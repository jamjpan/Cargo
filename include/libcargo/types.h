// MIT License
//
// Copyright (c) 2018 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef CARGO_INCLUDE_LIBCARGO_TYPES_H_
#define CARGO_INCLUDE_LIBCARGO_TYPES_H_

#include <chrono>
#include <limits>
#include <unordered_map>

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
// difference is better than nothing. But we don't have static type-checking.

// ints are guaranteed at least 32-bits ~> 2 billion values.
typedef int NodeId;
typedef int OriginId;
typedef int DestinationId;

// these are part of the same "type-class", and are interchangeable.
typedef int TripId;
typedef int VehicleId;
typedef int CustomerId;

// No need for double precision because these will never be operated on. Float
// gives us 7 decimal digits. For lng/lat coordinates, the 6th digit corresponds
// to a precision of roughly 110 centimeters(!), so 7 is more than enough.
typedef float Longitude;
typedef float Latitude;

// Spatial data type.
struct Point {
    Longitude lng;
    Latitude lat;
};

struct BoundingBox {
    Point lower_left;
    Point upper_right;
};

// All of these are in meters.
typedef int DistanceInt;
typedef float DistanceFloat;
typedef double DistanceDouble;

// Used as the internal simulation time; one SimTime is roughly equivalent to
// one real second. Time windows are expressed as SimTime, with 0 being the
// start of the simulation. Travel time is also expressed as SimTime, computed
// as the real (haversine) distance divided by the real speed, in m/s, and
// rounded.
typedef int SimTime;
typedef int EarlyTime;
typedef int LateTime;

typedef float Speed; // meters per second

// These used to be in all uppercase, but according to the ISO C++ guidelines,
// only macros should be in uppercase.
// http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-naming
enum class StopType {
    CustomerOrigin,
    CustomerDest,
    VehicleOrigin,
    Vehicledest,
};

enum class CustomerStatus {
    Waiting,
    Onboard,
    Arrived,
    Canceled,
};

enum class VehicleStatus {
    Waiting,
    Enroute,
    Arrived,
};

typedef int Load; // a Load < 0 indicates a vehicle; > 0 indicates a customer

typedef size_t RouteIndex;
typedef size_t ScheduleIndex;

// Lookup nodes.
typedef std::unordered_map<NodeId, Point> KeyValueNodes;

// Lookup edges.  The key-value store is "undirected"; that is, from-to and
// to-from key combinations both exist in the store. Usage:
//     EdgeMap em_;
//     em[from_id][to_id] = weight;
typedef std::unordered_map<NodeId, std::unordered_map<NodeId, DistanceDouble>>
    KeyValueEdges;

// Request broadcast time map
typedef std::unordered_map<
    CustomerId, std::chrono::time_point<std::chrono::high_resolution_clock>>
    KeyValueBroadcastTime;

// Simulator status flags
enum class SimulatorStatus {
    Running,
    Done,
};

// Filepath
typedef std::string Filepath;

// Infinity
const int InfinityInt = std::numeric_limits<int>::max();
const double InfinityDouble = std::numeric_limits<double>::infinity();

// Math PI
const double MathPI = 3.141592653589793238462643383279502884L;

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_TYPES_H_

