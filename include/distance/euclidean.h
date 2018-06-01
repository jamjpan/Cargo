#ifndef CARGO_EUCLIDEAN_H_
#define CARGO_EUCLIDEAN_H_

// #include "basic_types.h"
#include "libcargo/types.h"

#include <cmath>

namespace cargo {
namespace distance {

inline Distance euclidean(const Point &u, const Point &v) {
    return std::hypot((u.lng - v.lng), (u.lat - v.lat));
}

} // namespace distance
} // namespace cargo

#endif // CARGO_EUCLIDEAN_H_
