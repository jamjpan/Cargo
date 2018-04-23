#ifndef CARGO_HAVERSINE_H_
#define CARGO_HAVERSINE_H_

#include "basic_types.h"

#include <cmath>

namespace cargo {
namespace distance {

inline Distance haversine(const Point& u, const Point& v) const {
    double r = 6372800.0; // radius of Earth (m)
    double x = (u.lng - v.lng)*(kPI/180);
    double y = (u.lat - v.lat)*(kPI/180);
    double a = std::sin(y/2)*std::sin(y/2) +
               std::sin(x/2)*std::sin(x/2)*std::cos(u.lat*(kPI/180)) *
                   std::cos(v.lat*(kPI/180));
    return r*(2*std::asin(std::sqrt(a)));
}

} // namespace distance
} // namespace cargo

#endif // CARGO_HAVERSINE_H_
