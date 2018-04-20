#include "common.h"

double haversine(node_t u, node_t v)
{
    double r = 6372800.0; // radius of Earth (m)
    double x = (u.lng - v.lng) * (PI / 180);
    double y = (u.lat - v.lat) * (PI / 180);
    double a = sin(y / 2) * sin(y / 2) +
               sin(x / 2) * sin(x / 2) * cos(u.lat * (PI / 180)) *
                   cos(v.lat * (PI / 180));
    // ROUND DOWN because the weights in the bj road network are truncated
    return std::floor(r * (2 * asin(sqrt(a))));
}