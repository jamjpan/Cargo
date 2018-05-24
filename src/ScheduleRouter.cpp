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
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <iterator>

#include "libcargo/ScheduleRouter.h"

namespace cargo
{

ScheduleRouter::ScheduleRouter(GTree::G_Tree &g) : gtree_(g) {}

int ScheduleRouter::RouteThrough(const Schedule &s, Route &r) {
    Route r_;
    int c = 0;
    r_.push_back(s.begin()->node_id);
    for (auto i = s.begin(); i != std::prev(s.end()); ++i) {
        Route rt;
        gtree_.find_path(i->node_id, std::next(i)->node_id, rt);
        std::copy(std::next(rt.begin()), rt.end(), std::back_inserter(r_));
        c += gtree_.search(i->node_id, std::next(i)->node_id);
    }
    r = r_;
    return c;
}

} // namespace cargo
