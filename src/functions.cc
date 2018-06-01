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
#include <iterator>

#include "libcargo/functions.h"

namespace cargo {

// Complexity: O(n*|route|)
//   - for loop body executes n-1 times, for n stops
//   - std::copy is exactly |route| operations
//   - assume find_path, search are O(1) with gtree
DistanceInt route_through(GTree::G_Tree& gtree, const Schedule& s,
                          std::vector<NodeId>& r)
{
    DistanceInt cost = 0;
    r.clear();
    r.push_back(s.front().location());
    for (ScheduleIndex i = 0; i < s.size()-1; ++i) {
        std::vector<NodeId> seg;
        NodeId from = s.at(i).location();
        NodeId to = s.at(i+1).location();
        gtree.find_path(from, to, seg);
        std::copy(std::next(seg.begin()), seg.end(), std::back_inserter(r));
        cost += gtree.search(from, to);
    }
    return cost;
}

} // namespace cargo

