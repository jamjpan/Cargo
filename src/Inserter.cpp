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

#include "libcargo/Inserter.h"

namespace cargo
{

Inserter::Inserter(GTree::G_Tree &g) : gtree_(g) {}

bool Inserter::Inserter_jaw(Schedule &s, const Trip &t, Route &r) {
    Stop o {t.id, t.oid, StopType::CUSTOMER_ORIGIN, 0, t.early};
    Stop d {t.id, t.did, StopType::CUSTOMER_DEST, 0, t.late};

    Schedule s_min;
    int c_min = kIntInfinity;
    for (auto i = s.begin(); i != std::prev(s.end()); ++i) {
        s.insert(i, o);
        for (auto j = std::next(i); j != s.end(); ++j) {
            s.insert(j, d);
        }
    }
    return false;
}

} // namespace cargo
