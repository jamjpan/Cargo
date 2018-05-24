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
#include <iostream>
#include <limits>

#include "libcargo/Inserter.h"

namespace cargo {

Inserter::Inserter(GTree::G_Tree &g) : gtree_(g), rtr_(g) {}

Schedulel Inserter::Inserter_jaw(const Schedulel &s, const Stop o, const Stop d,
                                Routel &r) {
    Schedulel s_min {};
    Routel r_min, r_;
    int c_min = kIntInfinity;
    int c_;

    for (size_t i = 0; i < s.size(); ++i) {
        Schedulel so = s;
        so.insert(std::next(so.begin(), i), o);
        for (size_t j = i+1; j <= so.size(); ++j) {
            Schedulel sod = so;
            sod.insert(std::next(sod.begin(), j), d);
            c_ = rtr_.RouteThrough(sod, r_);
            if (c_ < c_min) {
                c_min = c_;
                r_min = r_;
                s_min = sod;
            }
        }
    }

    r = r_min;
    return s_min;
}

} // namespace cargo
