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
#ifndef CARGO_INCLUDE_LIBCARGO_GUI_H_
#define CARGO_INCLUDE_LIBCARGO_GUI_H_
#include <iostream>
#include <string>

#include "classes.h"
#include "types.h"

namespace cargo {
namespace gui {

inline void center(const NodeId& u) {
  std::cout << "gui center " << u << std::endl;
}

inline void route(const vec_t<Wayp>& route, const bool& current) {
  std::string route_str = "";
  for (size_t i = 0; i < route.size(); ++i) {
    route_str += (i == route.size()-1
        ?  std::to_string(route.at(i).second)
        : (std::to_string(route.at(i).second)+" "));
  }
  std::cout << "gui route " << (current ? "cur " : "new ") << route_str << std::endl;
}

inline void curroute(const Route& route) { cargo::gui::route(route.data(), true); }
inline void curroute(const vec_t<Wayp>& route) { cargo::gui::route(route, true); }
inline void newroute(const Route& route) { cargo::gui::route(route.data(), false); }
inline void newroute(const vec_t<Wayp>& route) { cargo::gui::route(route, false); }

inline void schedule(const vec_t<Stop>& sched) {
  std::string sched_str = "";
  for (size_t i = 0; i < sched.size(); ++i) {
    sched_str += (i == sched.size()-1
        ?  std::to_string(sched.at(i).loc())
        : (std::to_string(sched.at(i).loc())+" "));
  }
  std::cout << "gui schedule " << sched_str << std::endl;
}

inline void schedule(const Schedule& sched) {
  cargo::gui::schedule(sched.data());
}

inline void clinev(const CustId& cid, const VehlId& vid) {
  std::cout << "gui clinev " << cid << " " << vid << std::endl;
}

inline void clinec(const CustId& cid1, const CustId& cid2) {
  std::cout << "gui clinec " << cid1 << " " << cid2 << std::endl;
}

inline void chi(const CustId& cid) {
  std::cout << "gui hi cust " << cid << std::endl;
}

inline void vhi(const VehlId& vid) {
  std::cout << "gui hi vehl " << vid << std::endl;
}

inline void reset() {
  std::cout << "gui reset" << std::endl;
}

}  // namespace gui
}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_GUI_H_

