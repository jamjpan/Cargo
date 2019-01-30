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
#ifndef CARGO_INCLUDE_LIBCARGO_OPTIONS_H_
#define CARGO_INCLUDE_LIBCARGO_OPTIONS_H_

#include "types.h"

namespace cargo {

struct Options {
    Filepath path_to_roadnet = "";
    Filepath path_to_problem = "";

    // A multiplier for the ratio between the SimTime and real time. A
    // multiplier of 2, for example, will set one SimTime to be equal to
    // approximately 1/2 real seconds.
    float time_multiplier = 1;

    SimlTime matching_period = 60;

    // Set to TRUE if want simulation to run until last vehicle arrives
    // (or last customer is dropped off if vehicles are taxies)
    bool full_sim = false;

    // Set to TRUE for static mode
    bool static_mode = false;

    // Set to TRUE to enable strict assignment mode
    bool strict_mode = false;

    // Save in-memory database into file when simulation finishs
    Filepath path_to_save = "";
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_OPTIONS_H_
