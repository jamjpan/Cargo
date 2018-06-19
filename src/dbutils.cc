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
#include <iostream> /* cout */
#include <iterator> /* istream_iterator */
#include <sstream>
#include <string>
#include <vector>

#include "libcargo/dbutils.h"
#include "libcargo/classes.h"
#include "libcargo/types.h"
#include "sqlite3/sqlite3.h"

namespace cargo {

// https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string
std::vector<Stop> deserialize_schedule(const std::string& str)
{
    std::istringstream iss(str);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    std::vector<Stop> result;
    for (const auto& token : tokens) {
        std::vector<int> temp;
        size_t start = 0, end = 0;
        while ((end = token.find("|", start)) != std::string::npos) {
            temp.push_back(std::stoi(token.substr(start, end - start)));
            start = end + 1;
        }
        temp.push_back(std::stoi(token.substr(start)));
        Stop stop(temp[0], temp[1], static_cast<StopType>(temp[2]), temp[3],
                  temp[4], temp[5]);
        result.push_back(stop);
    }
    return result;
}

std::string serialize_schedule(const std::vector<Stop>& vec)
{
    std::string result = "";
    for (const auto& stop : vec)
        result.append(std::to_string(stop.owner())
                    + "|" + std::to_string(stop.location())
                    + "|" + std::to_string((int)stop.type())
                    + "|" + std::to_string(stop.early())
                    + "|" + std::to_string(stop.late())
                    + "|" + std::to_string(stop.visitedAt())
                    + " "); // whitespace delimiter
    return result;
}

namespace sql {} // namespace sql

} // namespace cargo

