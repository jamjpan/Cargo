// author @J.T. Hu

#pragma once

#include <string>

namespace cargo {

  class Cargo
  {
  public:
    std::string GTreePath;
    std::string RoadNetPath;

    Cargo();
    ~Cargo();
    void init();
    void run();
    
  };
}