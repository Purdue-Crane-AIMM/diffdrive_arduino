#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <cmath>
#include <string>

class Wheel {
public:
  std::string name = "";
  int enc = 0;
  double cmd = 0;

  Wheel() = default;

  Wheel(const std::string &wheel_name) { setup(wheel_name); }

  void setup(const std::string &wheel_name) { name = wheel_name; }
};

#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
