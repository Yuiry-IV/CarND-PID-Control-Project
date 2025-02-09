#ifndef P8_FUNC_AUX_H
#define P8_FUNC_AUX_H

#include <cmath>

#include <string>


// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
inline std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

inline double convert_max_velocity(const double &max_velocity, const double &cte){
   return max_velocity/(std::abs(cte)<1.0?1.0:std::abs(cte));
}


#endif // P8_FUNC_AUX_H
