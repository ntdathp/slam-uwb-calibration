#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <ros/time.h>

// cvs line to vector<string>
inline std::vector<std::string> splitCSVLine(const std::string &line, char delim = ',')
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, delim))
    out.emplace_back(item);
  return out;
}

//
inline std::string trim(const std::string &s)
{
  const char *ws = " \t\r\n";
  size_t b = s.find_first_not_of(ws);
  if (b == std::string::npos)
    return "";
  size_t e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

// Parse to ros::Time
inline ros::Time parseFlexibleStamp(const std::string &raw)
{
  std::string s = trim(raw);
  size_t dot = s.find('.');
  if (dot != std::string::npos)
  {
    std::string sec_str = s.substr(0, dot);
    std::string frac_str = s.substr(dot + 1);
    if (frac_str.size() > 9) frac_str = frac_str.substr(0, 9);
    else if (frac_str.size() < 9) frac_str.append(9 - frac_str.size(), '0');
    int64_t sec = 0, nsec = 0;
    try {
      sec = std::stoll(sec_str);
      nsec = std::stoll(frac_str);
    } catch (...) { return ros::Time(0); }
    return ros::Time((uint32_t)sec, (uint32_t)nsec);
  }

  long long v = 0;
  try { v = std::stoll(s); } catch (...) { return ros::Time(0); }

  if (v >= 1000000000000000LL) { ros::Time t; t.fromNSec((uint64_t)v); return t; }             // ns
  if (v >= 1000000000000LL)    { ros::Time t; t.fromNSec((uint64_t)v * 1000ULL); return t; }  // us
  if (v >= 1000000000LL)       { ros::Time t; t.fromNSec((uint64_t)v * 1000000ULL); return t; } // ms
  return ros::Time((uint32_t)v, 0); // s
}
