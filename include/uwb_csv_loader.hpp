#pragma once

#include <string>
#include <fstream>
#include <deque>
#include <map>
#include <array>
#include <unordered_map>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "estimator_types.hpp"


namespace uwb_csv
{


template <typename GetOdomFn>
void loadUwbFromCsv(
    const std::string &csv_path,
    GetOdomFn &&getLinearOdom,
    const std::map<int, std::vector<Eigen::Vector3d>> &antenna_offsets,
    const std::map<int, double> &external_biases,
    const std::map<int, std::array<double, 3>> &anchor_positions,
    double distance_error,
    std::deque<Measurement> &out_measurements)
{
  std::ifstream fin(csv_path);
  if (!fin.is_open())
  {
    ROS_ERROR_STREAM("Cannot open UWB CSV: " << csv_path);
    return;
  }

  // header
  std::string headerLine;
  if (!std::getline(fin, headerLine))
  {
    ROS_ERROR_STREAM("UWB CSV empty: " << csv_path);
    return;
  }
  auto headers = splitCSVLine(headerLine);
  std::unordered_map<std::string, int> col;
  for (int i = 0; i < (int)headers.size(); ++i)
    col[trim(headers[i])] = i;

  auto need = [&](const char *name) -> int
  {
    auto it = col.find(name);
    if (it == col.end())
    {
      ROS_ERROR_STREAM("UWB CSV missing column: " << name);
      return -1;
    }
    return it->second;
  };

  const int c_stamp = need("stamp");
  const int c_tag = need("tag");
  const int c_ant = need("antenna");
  const int c_anchor = need("anchor");
  const int c_dist = need("distance");
  if (c_stamp < 0 || c_tag < 0 || c_ant < 0 || c_anchor < 0 || c_dist < 0)
  {
    ROS_ERROR("Abort loading UWB CSV due to missing columns.");
    return;
  }

  size_t line_no = 1; 
  std::string line;
  size_t accepted = 0, rejected = 0;

  while (std::getline(fin, line))
  {
    ++line_no;
    std::string t = trim(line);
    if (t.empty() || t[0] == '#')
      continue;

    auto f = splitCSVLine(line);
    if ((int)f.size() < (int)headers.size())
    {
      ROS_WARN_STREAM("UWB CSV line " << line_no << " has fewer columns, skip.");
      continue;
    }

    // Parse 
    ros::Time stamp = parseFlexibleStamp(f[c_stamp]);
    if (stamp.isZero())
    {
      ROS_WARN_STREAM("UWB CSV line " << line_no << ": invalid stamp, skip.");
      ++rejected;
      continue;
    }

    int tag = 0, ant_idx = 0, anchor = 0;
    double dist = 0.0;
    try
    {
      tag = std::stoi(f[c_tag]);
      ant_idx = std::stoi(f[c_ant]);
      anchor = std::stoi(f[c_anchor]);
      dist = std::stod(f[c_dist]);
    }
    catch (...)
    {
      ROS_WARN_STREAM("UWB CSV line " << line_no << ": parse error, skip.");
      ++rejected;
      continue;
    }

    // inter odom
    nav_msgs::Odometry odom;
    if (!getLinearOdom(stamp, odom))
    {
    
      ++rejected;
      continue;
    }

    // p tag = p + q * offset(antenna)
    Eigen::Vector3d p(odom.pose.pose.position.x,
                      odom.pose.pose.position.y,
                      odom.pose.pose.position.z);
    Eigen::Quaterniond q(odom.pose.pose.orientation.w,
                         odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z);

    Eigen::Vector3d off(0, 0, 0);
    if (auto itOff = antenna_offsets.find(tag); itOff != antenna_offsets.end())
    {
      if (ant_idx >= 0 && ant_idx < (int)itOff->second.size())
        off = itOff->second[ant_idx];
      else
        ROS_WARN_STREAM("UWB CSV line " << line_no << ": antenna index " << ant_idx
                                        << " out of range for tag " << tag << ", using [0 0 0]");
    }
    Eigen::Vector3d tag_pos = p + q * off;

    // bias 
    double bias = 0.0;
    if (auto itB = external_biases.find(tag); itB != external_biases.end())
      bias = itB->second;

    double dist_with_bias = dist + bias;

    // outlier
    bool push_ok = true;
    if (auto it_ap = anchor_positions.find(anchor); it_ap != anchor_positions.end())
    {
      const auto &ap = it_ap->second;
      Eigen::Vector3d ap_pos(ap[0], ap[1], ap[2]);
      double predicted = (ap_pos - tag_pos).norm();
      double diff = std::fabs(predicted - dist_with_bias);
      if (diff > distance_error)
        push_ok = false;
    }

    if (push_ok)
    {
      out_measurements.push_back(Measurement{stamp, anchor, tag, ant_idx, tag_pos, dist_with_bias});
      ++accepted;
    }
    else
    {
      ++rejected;
    }
  }

  ROS_INFO_STREAM("Loaded UWB from CSV: accepted=" << accepted
                                                   << ", rejected=" << rejected
                                                   << " (" << csv_path << ")");
}

} // namespace uwb_csv
