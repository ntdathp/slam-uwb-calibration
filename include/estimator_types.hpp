// estimator_types.hpp
#pragma once
#include <ros/time.h>
#include <Eigen/Dense>

struct Measurement
{
  ros::Time stamp;
  int anchor_id;
  int tag_id;
  int antenna;
  Eigen::Vector3d tag_pos;
  double distance;
};
