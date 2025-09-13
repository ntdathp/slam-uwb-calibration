/*
 * anchor_estimator.cpp
 *
 * Estimate 3-D anchor positions from tag-to-anchor UWB ranges
 * by batching **all** measurements into ONE global non-linear
 * least-squares problem (Ceres Solver).
 *
 * © 2025 – ntdathp
 * ---------------------------------------------------------------------------
 */

// C++ Standard Library
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

// Eigen
#include <Eigen/Dense>

// Ceres Solver
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// ROS Core
#include <ros/param.h>
#include <ros/ros.h>

// Rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS Message Types
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// UWB Driver
#include <uwb_driver/UwbRange.h>

// XmlRpc (for param parsing of structs/arrays)
#include <xmlrpcpp/XmlRpcValue.h>
#include <XmlRpcValue.h>

// Project Factors
#include "factor/AnchorDistanceFactor.hpp"
#include "factor/ExtraFactors.hpp"
#include "factor/UwbFactor.hpp"

#include "csv_utils.hpp"
#include "uwb_csv_loader.hpp"
#include "estimator_types.hpp"

// ANSI color codes for colored logging
static const std::string COLOR_RESET = "\033[0m";
static const std::string COLOR_LIGHT_BLUE = "\033[1;34m"; // Light Blue for Ceres results

// Helper to wrap text with ANSI color codes
static inline std::string colorize(const std::string &text, const std::string &color)
{
  return color + text + COLOR_RESET;
}

// ============================================================================
// Main class for anchor estimation
// ============================================================================
class ApEstimator
{
public:
  explicit ApEstimator(ros::NodeHandle &nh)
      : nh(nh), pnh("~")
  {
    // Orchestration
    loadEstimatorParams();                // thresholds, loss params, plane height, CSV outputs
    loadOdomSourceFromParams();           // odom from CSV (or bag if you enable)
    loadAntennaOffsetsFromParam();        // ~antenna_offset
    loadRangingBiasesFromParam();         // ~ranging_bias
    loadInitialAnchorsFromParam();        // ~initial_anchor_positions (and default lock state)
    applyLockedAnchorsParam();            // ~locked_anchor_ids
    loadAnchorDistanceConstraintsParam(); // ~anchor_distance_constraints OR /anchor_distance_constraints
    loadUwbMeasurementsFromParams();      // UWB source → measurements (with outlier screening)
    maybeWriteMeasurementsCsv();          // optional dump to CSV
    runOptimization();                    // ceres solve
  }

  ~ApEstimator() = default;

private:
  // =========================
  // 1) Parameter loaders
  // =========================
  void loadEstimatorParams()
  {
    // Private (~) params from node scope
    pnh.param("distance_error", distance_error, 0.7); // match your YAML
    pnh.param("write_csv", write_csv, true);
    pnh.param("cauchy_scale", cauchy_scale, 0.5); // match your YAML
    pnh.param("huber_delta", huber_delta, 0.5);   // match your YAML
    pnh.param<std::string>("output_csv_path", output_csv_path, std::string("measurements.csv"));
    pnh.param("anchor_plane_height", plane_height, 1.0); // keep local unless you want it global

    ROS_INFO("Params: distance_error=%.3f, write_csv=%s, cauchy_scale=%.3f, huber_delta=%.3f, plane_h=%.3f",
             distance_error, (write_csv ? "true" : "false"), cauchy_scale, huber_delta, plane_height);
  }

  void loadOdomSourceFromParams()
  {
    // Prefer CSV (current workflow). Bag code is kept for future use.
    std::string csv_path, odom_frame_id;
    pnh.param<std::string>("odom_csv_path", csv_path, "");
    pnh.param<std::string>("odom_frame_id", odom_frame_id, "");
    if (!csv_path.empty())
    {
      loadOdomFromCsv(csv_path, odom_frame_id);
      return;
    }

    // If needed, you can re-enable bag loading here.
    // std::string bag_path, bag_topic;
    // pnh.param<std::string>("odom_bag_path", bag_path, "");
    // pnh.param<std::string>("odom_bag_topic", bag_topic, "");
    // if (!bag_path.empty() && !bag_topic.empty())
    // {
    //   loadOdomFromBag(bag_path, bag_topic);
    //   return;
    // }

    ROS_ERROR("Please set ~odom_csv_path (preferred) or enable bag loader in code.");
  }

  void loadAntennaOffsetsFromParam()
  {
    if (!pnh.getParam("antenna_offset", antenna_offset_param))
    {
      ROS_ERROR("~antenna_offset not found.");
      return;
    }
    parseAntennaOffsets();

    ROS_INFO("Loaded antenna offsets:");
    for (const auto &kv : antenna_offsets)
    {
      for (size_t i = 0; i < kv.second.size(); ++i)
      {
        const auto &o = kv.second[i];
        ROS_INFO("  tag %d ant#%zu -> [%.3f, %.3f, %.3f]", kv.first, i, o.x(), o.y(), o.z());
      }
    }
  }

  void loadRangingBiasesFromParam()
  {
    std::vector<double> bias_vec;
    if (pnh.getParam("ranging_bias", bias_vec) && bias_vec.size() >= 2)
    {
      for (int i = 0; i + 1 < bias_vec.size(); i += 2)
        external_biases[int(bias_vec[i])] = double(bias_vec[i + 1]);
    }
    ROS_INFO("Loaded ranging biases:");
    for (const auto &kv : external_biases)
      ROS_INFO("  tag %d -> bias=%.3f m", kv.first, kv.second);
  }

  void loadInitialAnchorsFromParam()
  {
    XmlRpc::XmlRpcValue init;
    if (!pnh.getParam("initial_anchor_positions", init) ||
        init.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("~initial_anchor_positions missing or wrong type!");
      ros::shutdown();
      return;
    }

    for (auto it = init.begin(); it != init.end(); ++it)
    {
      int id = std::stoi(it->first);
      auto &arr = it->second;
      if (arr.getType() != XmlRpc::XmlRpcValue::TypeArray || arr.size() != 3)
      {
        ROS_ERROR("~initial_anchor_positions[%s] must be an array of 3", it->first.c_str());
        ros::shutdown();
        return;
      }
      double x = static_cast<double>(arr[0]);
      double y = static_cast<double>(arr[1]);
      double z = static_cast<double>(arr[2]);
      anchor_positions[id] = {x, y, z};
      lockedAnchors[id] = false;
      ROS_INFO("Anchor %d init [%.3f, %.3f, %.3f], locked=false", id, x, y, z);
    }
  }

  void applyLockedAnchorsParam()
  {
    std::vector<int> locked_anchor_ids;
    if (pnh.getParam("locked_anchor_ids", locked_anchor_ids))
    {
      for (int id : locked_anchor_ids)
      {
        if (anchor_positions.find(id) != anchor_positions.end())
        {
          lockedAnchors[id] = true;
          ROS_INFO("Anchor %d is initially locked (by ~locked_anchor_ids)", id);
        }
        else
        {
          ROS_WARN("Locked anchor id %d specified but not found in ~initial_anchor_positions", id);
        }
      }
    }
    else
    {
      ROS_INFO("No ~locked_anchor_ids provided, all anchors unlocked initially.");
    }
  }

  void loadAnchorDistanceConstraintsParam()
  {
    XmlRpc::XmlRpcValue dist_param;
    bool ok = pnh.getParam("anchor_distance_constraints", dist_param);

    // Fallback: global scope (because your launch sets them outside <node> groups)
    if (!ok)
    {
      ros::NodeHandle nhglobal("/");
      ok = nhglobal.getParam("anchor_distance_constraints", dist_param);
    }

    if (!ok)
    {
      ROS_INFO("No anchor_distance_constraints found in ~ or /");
      return;
    }

    if (dist_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("anchor_distance_constraints should be a dict");
      return;
    }

    for (auto it = dist_param.begin(); it != dist_param.end(); ++it)
    {
      const std::string key = it->first; // e.g., "100_101"
      const XmlRpc::XmlRpcValue &val = it->second;

      // Parse ids
      size_t sep = key.find('_');
      if (sep == std::string::npos)
      {
        ROS_WARN("Invalid key in anchor_distance_constraints: %s", key.c_str());
        continue;
      }
      int a = std::stoi(key.substr(0, sep));
      int b = std::stoi(key.substr(sep + 1));

      // Accept double/int or single-element array
      double d = 0.0;
      if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
          val.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        d = static_cast<double>(val);
      }
      else if (val.getType() == XmlRpc::XmlRpcValue::TypeArray && val.size() >= 1)
      {
        if (val[0].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
            val[0].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
          d = static_cast<double>(val[0]);
        }
        else
        {
          ROS_WARN("anchor_distance_constraints[%s]: array[0] not scalar", key.c_str());
          continue;
        }
      }
      else
      {
        ROS_WARN("anchor_distance_constraints[%s]: unsupported type", key.c_str());
        continue;
      }

      anchor_distance_constraints[{a, b}] = d;
    }
  }

  void loadUwbMeasurementsFromParams()
  {
    std::string uwb_bag_path, uwb_bag_topic, uwb_csv_path;
    pnh.param<std::string>("uwb_bag_path", uwb_bag_path, "");
    pnh.param<std::string>("uwb_bag_topic", uwb_bag_topic, "");
    pnh.param<std::string>("uwb_csv_path", uwb_csv_path, "");

    // Prefer CSV loader (current workflow)
    uwb_csv::loadUwbFromCsv(
        uwb_csv_path,
        [this](const ros::Time &t, nav_msgs::Odometry &out)
        { return this->getLinearOdom(t, out); },
        antenna_offsets, external_biases, anchor_positions,
        distance_error, measurements);

    // If you want to use bag input instead, uncomment:
    // if (!uwb_bag_path.empty() && !uwb_bag_topic.empty()) {
    //   loadUwbFromBag(uwb_bag_path, uwb_bag_topic);
    // }
  }

  void maybeWriteMeasurementsCsv()
  {
    if (!write_csv)
    {
      ROS_INFO("write_csv is false; skipping CSV dump.");
      return;
    }

    std::ofstream ofs(output_csv_path, std::ofstream::out);
    if (!ofs.is_open())
    {
      ROS_ERROR("Failed to open %s for writing!", output_csv_path.c_str());
      return;
    }

    ofs << "stamp,tag,antenna,anchor,distance\n";
    for (const auto &m : measurements)
    {
      ofs << m.stamp.toNSec() << ","
          << m.tag_id << ","
          << m.antenna << ","
          << m.anchor_id << ","
          << m.distance << "\n";
    }
    ofs.close();
    ROS_INFO("Wrote %zu measurements to %s", measurements.size(), output_csv_path.c_str());
  }

  // =========================
  // 2) Data sources
  // =========================
  void loadOdomFromBag(const std::string &path, const std::string &topic)
  {
    rosbag::Bag bag(path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery({topic}));

    std::lock_guard<std::mutex> lk(odom_mutex);
    odom_buffer.clear();
    for (auto &m : view)
      if (auto od = m.instantiate<nav_msgs::Odometry>())
        odom_buffer.push_back(*od);

    bag.close();
    ROS_INFO("Loaded %zu odometry msgs from %s", odom_buffer.size(), path.c_str());
  }

  void loadOdomFromCsv(const std::string &csv_path, const std::string &frame_id = std::string())
  {
    std::ifstream fin(csv_path);
    if (!fin.is_open())
    {
      ROS_ERROR_STREAM("Cannot open Odom CSV: " << csv_path);
      return;
    }

    std::string headerLine;
    if (!std::getline(fin, headerLine))
    {
      ROS_ERROR_STREAM("CSV empty: " << csv_path);
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
        ROS_ERROR_STREAM("Missing column: " << name);
        return -1;
      }
      return it->second;
    };

    // Required columns (per your sample)
    const int c_time = need("time");
    const int c_seq = need("field.header.seq");
    const int c_stamp = need("field.header.stamp");
    const int c_px = need("field.pose.pose.position.x");
    const int c_py = need("field.pose.pose.position.y");
    const int c_pz = need("field.pose.pose.position.z");
    const int c_qx = need("field.pose.pose.orientation.x");
    const int c_qy = need("field.pose.pose.orientation.y");
    const int c_qz = need("field.pose.pose.orientation.z");
    const int c_qw = need("field.pose.pose.orientation.w");
    const int c_vx = need("field.twist.twist.linear.x");
    const int c_vy = need("field.twist.twist.linear.y");
    const int c_vz = need("field.twist.twist.linear.z");
    const int c_wx = need("field.twist.twist.angular.x");
    const int c_wy = need("field.twist.twist.angular.y");
    const int c_wz = need("field.twist.twist.angular.z");

    if (c_time < 0 || c_seq < 0 || c_stamp < 0 || c_px < 0 || c_py < 0 || c_pz < 0 ||
        c_qx < 0 || c_qy < 0 || c_qz < 0 || c_qw < 0 || c_vx < 0 || c_vy < 0 || c_vz < 0 ||
        c_wx < 0 || c_wy < 0 || c_wz < 0)
    {
      ROS_ERROR("CSV header missing required columns, abort.");
      return;
    }

    std::deque<nav_msgs::Odometry> tmp;
    std::string line;
    size_t line_no = 1;

    auto atd = [&](const std::vector<std::string> &f, int idx) -> double
    {
      try
      {
        return std::stod(f[idx]);
      }
      catch (...)
      {
        return 0.0;
      }
    };
    auto ati = [&](const std::vector<std::string> &f, int idx) -> uint32_t
    {
      try
      {
        long long v = std::stoll(f[idx]);
        return (v < 0) ? 0u : (uint32_t)v;
      }
      catch (...)
      {
        return 0u;
      }
    };

    while (std::getline(fin, line))
    {
      ++line_no;
      std::string t = trim(line);
      if (t.empty() || t[0] == '#')
        continue;

      auto f = splitCSVLine(line);
      if ((int)f.size() < (int)headers.size())
      {
        ROS_WARN_STREAM("Line " << line_no << " has fewer columns than header, skip.");
        continue;
      }

      nav_msgs::Odometry od;
      od.header.seq = ati(f, c_seq);

      // Prefer field.header.stamp; fallback to time
      ros::Time st = parseFlexibleStamp(f[c_stamp]);
      if (st.isZero())
        st = parseFlexibleStamp(f[c_time]);
      od.header.stamp = st;
      if (!frame_id.empty())
        od.header.frame_id = frame_id;

      od.pose.pose.position.x = atd(f, c_px);
      od.pose.pose.position.y = atd(f, c_py);
      od.pose.pose.position.z = atd(f, c_pz);

      od.pose.pose.orientation.x = atd(f, c_qx);
      od.pose.pose.orientation.y = atd(f, c_qy);
      od.pose.pose.orientation.z = atd(f, c_qz);
      od.pose.pose.orientation.w = atd(f, c_qw);

      od.twist.twist.linear.x = atd(f, c_vx);
      od.twist.twist.linear.y = atd(f, c_vy);
      od.twist.twist.linear.z = atd(f, c_vz);
      od.twist.twist.angular.x = atd(f, c_wx);
      od.twist.twist.angular.y = atd(f, c_wy);
      od.twist.twist.angular.z = atd(f, c_wz);

      tmp.push_back(std::move(od));
    }
    fin.close();

    // Sort by time for smooth interpolation
    std::sort(tmp.begin(), tmp.end(),
              [](const nav_msgs::Odometry &a, const nav_msgs::Odometry &b)
              { return a.header.stamp < b.header.stamp; });

    // Commit to buffer
    {
      std::lock_guard<std::mutex> lk(odom_mutex);
      odom_buffer.swap(tmp);
    }
    ROS_INFO("Loaded %zu odometry rows from CSV: %s", odom_buffer.size(), csv_path.c_str());
  }

  // UWB from bag (kept for completeness; CSV path is preferred)
  void loadUwbFromBag(const std::string &path, const std::string &topic)
  {
    rosbag::Bag bag(path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery({topic}));

    std::vector<uwb_driver::UwbRange::ConstPtr> uwb_msgs;
    for (auto &m : view)
      if (auto uwb = m.instantiate<uwb_driver::UwbRange>())
        uwb_msgs.push_back(uwb);

    bag.close();
    ROS_INFO("Loaded %zu UwbRange msgs from %s", uwb_msgs.size(), path.c_str());

    for (const auto &msg : uwb_msgs)
    {
      int tag = msg->requester_id;
      int anchor = msg->responder_id;

      nav_msgs::Odometry odom;
      if (!getLinearOdom(msg->header.stamp, odom))
        continue;

      Eigen::Vector3d p(odom.pose.pose.position.x,
                        odom.pose.pose.position.y,
                        odom.pose.pose.position.z);
      Eigen::Quaterniond q(odom.pose.pose.orientation.w,
                           odom.pose.pose.orientation.x,
                           odom.pose.pose.orientation.y,
                           odom.pose.pose.orientation.z);
      int ant_idx = msg->antenna;
      Eigen::Vector3d off(0, 0, 0);
      auto itOff = antenna_offsets.find(tag);
      if (itOff != antenna_offsets.end() && ant_idx < (int)itOff->second.size())
        off = itOff->second[ant_idx];

      double dist_with_bias = msg->filtered_range + external_biases[tag];
      Measurement m{msg->header.stamp, anchor, tag, ant_idx, p + q * off, dist_with_bias};

      auto it_ap = anchor_positions.find(anchor);
      if (it_ap != anchor_positions.end())
      {
        const auto &coords = it_ap->second;
        Eigen::Vector3d anchor_pos(coords[0], coords[1], coords[2]);
        Eigen::Vector3d tag_pos = p + q * off;

        double predicted = (anchor_pos - tag_pos).norm();
        double diff = std::fabs(predicted - dist_with_bias);
        if (diff <= distance_error)
          measurements.push_back(m);
      }
      else
      {
        measurements.push_back(m);
      }
    }
  }

  // =========================
  // 3) Interpolation
  // =========================
  bool getLinearOdom(const ros::Time &t, nav_msgs::Odometry &out)
  {
    std::lock_guard<std::mutex> lk(odom_mutex);
    if (odom_buffer.size() < 2)
      return false;

    size_t i = 0;
    while (i + 1 < odom_buffer.size() && odom_buffer[i + 1].header.stamp < t)
      ++i;
    if (i + 1 >= odom_buffer.size())
      return false;

    const auto &P1 = odom_buffer[i];
    const auto &P2 = odom_buffer[i + 1];

    double dt = (P2.header.stamp - P1.header.stamp).toSec();
    if (dt <= 0.0)
      return false;

    double u = (t - P1.header.stamp).toSec() / dt;
    if (u < 0.0 || u > 1.0)
      return false;

    Eigen::Vector3d p1(P1.pose.pose.position.x, P1.pose.pose.position.y, P1.pose.pose.position.z);
    Eigen::Vector3d p2(P2.pose.pose.position.x, P2.pose.pose.position.y, P2.pose.pose.position.z);
    Eigen::Vector3d pos = p1 + u * (p2 - p1);

    Eigen::Quaterniond q1(P1.pose.pose.orientation.w, P1.pose.pose.orientation.x,
                          P1.pose.pose.orientation.y, P1.pose.pose.orientation.z);
    Eigen::Quaterniond q2(P2.pose.pose.orientation.w, P2.pose.pose.orientation.x,
                          P2.pose.pose.orientation.y, P2.pose.pose.orientation.z);
    Eigen::Quaterniond ori = q1.slerp(u, q2);

    out = P1;
    out.header.stamp = t;
    out.pose.pose.position.x = pos.x();
    out.pose.pose.position.y = pos.y();
    out.pose.pose.position.z = pos.z();
    out.pose.pose.orientation.x = ori.x();
    out.pose.pose.orientation.y = ori.y();
    out.pose.pose.orientation.z = ori.z();
    out.pose.pose.orientation.w = ori.w();
    return true;
  }

  // =========================
  // 4) Optimization
  // =========================
  void runOptimization()
  {
    std::deque<Measurement> snap = measurements;
    if (snap.empty())
    {
      ROS_WARN_THROTTLE(10, "No measurements available; skipping optimization.");
      return;
    }

    ceres::Problem problem;

    // Range factors
    for (auto &m : snap)
    {
      double *p = anchor_positions[m.anchor_id].data();
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<UwbFactor, 1, 3>(
              new UwbFactor(m.tag_pos, m.distance)),
          new ceres::CauchyLoss(cauchy_scale),
          p);
    }

    // Known distance constraints between anchors
    for (const auto &kv : anchor_distance_constraints)
    {
      int id1 = kv.first.first;
      int id2 = kv.first.second;
      double d = kv.second;

      if (anchor_positions.count(id1) && anchor_positions.count(id2))
      {
        double *p1 = anchor_positions[id1].data();
        double *p2 = anchor_positions[id2].data();

        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AnchorDistanceFactor, 1, 3, 3>(
                new AnchorDistanceFactor(d)),
            nullptr, p1, p2);
      }
    }

    // Plane factor (e.g., enforce z≥0 or close to plane—implementation inside PlaneFactor)
    for (auto &kv : anchor_positions)
    {
      double *p = kv.second.data();
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<PlaneFactor, 1, 3>(new PlaneFactor()),
          nullptr, p);
    }

    // Lock selected anchors
    for (auto &kv : anchor_positions)
      if (lockedAnchors[kv.first])
        problem.SetParameterBlockConstant(kv.second.data());

    ceres::Solver::Options opts;
    opts.max_num_iterations = 500;
    opts.function_tolerance = 1e-10;
    opts.gradient_tolerance = 1e-8;
    opts.parameter_tolerance = 1e-10;
    opts.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);
    ROS_INFO_STREAM("Ceres: " << summary.BriefReport());

    for (auto &ap : anchor_positions)
    {
      std::stringstream ss;
      ss << "Anchor " << ap.first << " estimate: ["
         << ap.second[0] << ", " << ap.second[1] << ", " << ap.second[2] << "]";
      ROS_INFO_STREAM(colorize(ss.str(), COLOR_LIGHT_BLUE));
    }
  }

  void parseAntennaOffsets()
  {
    size_t i = 0;
    while (i + 1 < antenna_offset_param.size())
    {
      int tag = static_cast<int>(antenna_offset_param[i++]);
      int cnt = static_cast<int>(antenna_offset_param[i++]);

      std::vector<Eigen::Vector3d> offs;
      offs.reserve(cnt);
      for (int k = 0; k < cnt && i + 2 < antenna_offset_param.size(); ++k)
      {
        offs.emplace_back(antenna_offset_param[i],
                          antenna_offset_param[i + 1],
                          antenna_offset_param[i + 2]);
        i += 3;
      }
      antenna_offsets[tag] = std::move(offs);
    }
  }

private:
  // === Node handles ===
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  // === Data Buffers & Mutexes ===
  std::deque<nav_msgs::Odometry> odom_buffer;
  std::mutex odom_mutex;
  std::deque<Measurement> measurements;

  // === Parameters & Configuration ===
  std::vector<double> antenna_offset_param;
  double plane_height = 1.0;
  double cauchy_scale = 0.5;
  double huber_delta = 0.5;
  double distance_error = 0.7;
  bool write_csv = true;

  // === Anchor & Antenna Data ===
  std::map<int, std::vector<Eigen::Vector3d>> antenna_offsets;
  std::map<int, std::array<double, 3>> anchor_positions;
  std::map<int, double> external_biases;
  std::map<std::pair<int, int>, double> anchor_distance_constraints;
  std::map<int, bool> lockedAnchors;

  std::string output_csv_path;
};

// =========================
// main
// =========================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "anchor_estimator_node"); // match launch node name
  ros::NodeHandle nh;                             // global
  ApEstimator est(nh);
  return 0;
}
