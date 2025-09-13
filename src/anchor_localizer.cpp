/*
 * anchor_localizer.cpp
 *
 * Anchor-based localization using UWB range measurements
 * and sliding-window optimization with odometry constraints.
 *
 * Part of the UWB-SLAM Calibration Project
 *
 * © 2025 – ntdathp
 * ---------------------------------------------------------------------------
 */

// C++ Standard Library
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <sstream>
#include <filesystem>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

// XmlRpc
#include <XmlRpcValue.h>

// ROS Core
#include <ros/ros.h>

// Rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS Message Types
#include <nav_msgs/Odometry.h>
#include <uwb_driver/UwbRange.h>

// Ceres Solver
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// Project Headers
#include "utility.h"
#include "RelOdomFactor.h"
#include "UwbPoseRangeFactor.h"
#include "factor/ExtraFactors.hpp"

#include "csv_utils.hpp"

#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_RESET "\x1b[0m"

static inline double to_ms(const std::chrono::steady_clock::time_point &t0,
                           const std::chrono::steady_clock::time_point &t1)
{
    using namespace std::chrono;
    return duration_cast<duration<double, std::milli>>(t1 - t0).count();
}

// == Convert mytf to 7-element pose array ==
inline void mytfToPose7(const mytf &tf, double pose[7])
{
    pose[0] = tf.pos.x();
    pose[1] = tf.pos.y();
    pose[2] = tf.pos.z();
    pose[3] = tf.rot.x();
    pose[4] = tf.rot.y();
    pose[5] = tf.rot.z();
    pose[6] = tf.rot.w();
}

// == Convert 7-element pose array back to mytf ==
inline mytf pose7ToMytf(const double pose[7])
{
    Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
    Eigen::Vector3d t(pose[0], pose[1], pose[2]);
    return mytf(q, t);
}

// == Wrap angle to [−π, π] ==
static double wrapPI(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

// == Frame structure for sliding window ==
struct Frame
{
    ros::Time stamp;  // timestamp of this frame
    mytf Tf_S;        // pose in secondary frame S
    double pose_U[7]; // [tx, ty, tz, qx, qy, qz, qw]
};

class AnchorLocalizer
{
public:
    explicit AnchorLocalizer(ros::NodeHandle &nh) : nh(nh)
    {
        loadParameters();
        loadOdom();
        loadUwb();
        buildFrames();
        initializePoseU();
        runSlidingWindow();
    }

private:
    // ROS
    ros::NodeHandle nh;

    // Anchor positions (world frame and secondary frame)
    std::unordered_map<int, Eigen::Vector3d> anchor_positions;

    // Antenna offsets and range biases
    std::unordered_map<int, std::vector<Eigen::Vector3d>> antenna_offsets;
    std::unordered_map<int, double> external_biases;

    // UWB messages (filtered and raw)
    std::vector<uwb_driver::UwbRange::ConstPtr> uwb_msgs;

    // Odometry buffer and sliding window frames
    std::deque<nav_msgs::Odometry> odom_buffer;
    std::vector<Frame> frames;
    int window_size = 50;

    double distance_error;
    double cauchy_scale;
    // CSV output settings
    bool csv_write_ = false;
    std::string csv_path;

    // Initial orientation (rotation matrix and yaw angles)
    Eigen::Matrix3d R0_init;
    double yaw0_init_rad;
    double yawS0;
    double z0;

    ros::Publisher pose_pub;

    void loadParameters()
    {
        // == Load initial anchor positions from ROS parameter ==
        XmlRpc::XmlRpcValue init;
        if (!nh.getParam("initial_anchor_positions", init) ||
            init.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("Param 'initial_anchor_positions' missing or wrong type");
            ros::shutdown();
            return;
        }

        for (auto it = init.begin(); it != init.end(); ++it)
        {
            int id = std::stoi(it->first);
            auto arr = it->second;
            anchor_positions[id] = Eigen::Vector3d(
                double(arr[0]), double(arr[1]), double(arr[2]));
        }

        // Assume anchor_positions is a std::unordered_map<int, Eigen::Vector3d> holding three anchors

        std::vector<Eigen::Vector3d> anchors;
        anchors.reserve(3);
        for (const auto &kv : anchor_positions)
            anchors.push_back(kv.second);

        // ---------- 1. centroid ----------
        Eigen::Vector3d c = (anchors[0] + anchors[1] + anchors[2]) / 3.0;

        // ---------- 2. plane normal ----------
        Eigen::Vector3d n = (anchors[1] - anchors[0]).cross(anchors[2] - anchors[0]);
        if (n.norm() < 1e-6)
        {
            ROS_ERROR("Anchors nearly collinear cannot define plane!");
            ros::shutdown();
            return;
        }
        n.normalize();

        // ---------- 3. rotation: n → +Z ----------
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(n, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d R = q.toRotationMatrix();

        // ---------- 4. rotate about centroid ----------
        Eigen::Vector3d c_rot = R * c;
        std::array<Eigen::Vector3d, 3> anchors_rot;
        for (size_t i = 0; i < 3; ++i)
            anchors_rot[i] = R * (anchors[i] - c) + c_rot;

        // ---------- 5. translate onto z=0 ----------
        double z0 = (anchors_rot[0].z() + anchors_rot[1].z() + anchors_rot[2].z()) / 3.0;
        Eigen::Vector3d p(0.0, 0.0, -z0);
        z0 = -z0;

        // ---------- 6. store back ----------
        size_t idx = 0;
        for (auto &kv : anchor_positions)
        {
            kv.second = anchors_rot[idx++] + p; // now z ≈ 0
        }

        ROS_INFO("Flattened anchor positions:");
        for (const auto &kv : anchor_positions)
        {
            const auto &p = kv.second;
            ROS_INFO("  ID %d: (%.4f, %.4f, %.4f)", kv.first, p.x(), p.y(), p.z());
        }

        // == Parse optional arrays: antenna offsets and ranging bias ==
        XmlRpc::XmlRpcValue ao, br;
        if (nh.getParam("antenna_offset", ao) &&
            ao.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            parseAntennaOffsets(ao);
        }
        if (nh.getParam("ranging_bias", br) &&
            br.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            parseRangingBias(br);
        }

        // == Load basic ROS parameters ==
        nh.param("window_size", window_size, 50);
        nh.param("distance_error", distance_error, 1.0);
        nh.param("cauchy_scale", cauchy_scale, 0.5);

        nh.param("csv_write_enabled", csv_write_, false);
        nh.param("csv_output_path", csv_path, std::string("odom_in_U.csv"));

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("uwb_pose", 10);
    }

    void loadOdom()
    {
        std::string csv_path, frame_id;
        nh.param("odom_csv_path", csv_path, std::string());
        nh.param("odom_frame_id", frame_id, std::string());
        if (!csv_path.empty())
        {
            loadOdomFromCsv(csv_path, frame_id);
            if (!odom_buffer.empty())
                return;
        }

        std::string path, topic;
        nh.param("odom_bag_path", path, std::string());
        nh.param("odom_bag_topic", topic, std::string());
        rosbag::Bag bag(path, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(topic));
        for (auto &m : view)
            if (auto od = m.instantiate<nav_msgs::Odometry>())
                odom_buffer.push_back(*od);
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
        { try { return std::stod(f[idx]); } catch (...) { return 0.0; } };
        auto ati = [&](const std::vector<std::string> &f, int idx) -> uint32_t
        { try { long long v = std::stoll(f[idx]); return (v < 0) ? 0u : (uint32_t)v; } catch (...) { return 0u; } };

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

        std::sort(tmp.begin(), tmp.end(), [](const nav_msgs::Odometry &a, const nav_msgs::Odometry &b)
                  { return a.header.stamp < b.header.stamp; });

        odom_buffer.swap(tmp);
        ROS_INFO("Loaded %zu odometry rows from CSV: %s", odom_buffer.size(), csv_path.c_str());
    }

    void loadUwbFromCsv(const std::string &csv_path)
    {
        std::ifstream fin(csv_path);
        if (!fin.is_open())
        {
            ROS_ERROR_STREAM("Cannot open UWB CSV: " << csv_path);
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
            return;

        uwb_msgs.clear();
        std::string line;
        size_t line_no = 1;
        while (std::getline(fin, line))
        {
            ++line_no;
            std::string t = trim(line);
            if (t.empty() || t[0] == '#')
                continue;
            auto f = splitCSVLine(line);
            if ((int)f.size() < (int)headers.size())
                continue;

            auto *msg = new uwb_driver::UwbRange();
            msg->header.stamp.fromNSec(std::stoll(f[c_stamp]));
            msg->requester_id = std::stoi(f[c_tag]);
            msg->antenna = std::stoi(f[c_ant]);
            msg->responder_id = std::stoi(f[c_anchor]);
            msg->filtered_range = std::stod(f[c_dist]); // bias is added later
            uwb_msgs.push_back(uwb_driver::UwbRange::ConstPtr(msg));
        }
        fin.close();
        ROS_INFO("Loaded %zu UWB rows from CSV: %s", uwb_msgs.size(), csv_path.c_str());
    }

    void loadUwb()
    {
        // == Load bag file path & topic ==

        std::string path, topic;
        nh.param("uwb_bag_path", path, std::string());
        nh.param("uwb_bag_topic", topic, std::string());
        std::string uwb_csv_path;
        nh.param("uwb_csv_path", uwb_csv_path, std::string());
        // rosbag::Bag bag(path, rosbag::bagmode::Read);
        // rosbag::View view(bag, rosbag::TopicQuery(topic));

        if (!uwb_csv_path.empty())
        {
            loadUwbFromCsv(uwb_csv_path);
            if (!uwb_msgs.empty())
                return;
        }

        // == Log total number of UWB messages used ==
        ROS_INFO("UWB: using %zu raw messages from bag (no filtering applied)", uwb_msgs.size());
    }

    void buildFrames()
    {
        frames.reserve(odom_buffer.size());
        for (const auto &od : odom_buffer)
        {
            Frame F;
            F.stamp = od.header.stamp;
            F.Tf_S = mytf(od);
            frames.push_back(F);
        }
    }

    void initializePoseU()
    {
        // == Collect initial measurements(up to 200) ==
        std::vector<uwb_driver::UwbRange::ConstPtr> init_meas;
        init_meas.reserve(60);
        for (size_t i = 0; i < uwb_msgs.size() && init_meas.size() < 60; ++i)
            init_meas.push_back(uwb_msgs[i]);

        // == Read and convert initial yaw parameter ==
        double yaw0_deg;
        nh.param("yaw0_init_deg", yaw0_deg, 0.0);
        double psi0 = yaw0_deg * M_PI / 180.0;

        // == Initialize estimate [x, y, yaw] ==
        double x0 = 0.0, y0 = 0.0;
        double final_x[3] = {x0, y0, psi0};

        // == Set up Ceres problem ==
        ceres::Problem problem;

        // == Add residuals for each measurement ==
        for (const auto &m : init_meas)
        {
            int tag_id = m->requester_id;
            int link_idx = m->antenna;
            Eigen::Vector3d offs = getAntennaOffset(tag_id, link_idx);

            auto it = anchor_positions.find(m->responder_id);
            if (it == anchor_positions.end())
                continue;
            const Eigen::Vector3d &pA = it->second;

            double d = m->filtered_range;
            if (!std::isfinite(d) || d <= 0.05 || d > 100.0)
                continue;
            if (auto bias_it = external_biases.find(tag_id); bias_it != external_biases.end())
                d += bias_it->second;

            problem.AddResidualBlock(
                RangeFactor::Create(offs, pA, d),
                new ceres::CauchyLoss(0.5),
                final_x);
        }

        // == Keep yaw constant during initialization ==
        std::vector<int> const_idx = {2};
        problem.SetParameterization(
            final_x,
            new ceres::SubsetParameterization(3, const_idx));

        // == Configure and solve ==
        ceres::Solver::Options solver_options;
        solver_options.num_threads = 8;
        solver_options.max_num_iterations = 50;
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;

        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);

        // == Extract and wrap results ==
        Eigen::Vector3d pU_init(final_x[0], final_x[1], z0);
        yaw0_init_rad = wrapPI(final_x[2]);
        double yaw0_final_deg = yaw0_init_rad * 180.0 / M_PI;

        // == Log optimization summary ==
        ROS_INFO("Ceres Solve Summary:\n  initial cost = %.6f\n  final cost   = %.6f",
                 summary.initial_cost, summary.final_cost);
        ROS_INFO("Estimated initial position and yaw:\n  x = %.3f  y = %.3f  yaw = %.3f deg",
                 pU_init.x(), pU_init.y(), yaw0_final_deg);

        // == Compute initial rotation matrix ==
        R0_init = Eigen::AngleAxisd(yaw0_init_rad, Eigen::Vector3d::UnitZ())
                      .toRotationMatrix();

        // == Prepare propagation variables ==
        Eigen::Vector3d last_pU = pU_init;
        Eigen::Vector3d last_pS = frames.front().Tf_S.pos;
        yawS0 = frames.front().Tf_S.yaw() * M_PI / 180.0;

        size_t init_end = 10;

        // == Set first frame pose ==
        mytf tf0(
            Eigen::Quaterniond(Eigen::AngleAxisd(yaw0_init_rad, Eigen::Vector3d::UnitZ())),
            pU_init);
        mytfToPose7(tf0, frames[0].pose_U);

        // == Propagate pose_U for remaining frames ==
        for (size_t i = 1; i < init_end; ++i)
        {
            Frame &F = frames[i];
            Eigen::Vector3d pS = F.Tf_S.pos;
            Eigen::Vector3d deltaS = pS - last_pS;
            Eigen::Vector3d pU = last_pU + R0_init * deltaS;

            double yawS = F.Tf_S.yaw() * M_PI / 180.0;
            double deltaYawS = wrapPI(yawS - yawS0);
            double psi = wrapPI(yaw0_init_rad + deltaYawS);

            Eigen::Quaterniond qU =
                Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

            mytfToPose7(mytf(qU, pU), F.pose_U);

            last_pU = pU;
            last_pS = pS;
        }
    }
    void runSlidingWindow()
    {
        // == Open CSV output if enabled ==
        std::ofstream ofs;
        if (csv_write_)
        {
            ofs.open(csv_path);
        }
        if (csv_write_)
            ofs << "time_ns,tx,ty,tz,qx,qy,qz,qw,run_time\n";

        // == Slide window over all frames ==
        for (size_t end = 10; end <= frames.size(); ++end)
        {
            size_t start;
            if (end > window_size)
                start = end - window_size;
            else
                start = 0;

            // == Predict U-frame pose for the newest frame ==
            if (end > 10)
            {
                // == Predict new U pose from S pose ==
                size_t base = 0;
                // size_t base = start;
                size_t idx_new = end - 1;
                size_t idx_prev = idx_new - 1;

                // == 1) Δ in S-frame ==
                Eigen::Vector3d pS_base = frames[base].Tf_S.pos;
                Eigen::Vector3d pS_new = frames[idx_new].Tf_S.pos;
                Eigen::Vector3d deltaS = pS_new - pS_base;

                // == 2) Previous U-frame position ==
                double *u_base = frames[base].pose_U;
                Eigen::Vector3d pU_base(u_base[0], u_base[1], u_base[2]);

                // == 3) Predict new U-frame translation ==
                Eigen::Vector3d pU_new = pU_base + R0_init * deltaS;

                // == 4) Compute new global yaw ==
                double yawS_new = frames[idx_new].Tf_S.yaw() * M_PI / 180.0;
                double yawS_base = frames[base].Tf_S.yaw() * M_PI / 180.0;
                double deltaYawS = wrapPI(yawS_new - yawS_base);
                Eigen::Quaterniond qU_base(u_base[6], u_base[3], u_base[4], u_base[5]);
                qU_base.normalize();
                double yaw_U_base = std::atan2(
                    2.0 * (qU_base.w() * qU_base.z() + qU_base.x() * qU_base.y()),
                    1.0 - 2.0 * (qU_base.y() * qU_base.y() + qU_base.z() * qU_base.z()));
                double yaw_new = wrapPI(yaw_U_base + deltaYawS);

                // == 5) Build fixed-roll/pitch quaternion ==
                Eigen::Quaterniond qU_new =
                    Eigen::AngleAxisd(yaw_new, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

                // == 6) Store predicted pose_U ==
                mytfToPose7(mytf(qU_new, pU_new), frames[idx_new].pose_U);
            }

            // == Build Ceres problem for this window ==
            ceres::Problem problem;
            auto t_build0 = std::chrono::steady_clock::now();

            auto pose7 = new ceres::ProductParameterization(
                new ceres::IdentityParameterization(3),
                new ceres::QuaternionParameterization());
            for (size_t i = start; i < end; ++i)
                problem.AddParameterBlock(frames[i].pose_U, 7, pose7);
            problem.SetParameterBlockConstant(frames[start].pose_U);

            // == RelOdom factors ==
            std::vector<ceres::internal::ResidualBlock *> res_ids_relpose;
            double cost_relpose_init = -1, cost_relpose_final = -1;
            for (size_t i = start; i + 1 < end; ++i)
            {
                mytf dT = frames[i].Tf_S.inverse() * frames[i + 1].Tf_S;
                auto id = problem.AddResidualBlock(
                    RelOdomAutoDiffFactor::Create(
                        Eigen::Vector3d::Zero(), dT.pos,
                        Eigen::Quaterniond::Identity(), dT.rot,
                        0.25, 0.25),
                    nullptr,
                    frames[i].pose_U,
                    frames[i + 1].pose_U);
                res_ids_relpose.push_back(id);
            }

            // == UWB-Range interpolation factors ==
            std::vector<ceres::internal::ResidualBlock *> res_ids_range;
            double cost_range_init = -1, cost_range_final = -1;
            for (auto &uw : uwb_msgs)
            {
                if (uw->header.stamp < frames[start].stamp ||
                    uw->header.stamp > frames[end - 1].stamp)
                    continue;

                auto it = std::lower_bound(
                    frames.begin() + start, frames.begin() + end,
                    uw->header.stamp,
                    [](auto &f, const ros::Time &t)
                    { return f.stamp < t; });
                size_t k = it - frames.begin();
                if (k == start || k >= end)
                    continue;

                double dt = (frames[k].stamp - frames[k - 1].stamp).toSec();
                if (dt <= 0)
                    continue;

                double u = (uw->header.stamp.toSec() - frames[k - 1].stamp.toSec()) / dt;
                const auto &pA = anchor_positions.at(uw->responder_id);
                Eigen::Vector3d offs = getAntennaOffset(uw->requester_id, uw->antenna);
                double d_meas = uw->filtered_range + external_biases[uw->requester_id];

                Eigen::Vector3d posUm;
                Eigen::Quaterniond quatUm;

                if (interpLinearPoseU(frames[k - 1].pose_U, frames[k].pose_U, u, posUm, quatUm))
                {
                    Eigen::Vector3d tag_U = posUm + quatUm * offs;
                    double predicted = (pA - tag_U).norm();
                    double diff = std::fabs(predicted - d_meas);
                    if (diff >= distance_error)
                        continue;
                }
                else
                {
                    continue;
                }

                auto id = problem.AddResidualBlock(
                    UwbPoseRangeInterpFactor::Create(pA, offs, d_meas, u),
                    new ceres::CauchyLoss(cauchy_scale),
                    // nullptr,
                    frames[k - 1].pose_U,
                    frames[k].pose_U);
                res_ids_range.push_back(id);
            }
            auto t_build1 = std::chrono::steady_clock::now();
            // == Solve ==
            Util::ComputeCeresCost(res_ids_relpose, cost_relpose_init, problem);
            Util::ComputeCeresCost(res_ids_range, cost_range_init, problem);
            ceres::Solver::Options opts;
            opts.num_threads = 8;
            opts.max_linear_solver_iterations = 50;
            opts.max_num_iterations = 4;
            opts.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            opts.function_tolerance = 1e-6;
            opts.gradient_tolerance = 1e-10;
            opts.parameter_tolerance = 1e-8;
            opts.gradient_check_relative_precision = 1e-6;
            opts.gradient_check_numeric_derivative_relative_step_size = 1e-6;
            opts.initial_trust_region_radius = 0.1;
            opts.max_trust_region_radius = 1.0;
            ceres::Solver::Summary summary;
            auto t_solve0 = std::chrono::steady_clock::now();
            ceres::Solve(opts, &problem, &summary);
            auto t_solve1 = std::chrono::steady_clock::now();
            Util::ComputeCeresCost(res_ids_relpose, cost_relpose_final, problem);
            Util::ComputeCeresCost(res_ids_range, cost_range_final, problem);

            // == Log costs ==
            const size_t n_pose = (end - start);
            const size_t n_rel = res_ids_relpose.size();
            const size_t n_range = res_ids_range.size();

            const double t_build_ms = to_ms(t_build0, t_build1);
            const double t_solve_ms = to_ms(t_solve0, t_solve1);
            const double t_total_ms = t_build_ms + t_solve_ms;

            printf(ANSI_COLOR_GREEN
                   "AL. J: %9.3f -> %9.3f | Jrp: %9.3f -> %9.3f | Jr: %9.3f -> %9.3f\n"
                   "AL. Time: build = %8.3f ms, solve = %8.3f ms | blocks: poses=%zu, rel=%zu, range=%zu | iters=%d\n" ANSI_COLOR_RESET,
                   summary.initial_cost, summary.final_cost,
                   cost_relpose_init, cost_relpose_final,
                   cost_range_init, cost_range_final,
                   to_ms(t_build0, t_build1),
                   to_ms(t_solve0, t_solve1),
                   n_pose, n_rel, n_range,
                   (int)summary.iterations.size());

            const auto &F = frames[end - 1];
            ros::Time stamp_proc = F.stamp + ros::Duration(t_total_ms / 1000.0);
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = stamp_proc;
            msg.header.frame_id = "uwb_frame";

            msg.pose.position.x = F.pose_U[0];
            msg.pose.position.y = F.pose_U[1];
            msg.pose.position.z = F.pose_U[2];
            msg.pose.orientation.x = F.pose_U[3];
            msg.pose.orientation.y = F.pose_U[4];
            msg.pose.orientation.z = F.pose_U[5];
            msg.pose.orientation.w = F.pose_U[6];

            pose_pub.publish(msg);

            // == Write end-of-window pose to CSV ==
            if (csv_write_)
            {
                ofs << stamp_proc.toNSec() << ','
                    << F.pose_U[0] << ',' << F.pose_U[1] << ',' << F.pose_U[2] << ','
                    << F.pose_U[3] << ',' << F.pose_U[4] << ',' << F.pose_U[5] << ',' << F.pose_U[6] << ','
                    << (t_build_ms + t_solve_ms)
                    << '\n';
            }
        }

        // == Finalize ==
        if (csv_write_)
            ofs.close();
        ROS_INFO("Sliding window optimization done (window size = %d)", window_size);
    }

    inline bool interpLinearPoseU(const double pose1_U[7],
                                  const double pose2_U[7],
                                  double u,
                                  Eigen::Vector3d &pos_out,
                                  Eigen::Quaterniond &q_out)
    {
        // --- sanity checks ------------------------------------------------------
        if (u < 0.0 || u > 1.0)
            return false;

        // --- translation: simple linear blend -----------------------------------
        Eigen::Vector3d p1(pose1_U[0], pose1_U[1], pose1_U[2]);
        Eigen::Vector3d p2(pose2_U[0], pose2_U[1], pose2_U[2]);
        pos_out = (1.0 - u) * p1 + u * p2;

        // --- orientation: SLERP --------------------------------------------------
        Eigen::Quaterniond q1(pose1_U[6], pose1_U[3], pose1_U[4], pose1_U[5]); // (w,x,y,z)
        Eigen::Quaterniond q2(pose2_U[6], pose2_U[3], pose2_U[4], pose2_U[5]);

        if (!q1.norm() || !q2.norm())
            return false; // invalid input quaternion(s)

        q1.normalize();
        q2.normalize();
        q_out = q1.slerp(u, q2);

        return true;
    }

    void parseAntennaOffsets(const XmlRpc::XmlRpcValue &ao)
    {
        int idx = 0;
        while (idx + 1 < ao.size())
        {
            int tag = int(ao[idx++]);
            int cnt = int(ao[idx++]);
            std::vector<Eigen::Vector3d> v;
            for (int k = 0; k < cnt && idx + 2 < ao.size(); ++k)
            {
                v.emplace_back(double(ao[idx]), double(ao[idx + 1]), double(ao[idx + 2]));
                idx += 3;
            }
            antenna_offsets[tag] = v;
        }

        // for (const auto &kv : antenna_offsets)
        // {
        //     int tag = kv.first;
        //     const auto &vec = kv.second;
        //     std::cout << "Offsets for tag " << tag << "):\n";
        //     for (size_t j = 0; j < vec.size(); ++j)
        //     {
        //         const auto &off = vec[j];
        //         std::cout << "  [" << j << "]: ("
        //                   << off.x() << ", "
        //                   << off.y() << ", "
        //                   << off.z() << ")\n";
        //     }
        // }
    }

    void parseRangingBias(const XmlRpc::XmlRpcValue &br)
    {
        for (int i = 0; i + 1 < br.size(); i += 2)
            external_biases[int(br[i])] = double(br[i + 1]);
        // for (const auto &kv : external_biases)
        // {
        //     int tag_id = kv.first;
        //     double bias_val = kv.second;
        //     std::cout << "external_biases[" << tag_id << "] = " << bias_val << std::endl;
        // }
    }

    Eigen::Vector3d getAntennaOffset(int tag, int idx) const
    {
        auto it = antenna_offsets.find(tag);
        if (it == antenna_offsets.end() || it->second.empty())
            return Eigen::Vector3d::Zero();
        const auto &vec = it->second;
        if (idx < 0)
            idx = 0;
        if (idx >= static_cast<int>(vec.size()))
            idx = static_cast<int>(vec.size()) - 1;
        return vec[idx];
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "anchor_localizer_sliding_window");
    ros::NodeHandle nh("~");
    AnchorLocalizer node(nh);
    return 0;
}