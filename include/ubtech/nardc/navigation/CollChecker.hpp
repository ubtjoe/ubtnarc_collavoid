/*
 * Author: Dejun Guo
 */
#pragma once
// headers
#include <ros/ros.h>
#include <cmath>
#include <limits>
#include <vector>
#include <boost/geometry.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "ubtech/nardc/navigation/collision_checker.hpp"

namespace ubtech {
namespace nardc {
namespace navigation {

class CollChecker {
 public:
   static constexpr double c_omega_tol = M_PI / 18000.0;  // tolerance for determining if moving straight or not (=0.01 deg/sec)
   /**
    * CollChecker()
    *
    * @brief construct empty CollChecker; 
    *
    * @note initialize robot's size, maximum velocity, prediction time for collavoid., initial position
    */
   CollChecker(ros::NodeHandle, ros::NodeHandle);
   Eigen::Matrix3d const get_motion_transform(double const & theta) noexcept;
   bool is_safe(void) noexcept;

 private:
   ros::NodeHandle m_nh;
   ros::Subscriber m_tf_sub, m_scan_sub, m_odom_sub;
   std::vector<ros::Subscriber> m_ir_subs;
   std::vector<ros::Subscriber> m_sonar_subs;

   double m_lookahead_time, m_dt;
   double m_linear_vel, m_angular_vel, m_linvel_min, m_angvel_min;
   double m_length, m_width;
   size_t m_num_ir, m_num_sonar;
   geometry_msgs::Vector3 m_P_lidar;
   std::vector<geometry_msgs::Vector3> m_P_ir, m_P_sonar;
   points_t m_ranging_data, m_ir_data, m_sonar_data;
   std::vector<size_t> m_cached_ir_transforms, m_cached_sonar_transforms;
   bool m_cached_lidar_transform;

  /**
   * callbacks
   *
   * @brief odom, ranging, and transform data
   */
   void range_callback(const sensor_msgs::Range::ConstPtr&);
   void scan_callback(const sensor_msgs::LaserScan::ConstPtr&);
   void tf_callback(const tf2_msgs::TFMessage::ConstPtr&);
   void odom_callback(const nav_msgs::Odometry::ConstPtr&);
   // TODO(jwd) - need to add ranging data 
};

CollChecker::CollChecker(ros::NodeHandle t_handle, ros::NodeHandle t_phandle) {
  m_nh = t_handle;
  t_phandle.param<double>("lookahead_time", m_lookahead_time, 1.0); 
  t_phandle.param<double>("lookahead_dt", m_dt, 0.2); 
  t_phandle.param<double>("veh_length", m_length, 0.84);
  t_phandle.param<double>("veh_width", m_width, 0.5);
  t_phandle.param<double>("min_linvel", m_linvel_min, -1.0);  // m/s
  t_phandle.param<double>("min_angvel", m_angvel_min, -1.0);  // 5deg/sec

  int num_ir, num_sonar;
  t_phandle.param<int>("num_ir", num_ir, 8);
  t_phandle.param<int>("num_sonar", num_sonar, 8);
  m_num_ir = static_cast<size_t>(num_ir);
  m_num_sonar = static_cast<size_t>(num_sonar);

  m_odom_sub = m_nh.subscribe<nav_msgs::Odometry>("odom", 200,
          &CollChecker::odom_callback, this);
  m_tf_sub = m_nh.subscribe<tf2_msgs::TFMessage>("tf_static", 200,
          &CollChecker::tf_callback, this);
  m_scan_sub = m_nh.subscribe<sensor_msgs::LaserScan>("scan_filtered", 200,
          &CollChecker::scan_callback, this);
  
  m_ir_subs.resize(m_num_ir);
  for (int i = 0; i < m_num_ir; ++i) {
      std::string sub_name = "ir" + std::to_string(i);
      m_ir_subs[i] = m_nh.subscribe<sensor_msgs::Range>(sub_name, 100,
                &CollChecker::range_callback, this);
  }
  
  m_sonar_subs.resize(m_num_sonar);
  for (int i = 0; i < m_num_sonar; ++i) {
      std::string sub_name = "sonar" + std::to_string(i);
      m_sonar_subs[i] = m_nh.subscribe<sensor_msgs::Range>(sub_name, 100,
                &CollChecker::range_callback, this);
  }

  m_P_ir.resize(m_num_ir);
  m_P_sonar.resize(m_num_sonar);
  for (size_t i = 0; i < m_num_ir; ++i) {
    m_P_ir[i].x = std::numeric_limits<double>::quiet_NaN();
    m_P_ir[i].y = std::numeric_limits<double>::quiet_NaN();
    m_P_ir[i].z = std::numeric_limits<double>::quiet_NaN();
  }
  for (size_t i = 0; i < m_num_sonar; ++i) {
    m_P_sonar[i].x = std::numeric_limits<double>::quiet_NaN();
    m_P_sonar[i].y = std::numeric_limits<double>::quiet_NaN();
    m_P_sonar[i].z = std::numeric_limits<double>::quiet_NaN();
  }
  m_P_lidar.x = std::numeric_limits<double>::quiet_NaN();
  m_P_lidar.y = std::numeric_limits<double>::quiet_NaN();
  m_P_lidar.z = std::numeric_limits<double>::quiet_NaN();
  
  m_cached_sonar_transforms.resize(0);
  m_cached_ir_transforms.resize(0);
  m_cached_lidar_transform = false;
  m_ranging_data.resize(0);
  m_ir_data.resize(m_num_ir);
  m_sonar_data.resize(m_num_sonar);
}

// TODO(jwd) - should only need to call this once since transforms are static
void CollChecker::tf_callback(const tf2_msgs::TFMessage::ConstPtr& tf_msg_in) {
  if (m_cached_lidar_transform
        && (m_cached_ir_transforms.size() == m_num_ir)
        && (m_cached_sonar_transforms.size() == m_num_sonar)) {
      return;  // everything is already cached
  }
  for (size_t i = 0; i < tf_msg_in->transforms.size(); ++i) {
    std::string const curr_tf = tf_msg_in->transforms[i].child_frame_id;
    ROS_WARN("%s", curr_tf.c_str());
    if (curr_tf.compare("hokuyo_link") && !m_cached_lidar_transform) {
      m_P_lidar.x = tf_msg_in->transforms[i].transform.translation.x;
      m_P_lidar.y = tf_msg_in->transforms[i].transform.translation.y;
      geometry_msgs::Quaternion ori;
      ori = tf_msg_in->transforms[i].transform.rotation;
      tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
      tf::Matrix3x3 m(q);
      geometry_msgs::Vector3 ang;
      m.getRPY(ang.x, ang.y, m_P_lidar.z);
      m_cached_lidar_transform = true;
      break;
    } else if (curr_tf.rfind("ir") == 0) {
      size_t const sid = static_cast<std::string>("ir").size();
      size_t const eid = curr_tf.rfind("_link") - 1;
      size_t const id = static_cast<size_t>(std::stoi(curr_tf.substr(sid, eid-sid+1)));
      if (!std::isnan(m_P_ir[id].x)) {
          break;
      }
      m_P_ir[id].x = tf_msg_in->transforms[i].transform.translation.x;
      m_P_ir[id].y = tf_msg_in->transforms[i].transform.translation.y;
      geometry_msgs::Quaternion ori;
      ori = tf_msg_in->transforms[i].transform.rotation;
      tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
      tf::Matrix3x3 m(q);
      geometry_msgs::Vector3 ang;
      m.getRPY(ang.x, ang.y, m_P_ir[id].z);
      m_cached_ir_transforms.emplace_back(id);
      break;
    } else if (curr_tf.rfind("sonar") == 0) {
      size_t const sid = static_cast<std::string>("sonar").size();
      size_t const eid = curr_tf.rfind("_link") - 1;
      size_t const id = static_cast<size_t>(std::stoi(curr_tf.substr(sid, eid-sid+1)));
      if (!std::isnan(m_P_sonar[id].x)) {
          break;
      }
      m_P_sonar[id].x = tf_msg_in->transforms[i].transform.translation.x;
      m_P_sonar[id].y = tf_msg_in->transforms[i].transform.translation.y;
      geometry_msgs::Quaternion ori;
      ori = tf_msg_in->transforms[i].transform.rotation;
      tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
      tf::Matrix3x3 m(q);
      geometry_msgs::Vector3 ang;
      m.getRPY(ang.x, ang.y, m_P_sonar[id].z);
      m_cached_sonar_transforms.emplace_back(id);
      break;
    }
  }
}

void CollChecker::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
  geometry_msgs::Twist const twist = odom_msg->twist.twist;
  double const vx = twist.linear.x;
  double const vy = twist.linear.y;
  m_linear_vel = sqrt(vx*vx + vy*vy);
  m_angular_vel = twist.angular.z;
}

void CollChecker::scan_callback(const sensor_msgs::LaserScan::ConstPtr& t_rdata_msg){
  auto const & angle_min = t_rdata_msg->angle_min;
  auto const & angle_max = t_rdata_msg->angle_max;
  auto const & angle_increment = t_rdata_msg->angle_increment;
  if (std::isnan(m_P_lidar.x)) {
    return;
  }
  Eigen::Matrix3d H_b2l;
  // START HERE TOMORROW - verify correct tranformation convention
  H_b2l << std::cos(m_P_lidar.z), std::sin(m_P_lidar.z), m_P_lidar.x,
      -std::sin(m_P_lidar.z), std::cos(m_P_lidar.z), m_P_lidar.y,
      0, 0, 1;
  Eigen::Matrix3d const H_l2b = H_b2l.inverse();
  // if the data structure is uninitialized, do this
  if (m_ranging_data.size() == 0) {
    for (size_t i = 0; i < t_rdata_msg->ranges.size(); ++i){	
      if (!std::isnan(t_rdata_msg->ranges[i])) {
        auto const ang = angle_min + angle_increment * static_cast<double>(i);
        Eigen::Vector3d xl, xb;
        xl << t_rdata_msg->ranges[i]*std::cos(ang),
            t_rdata_msg->ranges[i]*std::sin(ang),
            1;
        xb = H_l2b*xl;
        m_ranging_data.emplace_back(point_t(xb[0], xb[1]));
      }
    }
  } else {
    size_t ctr = 0;
    for (size_t i = 0; i < t_rdata_msg->ranges.size(); ++i){	
      if (!std::isnan(t_rdata_msg->ranges[i])) {
        auto const ang = angle_min + angle_increment * static_cast<double>(i);
        Eigen::Vector3d xl, xb;
        xl << t_rdata_msg->ranges[i]*std::cos(ang),
            t_rdata_msg->ranges[i]*std::sin(ang),
            1;
        xb = H_l2b*xl;
        m_ranging_data[ctr++] = point_t(xb[0], xb[1]);
      }
    }
  }
}

void CollChecker::range_callback(const sensor_msgs::Range::ConstPtr& t_range_msg) {
  auto const frame_id = static_cast<std::string>(t_range_msg->header.frame_id);
  if (frame_id.rfind("ir") == 0) {
      ROS_WARN("ir callback");
    size_t const sid = static_cast<std::string>("ir").size();
    size_t const eid = frame_id.rfind("_link") - 1;
    size_t const id = static_cast<size_t>(std::stoi(frame_id.substr(sid, eid-sid+1)));
      ROS_WARN("%d", id);
    Eigen::Matrix3d H_b2l;
    H_b2l << std::cos(m_P_ir[id].z), std::sin(m_P_ir[id].z), m_P_ir[id].x,
        -std::sin(m_P_ir[id].z), std::cos(m_P_ir[id].z), m_P_ir[id].y,
        0, 0, 1;
    Eigen::Matrix3d const H_l2b = H_b2l.inverse();
    Eigen::Vector3d xl, xb;
    xl << t_range_msg->range,
        0,
        1;
    xb = H_l2b*xl;
    ROS_WARN("%f, %f", xb[0], xb[1]);
    m_ir_data[id] = point_t(xb[0], xb[1]);
  } else if (frame_id.rfind("sonar") == 0) {
    size_t const sid = static_cast<std::string>("sonar").size();
    size_t const eid = frame_id.rfind("_link") - 1;
    size_t const id = static_cast<size_t>(std::stoi(frame_id.substr(sid, eid-sid+1)));
    Eigen::Matrix3d H_b2l;
    H_b2l << std::cos(m_P_sonar[id].z), std::sin(m_P_sonar[id].z), m_P_sonar[id].x,
        -std::sin(m_P_sonar[id].z), std::cos(m_P_sonar[id].z), m_P_sonar[id].y,
        0, 0, 1;
    Eigen::Matrix3d const H_l2b = H_b2l.inverse();
    Eigen::Vector3d xl, xb;
    xl << t_range_msg->range,
        0,
        1;
    xb = H_l2b*xl;
    m_sonar_data[id] = point_t(xb[0], xb[1]);
  }
}

Eigen::Matrix3d const CollChecker::get_motion_transform(double const & theta) noexcept {
  double dx, dy;
  if (m_angular_vel < c_omega_tol) {
    dx = m_linear_vel * std::cos(theta) * m_dt;
    dy = m_linear_vel * std::sin(theta) * m_dt;
  } else {
    auto const r = m_linear_vel / m_angular_vel;
    auto const theta_new = theta + m_angular_vel * m_dt;
    dx = -r * std::sin(theta) + r * std::sin(theta_new);;
    dy = r * std::cos(theta) - r * std::cos(theta_new);;
  }

  Eigen::Matrix3d T;
  T << 1, 0, dx,
       0, 1, dy,
       0, 0, 1;
  return T;
}

bool CollChecker::is_safe() noexcept {
  //! don't check if speed is too low
  if (std::abs(m_linear_vel) < m_linvel_min || std::abs(m_angular_vel) < m_angvel_min) {
    return true;
  }
  double time = 0;
  double theta = 0;

  polygon_t poly;
  bg::append(poly.outer(), point_t(0.5*m_length, 0.5*m_width));  // top-left
  bg::append(poly.outer(), point_t(0.5*m_length, -0.5*m_width));  // top-right
  bg::append(poly.outer(), point_t(-0.5*m_length, -0.5*m_width));  // bottom-right
  bg::append(poly.outer(), point_t(-0.5*m_length, 0.5*m_width));  // bottom-left
  bg::append(poly.outer(), point_t(0.5*m_length, 0.5*m_width));  // closure

  CollisionChecker cc(/* check radius = */ 5);
  cc.set_points(m_ranging_data);
  ROS_WARN("LS: %d", m_ranging_data.size());
  cc.add_points(m_ir_data);
  ROS_WARN("IR: %d", m_ir_data.size());
  cc.add_points(m_sonar_data);
  ROS_WARN("SNR: %d", m_sonar_data.size());

  Eigen::Matrix3d T(Eigen::Matrix3d::Identity());
  
  while (time <= m_lookahead_time) {
    //! check for collision
    if (cc.check_intersection(poly, T)) {
      return false;
    }
    //! update transformation
    auto const Tnew = get_motion_transform(theta);
    T = Tnew*T;
    theta += m_angular_vel * m_dt;
    time += m_dt;
  }
  //! no collision detected
  return true;
}
}  // end namespace navigation
}  // end namespace nardc
}  // end namespace ubtech
