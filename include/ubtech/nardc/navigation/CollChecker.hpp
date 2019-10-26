/*
 * Author: Dejun Guo
 */
#pragma once
// headers
#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <boost/geometry.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
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
   ros::Subscriber m_tf_sub, m_rdata_sub, m_odom_sub;
   double m_lookahead_time, m_dt;
   double m_linear_vel, m_angular_vel;
   double m_length, m_width;
   geometry_msgs::Vector3 m_P_laser;
   points_t m_ranging_data;

  /**
   * callbacks
   *
   * @brief odom, ranging, and transform data
   */
   void tf_callback(const tf2_msgs::TFMessage::ConstPtr&);
   void rdata_callback(const sensor_msgs::LaserScan::ConstPtr&);
   void odom_callback(const nav_msgs::Odometry::ConstPtr&);
   // TODO(jwd) - need to add ranging data 
};

CollChecker::CollChecker(ros::NodeHandle t_handle, ros::NodeHandle t_phandle) {
  m_nh = t_handle;
  t_phandle.param<double>("lookahead_time", m_lookahead_time, 1.0); 
  t_phandle.param<double>("lookahead_dt", m_dt, 0.2); 
  t_phandle.param<double>("veh_length", m_length, 0.84);
  t_phandle.param<double>("veh_width", m_width, 0.5);
  m_odom_sub = m_nh.subscribe<nav_msgs::Odometry>("base/odom", 200,
          &CollChecker::odom_callback, this);
  m_tf_sub = m_nh.subscribe<tf2_msgs::TFMessage>("tf_static", 200,
          &CollChecker::tf_callback, this);
  m_rdata_sub = m_nh.subscribe<sensor_msgs::LaserScan>("scan_filtered", 200,
          &CollChecker::rdata_callback, this);
}

// TODO(jwd) - should only need to call this once since transforms are static
void CollChecker::tf_callback(const tf2_msgs::TFMessage::ConstPtr& tf_msg_in) {
  for (size_t i = 0; i < tf_msg_in->transforms.size(); ++i) {
    if (tf_msg_in->transforms[i].child_frame_id=="hokuyo_link") {
      m_P_laser.x = tf_msg_in->transforms[i].transform.translation.x;
      m_P_laser.y = tf_msg_in->transforms[i].transform.translation.y;
      geometry_msgs::Quaternion ori;
      ori = tf_msg_in->transforms[i].transform.rotation;
      tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
      tf::Matrix3x3 m(q);
      geometry_msgs::Vector3 ang;
      m.getRPY(ang.x, ang.y, m_P_laser.z);
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

void CollChecker::rdata_callback(const sensor_msgs::LaserScan::ConstPtr& t_rdata_msg){
  m_ranging_data.resize(0);
  auto const & angle_min = t_rdata_msg->angle_min;
  auto const & angle_max = t_rdata_msg->angle_max;
  auto const & angle_increment = t_rdata_msg->angle_increment;
  for (size_t i = 0; i < t_rdata_msg->ranges.size(); ++i){	
    if (!std::isnan(t_rdata_msg->ranges[i])) {
      auto const ang = angle_min + angle_increment * static_cast<double>(i);
      m_ranging_data.emplace_back(point_t(t_rdata_msg->ranges[i]*std::cos(ang),
            t_rdata_msg->ranges[i]*std::sin(ang)));
    }
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
  double time = 0;
  double theta = 0;

  polygon_t poly;
  bg::append(poly.outer(), point_t(0.5*m_width, 0.5*m_length));  // top-left
  bg::append(poly.outer(), point_t(0.5*m_width, -0.5*m_length));  // top-right
  bg::append(poly.outer(), point_t(-0.5*m_width, -0.5*m_length));  // bottom-right
  bg::append(poly.outer(), point_t(-0.5*m_width, 0.5*m_length));  // bottom-left
  bg::append(poly.outer(), point_t(0.5*m_width, 0.5*m_length));  // closure

  CollisionChecker cc(/* check radius = */ 5);
  cc.set_points(m_ranging_data);

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
