/*
 * Author: Dejun Guo
 */
#pragma once
// headers
#ifndef Included_NameModel_H
#define Included_NameModel_H
#include <ros/ros.h>
#include <vector>
#include <boost/geometry.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
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
  /**
  * CollChecker()
  *
  * @brief construct empty CollChecker; 
  *
  * @note initialize robot's size, maximum velocity, prediction time for collavoid., initial position
  */
  CollChecker (ros::NodeHandle);
  /**
  * publish_msg()
  *
  * @brief publish the filtered cmd_vel
  */
  void publish_msg();
  /**
  * look_ahead()
  *
  * @brief look ahead in a given time to check if robot will go into obstacles
  */
  void look_ahead(double const);
  /**
  * kinematics()
  *
  * @brief global function - simulate kinematics
  */
  void kinematics(geometry_msgs::Vector3 const, geometry_msgs::Vector3&, double const);
  /**
  * intersection_checker()
  *
  * @brief collision avoidance based on point-polygon intersection checking
  */
  void intersection_checker(geometry_msgs::Vector3,geometry_msgs::Vector3,geometry_msgs::Vector3&,
     std::vector<geometry_msgs::Vector3>, double const);
  /**
  * Transf_lTo()
  *
  * @brief transfermation from world frame to base frame
  */
  void Transf_lTo(geometry_msgs::Vector3 const,geometry_msgs::Vector3 const,geometry_msgs::Vector3&);
  /**
  * Transf_oTl()
  *
  * @brief transfermation from base frame to world frame
  */
  void Transf_oTl(geometry_msgs::Vector3 const,geometry_msgs::Vector3 const,geometry_msgs::Vector3&);
private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_joy_sub,m_tf_sub,m_joy2_sub, m_mb_sub,m_rdata_sub;
  ros::Publisher m_curP_pub;
  geometry_msgs::Vector3 m_P,m_V_cmd,m_V_fil;
  geometry_msgs::Twist m_cmdvel_fil;
  bool m_if_manual;
  double m_t_pred;
  double m_lt,m_lb,m_rt,m_rb;
  double angle_min,angle_max,angle_increment;
  double m_max_V,m_max_w;
  geometry_msgs::Vector3 m_P_laser;
  points_t m_ranging_data;
  /**
  * cmdvel_callback() rdata_callback()
  *
  * @brief subscribe cmd_vel and ranging data
  */
  void cmdvel_callback(const geometry_msgs::Twist::ConstPtr&);
  void tf_callback(const tf2_msgs::TFMessage::ConstPtr&);
  void cmdvelmb_callback(const geometry_msgs::Twist::ConstPtr&);
  void rdata_callback(const sensor_msgs::LaserScan::ConstPtr&);
  void gamepad_callback(const sensor_msgs::Joy::ConstPtr&);
};
CollChecker::CollChecker(ros::NodeHandle t_handle): m_if_manual(false) {
  m_max_V=0.75;m_max_w=1;
  m_P.x=0;m_P.y=0;m_nh=t_handle;
  m_curP_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  m_joy_sub = m_nh.subscribe<geometry_msgs::Twist>("cmd_vel_joy", 10, &CollChecker::cmdvel_callback,this);
  m_tf_sub = m_nh.subscribe<tf2_msgs::TFMessage>("tf_static", 10, &CollChecker::tf_callback,this);
  //m_mb_sub = m_nh.subscribe<geometry_msgs::Twist>("cmd_vel_kinematic", 10, &CollChecker::cmdvelmb_callback,this);
  m_mb_sub = m_nh.subscribe<geometry_msgs::Twist>("cmd_vel_movebase", 10, &CollChecker::cmdvelmb_callback,this);
  m_rdata_sub = m_nh.subscribe<sensor_msgs::LaserScan>("scan_filtered", 10, &CollChecker::rdata_callback,this);
  m_joy2_sub = m_nh.subscribe<sensor_msgs::Joy>("joy", 10, &CollChecker::gamepad_callback, this);
  //m_lx=0.9/2+0.15;m_ly=0.5/2+0.15;
  m_lt=0.05;m_lb=0.3;m_rt=0.05;m_rb=0.3;
  m_t_pred=1;
}
void CollChecker::tf_callback(const tf2_msgs::TFMessage::ConstPtr& tf_msg_in) {
  for (int i=0;i<tf_msg_in->transforms.size();++i) {
    if(tf_msg_in->transforms[i].child_frame_id=="hokuyo_link") {
      m_P_laser.x=tf_msg_in->transforms[i].transform.translation.x;
      m_P_laser.y=tf_msg_in->transforms[i].transform.translation.y;
      geometry_msgs::Quaternion ori;
      ori=tf_msg_in->transforms[i].transform.rotation;
      tf::Quaternion q(ori.x,ori.y,ori.z,ori.w);
      tf::Matrix3x3 m(q);
      geometry_msgs::Vector3 ang;
      m.getRPY(ang.x,ang.y,m_P_laser.z);
      //std::cout<<m_P_laser.x<<" "<<m_P_laser.y<<" "<<m_P_laser.z<<std::endl;
      break;
    }
  }
}
void CollChecker::cmdvel_callback(const geometry_msgs::Twist::ConstPtr& t_cmdvel_msg){
  if (m_if_manual) {
    m_V_cmd.x=t_cmdvel_msg->linear.x*m_max_V;
    m_V_cmd.y=t_cmdvel_msg->angular.z*m_max_w;
  }
}
void CollChecker::cmdvelmb_callback(const geometry_msgs::Twist::ConstPtr& t_cmdvel_msg){
  if (!m_if_manual) {
    m_cmdvel_fil.linear.x=t_cmdvel_msg->linear.x;
    m_cmdvel_fil.angular.z=t_cmdvel_msg->angular.z;
    m_curP_pub.publish(m_cmdvel_fil);
  }
}
void CollChecker::gamepad_callback(const sensor_msgs::Joy::ConstPtr& t_joy) {
  if (t_joy->buttons[5]==1) m_if_manual=true;
  if (t_joy->buttons[4]==1) m_if_manual=false;
}
void CollChecker::rdata_callback(const sensor_msgs::LaserScan::ConstPtr& t_rdata_msg){
  m_ranging_data.clear();
  angle_min=t_rdata_msg->angle_min;
  angle_max=t_rdata_msg->angle_max;
  angle_increment=t_rdata_msg->angle_increment;
  for (int i=0;i<t_rdata_msg->ranges.size();i++){	
    double ang;
    ang=angle_min+angle_increment*i;
    m_ranging_data.push_back(point_t(t_rdata_msg->ranges[i]*cos(ang),t_rdata_msg->ranges[i]*sin(ang)));
  }
}
void CollChecker::publish_msg(){
  if (m_if_manual) {
    m_curP_pub.publish(m_cmdvel_fil);
  }
}
void CollChecker::kinematics(geometry_msgs::Vector3 const t_V, geometry_msgs::Vector3& t_P, double const t_T){
  geometry_msgs::Vector3 Pdot;
  Pdot.x=t_V.x*cos(t_P.z);
  Pdot.y=t_V.x*sin(t_P.z);
  Pdot.z=t_V.y;//yaw
  t_P.x+=Pdot.x*t_T;
  t_P.y+=Pdot.y*t_T;
  t_P.z+=Pdot.z*t_T;//yaw
}
void CollChecker::look_ahead(double const t_T){
  if (m_if_manual) {
    std::vector<geometry_msgs::Vector3> P_pred;
    geometry_msgs::Vector3 P_temp;
    P_temp.x=0;P_temp.y=0;P_temp.z=0;
    P_pred.push_back(P_temp);
    for (int i=0;i<int(m_t_pred/0.1);i++){
      kinematics(m_V_cmd,P_temp,0.1);
      P_pred.push_back(P_temp);
    }
    intersection_checker(m_P,m_V_cmd,m_V_fil,P_pred,t_T);
    kinematics(m_V_fil,m_P,t_T);
    m_cmdvel_fil.linear.x=m_V_fil.x;
    m_cmdvel_fil.angular.z=m_V_fil.y;
    //m_cmdvel_fil.linear.x=m_V_cmd.x;
    //m_cmdvel_fil.angular.z=m_V_cmd.y;
  }
}


void CollChecker::intersection_checker(geometry_msgs::Vector3 P,geometry_msgs::Vector3 V_in,geometry_msgs::Vector3 & V_fil, std::vector<geometry_msgs::Vector3> P_pred, double const T){
  CollisionChecker cc(5);
  cc.set_points(m_ranging_data);
  while(!P_pred.empty()){
    geometry_msgs::Vector3 P_cen,q_b;
    P_cen=P_pred.back();P_pred.pop_back();
    //set robot's boundary in world frame
    geometry_msgs::Vector3 q1b,q2b,q3b,q4b,q1,q2,q3,q4;
    q_b.x= 0.1;q_b.y= 0.15;Transf_oTl(P_cen,q_b,q1b);Transf_lTo(m_P_laser,q1b,q1);
    q_b.x= 0.1;q_b.y=-0.15;Transf_oTl(P_cen,q_b,q2b);Transf_lTo(m_P_laser,q2b,q2);
    q_b.x=-0.25;q_b.y=-0.15;Transf_oTl(P_cen,q_b,q3b);Transf_lTo(m_P_laser,q3b,q3);
    q_b.x=-0.25;q_b.y= 0.15;Transf_oTl(P_cen,q_b,q4b);Transf_lTo(m_P_laser,q4b,q4);
    std::vector<point_t> robot_corners = {{q1.x, q1.y}, {q2.x, q2.y}, {q3.x, q3.y}, {q4.x, q4.y}, {q1.x, q1.y}};
    polygon_t polygon;
    boost::geometry::assign_points(polygon, robot_corners);
    //check collision avoidance
    if (cc.check_intersection(polygon)){
      V_fil.x=0;V_fil.y=0;
      break;
    }
    else
      V_fil=V_in;
  }
}
void CollChecker::Transf_lTo(geometry_msgs::Vector3 const t_P_car,geometry_msgs::Vector3 const q_w,geometry_msgs::Vector3& q_b){
  Eigen::Matrix3d wRb= Eigen::Matrix3d::Identity();
  Eigen::Vector3d Pw,Pb;
  Pw(0,0)=q_w.x;Pw(1,0)=q_w.y;Pw(2,0)=1;
  wRb(0,0)=cos(t_P_car.z);wRb(0,1)=-sin(t_P_car.z);wRb(0,2)=t_P_car.x;
  wRb(1,0)=sin(t_P_car.z);wRb(1,1)= cos(t_P_car.z);wRb(1,2)=t_P_car.y;
  Pb=wRb.inverse()*Pw;
  q_b.x=Pb(0,0);q_b.y=Pb(1,0);
}
void CollChecker::Transf_oTl(geometry_msgs::Vector3 const P_car,geometry_msgs::Vector3 const q_b,geometry_msgs::Vector3& q_w){
  Eigen::Matrix3d wRb= Eigen::Matrix3d::Identity();
  Eigen::Vector3d Pw,Pb;
  Pb(0,0)=q_b.x;Pb(1,0)=q_b.y;Pb(2,0)=1;
  wRb(0,0)=cos(P_car.z);wRb(0,1)=-sin(P_car.z);wRb(0,2)=P_car.x;
  wRb(1,0)=sin(P_car.z);wRb(1,1)= cos(P_car.z);wRb(1,2)=P_car.y;
  Pw=wRb*Pb;
  q_w.x=Pw(0,0);q_w.y=Pw(1,0);
}
}  // end namespace navigation
}  // end namespace nardc
}  // end namespace ubtech
#endif // Included_NameModel_H 
