/*
 * Author: Dejun Guo
 */
#include <ros/ros.h>

#include "ubtech/nardc/navigation/collision_checker.hpp"
#include "ubtech/nardc/navigation/CollChecker.hpp"

#include "ubtnarc_collavoid/collavoid.h"  // message definition

namespace ubnn = ubtech::nardc::navigation;

int main(int argc, char** argv){
  ros::init(argc, argv, "collision avoidance module");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  int rate;
  pnh.param<int>("rate", rate, 50);
  ros::Rate loop_rate(rate);
  
  ubnn::CollChecker cc(nh, pnh);

  ubtnarc_collavoid::collavoid msg;
  ros::Publisher pub = nh.advertise<ubtnarc_collavoid::collavoid>("collision_avoidance", 100);

  while(ros::ok()){
    // check if all-clear
    ros::spinOnce();  // process callbacks
    auto const current_time = ros::Time::now();
    msg.all_clear = cc.is_safe();
    ROS_WARN("collision checker took %fmsec", 1000.*(ros::Time::now().toSec() - current_time.toSec()));
    msg.header.stamp = current_time;
    pub.publish(msg);
    loop_rate.sleep();
  }	
}
