/*
 * Author: Dejun Guo
 */
#include "ubtech/nardc/navigation/collision_checker.hpp"
#include "ubtech/nardc/navigation/CollChecker.hpp"
namespace ubnn = ubtech::nardc::navigation;
int main(int argc, char** argv){
     ros::init(argc, argv, "simulator_node");
     ros::NodeHandle nh;
     double T=1/50.0;
     ros::Rate loop_rate(1/T);
     ubnn::CollChecker cc(nh);
     while(ros::ok()){
          // simulate
          cc.look_ahead(T);
          // publish data to channels
          cc.publish_msg();
          ros::spinOnce();
          loop_rate.sleep();
     }	
}
