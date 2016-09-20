#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "dq_robotics/dqdynamicConfig.h"

void callback(dq_robotics::dqdynamicConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f", 
            config.kp_param, config.ki_param, 
            config.kd_param);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "baxter_dq_control");

  dynamic_reconfigure::Server<dq_robotics::dqdynamicConfig> server;
  dynamic_reconfigure::Server<dq_robotics::dqdynamicConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  int a=1;
  ros::spin();
  return 0;
}
