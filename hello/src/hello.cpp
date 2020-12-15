#include <string.h>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "hello_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.5);
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  int count = 0;
  while (ros::ok()){
    ROS_DEBUG("log:%d", count);
    ROS_INFO("log:%d", count);
    ROS_WARN("log:%d", count);
    ROS_ERROR("log:%d", count);
    ROS_FATAL("log:%d", count);

    count++;
    if(count>=10){
      count=0;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}