#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_node");
  nodelet::Loader manager(true);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;
  manager.load(ros::this_node::getName(), "camera_aravis/CameraNodelet", remappings, my_argv);
  ros::spin();
}
