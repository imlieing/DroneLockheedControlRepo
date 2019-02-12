#include <ros/ros.h>
#include "autonomous.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autonomous_control");

  autonomous_control::autonomous t;

  ros::Rate r(20);
  while(ros::ok()) {
    ros::spinOnce();
    t.control();
    r.sleep();
  }
}
