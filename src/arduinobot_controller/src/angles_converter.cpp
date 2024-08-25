#include "ros/ros.h"
#include "arduinobot_controller/AnglesConverter.h"
#include "math.h"

bool radToDeg(arduinobot_controller::AnglesConverter::Request &req,
              arduinobot_controller::AnglesConverter::Response &res)
{
  res.base = static_cast<int>(((req.base + M_PI_2) * 180.f) / M_PI);
  res.shoulder = 180 - static_cast<int>(((req.shoulder + M_PI_2) * 180.f) / M_PI);
  res.elbow = static_cast<int>(((req.elbow + M_PI_2) * 180.f) / M_PI);
  res.gripper = static_cast<int>(((-req.gripper) * 180.f) / M_PI_2);
  return true;
}

bool degToRad(arduinobot_controller::AnglesConverter::Request &req,
              arduinobot_controller::AnglesConverter::Response &res)
{
  res.base = ((M_PI * req.base) - ((M_PI * 0.5f) * 180.f)) / 180.f;
  res.shoulder = (((180.f - req.shoulder) * M_PI) - (M_PI_2 * 180.f)) / 180.f;
  res.elbow = ((M_PI * req.elbow) - (M_PI_2 * 180.f)) / 180.f;
  res.gripper = -(M_PI_2 * req.gripper) / 180.f;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "angles_converter");
  ros::NodeHandle n;

  ros::ServiceServer radians_to_degrees = n.advertiseService("radians_to_degrees", radToDeg);
  ros::ServiceServer degrees_to_radians = n.advertiseService("degrees_to_radians", degToRad);

  ROS_INFO("Angles Converter Service Started");
  ros::spin();
  return 0;
}