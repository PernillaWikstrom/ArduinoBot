#pragma once
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <vector>

class ArduinobotInterface : public hardware_interface::RobotHW
{
public:
    ArduinobotInterface(ros::NodeHandle &);
    void update(const ros::TimerEvent &event);
    void read();
    void write(ros::Duration);

private:
    ros::NodeHandle _nodeHandle;
    ros::NodeHandle _privateNodeHandle;
    ros::Duration _elapsedTime;
    ros::Duration _updateFrequency;
    ros::Timer _looper;
    ros::Publisher _hardwarePublisher;
    ros::ServiceClient _hardwareService;

    hardware_interface::JointStateInterface _jointStateInterface;
    hardware_interface::PositionJointInterface _jointPositionInterface;
    boost::shared_ptr<controller_manager::ControllerManager> _controllerManager;

    std::vector<double> _cmd;
    std::vector<double> _angle;
    std::vector<double> _rate;
    std::vector<double> _torque;
    std::vector<std::string> _names;
};
