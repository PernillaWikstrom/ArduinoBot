#include <arduinobot_controller/arduinobot_interface.h>
#include <std_msgs/UInt16MultiArray.h>
#include <arduinobot_controller/AnglesConverter.h>

ArduinobotInterface::ArduinobotInterface(ros::NodeHandle &nh) : _nodeHandle(nh),
                                                                _privateNodeHandle("~"),
                                                                _angle(4, 0),
                                                                _rate(4, 0),
                                                                _torque(4, 0),
                                                                _cmd(4, 0),
                                                                _names{"joint_1", "joint_2", "joint_3", "joint_4"}

{
    _privateNodeHandle.param("joint_names", _names, _names);
    _hardwarePublisher = _privateNodeHandle.advertise<std_msgs::UInt16MultiArray>("/arduino/arm_actuate", 1000);
    _hardwareService = _privateNodeHandle.serviceClient<arduinobot_controller::AnglesConverter>("/radians_to_degrees");

    ROS_INFO("DBGPW: Starting Arduinobot Hardware Interface...");

    hardware_interface::JointStateHandle stateHandle1(_names.at(0), &_angle.at(0), &_rate.at(0), &_torque.at(0));
    _jointStateInterface.registerHandle(stateHandle1);
    hardware_interface::JointStateHandle stateHandle2(_names.at(1), &_angle.at(1), &_rate.at(1), &_torque.at(1));
    _jointStateInterface.registerHandle(stateHandle2);
    hardware_interface::JointStateHandle stateHandle3(_names.at(2), &_angle.at(2), &_rate.at(2), &_torque.at(2));
    _jointStateInterface.registerHandle(stateHandle3);
    hardware_interface::JointStateHandle stateHandle4(_names.at(3), &_angle.at(3), &_rate.at(3), &_torque.at(3));
    _jointStateInterface.registerHandle(stateHandle4);
    registerInterface(&_jointStateInterface);

    hardware_interface::JointHandle positionHandle1(_jointStateInterface.getHandle(_names.at(0)), &_cmd.at(0));
    _jointPositionInterface.registerHandle(positionHandle1);
    hardware_interface::JointHandle positionHandle2(_jointStateInterface.getHandle(_names.at(1)), &_cmd.at(1));
    _jointPositionInterface.registerHandle(positionHandle2);
    hardware_interface::JointHandle positionHandle3(_jointStateInterface.getHandle(_names.at(2)), &_cmd.at(2));
    _jointPositionInterface.registerHandle(positionHandle3);
    hardware_interface::JointHandle positionHandle4(_jointStateInterface.getHandle(_names.at(3)), &_cmd.at(3));
    _jointPositionInterface.registerHandle(positionHandle4);
    registerInterface(&_jointPositionInterface);

    ROS_INFO("DBGPW: Interfaces registered.");
    ROS_INFO("DBGPW: Preparing the Controller Manager");

    _controllerManager.reset(new controller_manager::ControllerManager(this, _nodeHandle));
    _updateFrequency = ros::Duration(0.1);
    _looper = _nodeHandle.createTimer(_updateFrequency, &ArduinobotInterface::update, this);

    ROS_INFO("DBGPW: Ready to execute the control loop");
}

void ArduinobotInterface::update(const ros::TimerEvent &event)
{
    _elapsedTime = ros::Duration(event.current_real - event.last_real);
    read();
    _controllerManager->update(ros::Time::now(), _elapsedTime);
    write(_elapsedTime);
}

void ArduinobotInterface::read()
{
    _angle.at(0) = _cmd.at(0);
    _angle.at(1) = _cmd.at(1);
    _angle.at(2) = _cmd.at(2);
    _angle.at(3) = _cmd.at(3);
}

void ArduinobotInterface::write(ros::Duration elapsedTime)
{
    arduinobot_controller::AnglesConverter srv;
    srv.request.base = _cmd.at(0);
    srv.request.shoulder = _cmd.at(1);
    srv.request.elbow = _cmd.at(2);
    srv.request.gripper = _cmd.at(3);

    if (_hardwareService.call(srv))
    {
        std::vector<unsigned int> anglesDeg;
        anglesDeg.push_back(srv.response.base);
        anglesDeg.push_back(srv.response.shoulder);
        anglesDeg.push_back(srv.response.elbow);
        anglesDeg.push_back(srv.response.gripper);

        std_msgs::UInt16MultiArray msg;
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = anglesDeg.size();
        msg.layout.dim[0].stride = 1;

        msg.data.clear();
        msg.data.insert(msg.data.end(), anglesDeg.begin(), anglesDeg.end());

        _hardwarePublisher.publish(msg);
    }
    else
    {
        ROS_ERROR("Failed to code radians_to_degrees_service");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arduinobot_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    ArduinobotInterface robot(nh);
    spinner.spin();
    return 0;
}
