#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <arduinobot_remote/ArduinobotTaskAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

class TaskServer
{
private:
    /* data */
    ros::NodeHandle _nodeHandle;
    actionlib::SimpleActionServer<arduinobot_remote::ArduinobotTaskAction> _actionServer;
    arduinobot_remote::ArduinobotTaskResult _result;
    std::vector<double> _arm_goal;
    std::vector<double> _gripper_goal;
    std::string _actionName;
    moveit::planning_interface::MoveGroupInterface _armMoveGroup;
    moveit::planning_interface::MoveGroupInterface _gripperMoveGroup;

public:
    TaskServer(std::string name);

    void executeCb(const arduinobot_remote::ArduinobotTaskGoalConstPtr &goal);
};

TaskServer::TaskServer(std::string name) : _actionServer(_nodeHandle, name, boost::bind(&TaskServer::executeCb, this, _1), false),
                                           _actionName(name),
                                           _armMoveGroup("arduinobot_arm"),
                                           _gripperMoveGroup("arduinobot_hand")
{
    _actionServer.start();
}

void TaskServer::executeCb(const arduinobot_remote::ArduinobotTaskGoalConstPtr &goal)
{
    bool success = true;

    if (goal->task_number == 0)
    {
        _arm_goal = {0.0, 0.0, 0.0};
        _gripper_goal = {-0.7, 0.7};
    }
    else if (goal->task_number == 1)
    {
        _arm_goal = {-1.14, -0.6, -0.07};
        _gripper_goal = {0.0, 0.0};
    }
    else if (goal->task_number == 2)
    {
        _arm_goal = {-1.57, 0.0, -1.0};
        _gripper_goal = {0.0, 0.0};
    }
    else
    {
        ROS_ERROR("Invalid goal");
        return;
    }

    _armMoveGroup.setJointValueTarget(_arm_goal);
    _gripperMoveGroup.setJointValueTarget(_gripper_goal);

    _armMoveGroup.move();
    _gripperMoveGroup.move();

    _armMoveGroup.stop();
    _gripperMoveGroup.stop();

    if (_actionServer.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("%s: Preempted", _actionName.c_str());
        _actionServer.setPreempted();
        success = false;
    }

    if (success)
    {
        _result.success = true;
        ROS_INFO("%s Succeeded", _actionName.c_str());
        _actionServer.setSucceeded(_result);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_server");
    TaskServer server("task_server");
    ros::spin();
}