#! usr/bin/env python3
import sys
import rospy
import actionlib
import moveit_commander
from arduinobot_remote.msg import ArduinobotTaskAction, ArduinobotTaskResult


class TaskServer(object):
    result = ArduinobotTaskResult()
    arm_goal = []
    gripper_goal = []

    def __init__(self, name: str) -> None:
        self.action_name = name
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_move_group = moveit_commander.MoveGroupCommander(
            "arduinobot_arm")
        self.hand_move_group = moveit_commander.MoveGroupCommander(
            "arduinobot_hand")
        self.actionServer = actionlib.SimpleActionServer(
            self.action_name, ArduinobotTaskAction, execute_cb=self.executeCb, auto_start=False)
        self.actionServer.start()

    def executeCb(self, goal):
        success = True
        if goal.task_number == 0:
            self.arm_goal = [0.0, 0.0, 0.0]
            self.gripper_goal = [-0.7, 0.7]
        elif goal.task_number == 1:
            self.arm_goal = [-1.14, -0.6, -0.07]
            self.gripper_goal = [0., 0.]
        elif goal.task_number == 2:
            self.arm_goal = [-1.57, 0., -1.0]
            self.gripper_goal = [0., 0.]
        else:
            rospy.loginfo("Invalid goal")
            return

        self.arm_move_group.go(self.arm_goal, wait=True)
        self.hand_move_group.go(self.gripper_goal, wait=True)
        self.arm_move_group.stop()
        self.hand_move_group.stop()

        if self.actionServer.is_preempt_requested():
            rospy.loginfo("%s is preempted" % self.action_name)
            self.actionServer.set_preempted()
            success = False

        if success:
            self.result.success = True
            rospy.loginfo("%s Succeeded" % self.action_name)
            self.actionServer.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node("task_server")
    ts = TaskServer(rospy.get_name())
    rospy.spin()
