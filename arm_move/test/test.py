#!/usr/bin/env python3

""" Create a unittest node that tests out the step service"""
import unittest
import rospy
import rostest
from geometry_msgs.msg import Pose, Point
from arm_move.srv import reset, step

class ArmMoveTests(unittest.TestCase):
    def __init__(self, *args):
        super(ArmMoveTests, self).__init__(*args)
        rospy.init_node("test_client")
        self.test_step()

    def test_step(self):
        rospy.wait_for_service("reset")
        Reset = rospy.ServiceProxy("reset", reset)
        rospy.wait_for_service("step")
        Step = rospy.ServiceProxy("step", step)
        box = Pose()
        box.position = Point(x=0.2,y=0.2,z=0.05)
        box.orientation.w = 1
        Reset(box_pose=box,clear_waypoints=True)
        resp = Step(position=Point(x=0.1,y=0,z=-0.1),gripper=False)
        self.assertEqual(resp.val, -1)

if __name__ == "__main__":
    import rostest
    rostest.rosrun('arm_move', "arm_move_tests", ArmMoveTests)