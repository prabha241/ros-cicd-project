#!/usr/bin/env python


import rospy
import rostest
import time
import os
import unittest
from rosgraph_msgs.msg import Clock
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags
from geometry_msgs.msg import Twist
import boto3

class SimulationUtils():

    def cancel_job(self):
        rospy.loginfo("Successfully requested cancel job")
        rospy.loginfo('byebyee')
        rospy.wait_for_service('/robomaker/job/cancel')
        requestCancel = rospy.ServiceProxy('/robomaker/job/cancel', Cancel)
        response = requestCancel()
        if response.success:
            self.is_cancelled = True
        else:
            rospy.logerr("Cancel request failed: %s", response.message)

    def set_tag(self,name, value):
        rospy.wait_for_service('/robomaker/job/add_tags')
        requestAddTags = rospy.ServiceProxy('/robomaker/job/add_tags', AddTags)
        tags = ([Tag(key=name, value=value)])
        response = requestAddTags(tags)
        if response.success:
            rospy.loginfo("Successfully added tags: %s", tags)
        else:
            rospy.logerr("Add tags request failed for tags (%s): %s", tags, response.message)

class RotatorTimeTest(unittest.TestCase):
    
    def setUp(self):
        self.dynamic_speed = False
        if os.getenv('ROTATION_SPEED'):
            self.dynamic_speed = True
            self.start_time = time.time()
            self.rotation_speed = float(os.getenv('ROTATION_SPEED'))
        else:
            self.rotation_speed = 0.2
        self.test_name = 'rotate_test'
        self.utils = SimulationUtils()
        self.is_completed = False
        self.speed_check = False
        if os.getenv('TIME_TEST_LENGTH_IN_SECONDS'):
            self.test_time = float(os.getenv('TIME_TEST_LENGTH_IN_SECONDS'))
        else:
            self.test_time = float(10)
    def check_speed(self,msg):
        if msg.angular.z == self.rotation_speed:
            self.speed_check = True
    def check_complete(self,msg):
        if msg.clock.secs > self.test_time and self.is_completed == False:
            if self.dynamic_speed == True:
                self.utils.set_tag(name = self.test_name + "_Time_Elapsed_Started_with_dynamic_speed" , value = "Passed")
            else:
                self.utils.set_tag(name = self.test_name + "_Time_Elapsed_Started_with_dynamic_speed" , value = "Failed")
            self.is_completed == True
            self.utils.cancel_job()
    def test_rotator_time(self):
        rospy.Subscriber('/cmd_vel', Twist, self.check_speed)
        try:
            rospy.Subscriber('/clock', Clock, self.check_complete)
            self.utils.set_tag(name = self.test_name + "_Time_Elapsed_Started" , value = "Passed")
            rospy.spin()
        except:
            self.utils.set_tag(name = self.test_name + "_Time_Elapsed_Status" , value = "Failed")
            self.utils.cancel_job()

    def runTest(self):
        self.test_rotator_time()

if __name__ == "__main__":
    rospy.init_node("rotate_test", log_level=rospy.INFO)
    rostest.rosrun("hello_world_robot", "rotate_test", RotatorTimeTest)
