#!/usr/bin/env python
from rotate import Rotator

import rospy
import rostest
import time
import os
import unittest

from rosgraph_msgs.msg import Clock
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags
from geometry_msgs.msg import Twist

class SimulationUtils():

    def cancel_job(self):
        rospy.wait_for_service('/robomaker/job/cancel')
        requestCancel = rospy.ServiceProxy('/robomaker/job/cancel', Cancel)
        response = requestCancel()
        if response.success:
            self.is_cancelled = True
            rospy.loginfo("Successfully requested cancel job")
        else:
            rospy.logerr("Cancel request failed: %s", response.message)

    def set_tag(self,name, value):
        rospy.loginfo("inside tag")
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
        rospy.loginfo(self.rotation_speed)
    
        self.test_name = 'rotate_test'
        self.utils = SimulationUtils()
        self.is_completed = False
        self.speed_check = False
        if os.getenv('TIME_TEST_LENGTH_IN_SECONDS'):
            self.test_time = float(os.getenv('TIME_TEST_LENGTH_IN_SECONDS'))
        else:
            self.test_time = float(10)
    
    def check_speed(self,msg):
        rospy.loginfo("inside check")
        if msg.angular.z == self.rotation_speed and self.is_completed == False and self.dynamic_speed:
            rospy.loginfo(type(msg.angular.z))
            rospy.loginfo("check speed")
            self.speed_check = True
            self.check_complete_dynamic_speed()
            pass
        else:
            rospy.loginfo(type(msg.angular.z))
            rospy.loginfo("no check speed")
            self.check_complete_standard_speed()
            pass
            
    def check_complete_dynamic_speed(self):
        rospy.loginfo("check2")
        self.time_now = time.time()
        rospy.loginfo(self.time_now-self.start_time)
        if self.time_now-self.start_time > self.test_time and self.is_completed == False:
            rospy.loginfo(self.time_now-self.start_time)
            rospy.loginfo("done1")
            self.is_completed = True
            self.utils.set_tag(name = self.test_name + "_with_dynamic_speed" , value = "Passed")
            rospy.loginfo("dynamic_speed")
            self.utils.cancel_job()
            
    def check_complete_standard_speed(self):
        rospy.loginfo("check3")
        self.time_now = time.time()
        rospy.loginfo(self.time_now-self.start_time)
        if self.time_now-self.start_time > self.test_time and self.is_completed == False:
            rospy.loginfo("done")
            rospy.loginfo("standard speed")
            self.is_completed == True
            self.utils.set_tag(name = self.test_name + "_with_standard_speed" , value = "Passed")
            self.utils.cancel_job()
    def test_rotator_time(self):
        try:
            rospy.Subscriber("/cmd_vel", Twist, self.check_speed)
            rospy.spin()
        except:
            self.utils.set_tag(name = self.test_name + "_Time_Elapsed_Status" , value = "Failed")
            self.utils.cancel_job()

    def runTest(self):
        self.test_rotator_time()

if __name__ == "__main__":
    rospy.init_node("rotate_test", log_level=rospy.INFO)
    rostest.rosrun("hello_world_robot", "rotate_test", RotatorTimeTest)
