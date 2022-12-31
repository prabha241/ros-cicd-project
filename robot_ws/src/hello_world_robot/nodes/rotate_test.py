#!/usr/bin/env python
from rotate import Rotator

import rospy
import rostest
import time
import os
import unittest

from rosgraph_msgs.msg import Clock
from ros_monitoring_msgs.msg import MetricList
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags

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
        if rospy.get_param('ROTATION_SPEED'):
            self.dynamic_speed = True
            self.rotation_speed = rospy.get_param('ROTATION_SPEED')
        else:
            self.rotation_speed = 0.2
    
        self.test_name = 'rotate_test'
        self.utils = SimulationUtils()
        self.is_completed = False
        if rospy.get_param('TIME_TEST_LENGTH_IN_SECONDS'):
            self.test_time = rospy.get_param('TIME_TEST_LENGTH_IN_SECONDS')
        else:
            self.test_time = 60
    
    def check_speed(self):
        if msg.angular.z == self.rotation_speed and self.is_completed == False and self.dynamic_speed:
            return True
        else:
            return False
            
    def check_complete_dynamic_speed(self):
        if msg.clock.secs > self.test_time and self.is_completed == False:
            self.is_completed == True
            self.utils.set_tag(name = self.test_name + "_with_dynamic_speed" , value = "Passed")
            self.utils.cancel_job()
            
    def check_complete_standard_speed(self):
        if msg.clock.secs > self.test_time and self.is_completed == False:
            self.is_completed == True
            self.utils.set_tag(name = self.test_name + "_with_standard_speed" , value = "Passed")
            self.utils.cancel_job()
    def test_rotator_time(self):
        try:
            self.speed_check = rospy.Subscriber("/cmd_vel", Twist, self.check_speed)
            if self.speed_check:
                self.clock = rospy.Subscriber('/clock', Clock, self.check_complete_dynamic_speed)
                self.utils.set_tag(name = self.test_name + "_Time_Elapsed_Started" , value = "Passed")
            else:
                self.clock = rospy.Subscriber('/clock', Clock, self.check_complete_standard_speed)
                self.utils.set_tag(name = self.test_name + "_Time_Elapsed_Started" , value = "Passed")
            rospy.spin()
        except:
            self.utils.set_tag(name = self.test_name + "_Time_Elapsed_Status" , value = "Failed")
            self.utils.cancel_job()

    def runTest(self):
        self.test_rotator_time()

if __name__ == "__main__":
    rospy.init_node("rotate_test", log_level=rospy.INFO)
    rostest.rosrun("helloworld_robot", "rotate_test", RotatorTimeTest)
