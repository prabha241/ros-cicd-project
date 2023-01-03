#!/usr/bin/env python

from geometry_msgs.msg import Twist

import rospy


class Rotator():

    def __init__(self):
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        if rospy.get_param('ROTATION_SPEED'):
            self.rotation_speed = rospy.get_param('ROTATION_SPEED')
        else:
            self.rotation_speed = 0.2
    def rotate_forever(self):
        self.twist = Twist()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.twist.angular.z = self.rotation_speed
            self._cmd_pub.publish(self.twist)
            rospy.loginfo('Rotating robot: %s', self.twist)
            r.sleep()


def main():
    rospy.init_node('rotate')
    try:
        rotator = Rotator()
        rotator.rotate_forever()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
