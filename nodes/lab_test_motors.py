#!/usr/bin/env python
# license removed for brevity

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import random


if __name__ == '__main__':

    rospy.init_node('labtestmotors', anonymous=True)

    command_limit = 200.
    time_period_limit = 20.

    lpub = rospy.Publisher("/left_thrust_cmd",Float32,queue_size=1)
    rpub = rospy.Publisher("/right_thrust_cmd",Float32,queue_size=1)

    thrust_on = True
    publish_thrust = True
    time_period = random.uniform(0, time_period_limit)
    time_last = rospy.get_rostime()

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            if publish_thrust:
                if thrust_on:
                    thrust_msg = Float32()
                    thrust_cmd = random.uniform(0, command_limit)
                    thrust_msg.data = thrust_cmd
                    lpub.publish(thrust_cmd)
                    rpub.publish(thrust_cmd)
            rate.sleep()
            if rospy.get_rostime() > time_last + time_period:
                time_last = rospy.get_rostime()
                time_period = random.uniform(0, time_period_limit)
                thrust_on = bool(random.randint(0, 1))
                publish_thrust = bool(random.randint(0, 1))
    except rospy.ROSInterruptException:
        pass
