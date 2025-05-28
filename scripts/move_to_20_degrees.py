#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math

def publish_position(angle_deg):
    rospy.init_node('rotate_joint_position_node')
    pub = rospy.Publisher('/front_wheel_position_controller/command', Float64, queue_size=10)

    angle_rad = math.radians(angle_deg)
    rospy.sleep(1)
    pub.publish(Float64(angle_rad))
    rospy.loginfo(f"Published {angle_deg} deg = {angle_rad:.3f} rad to base_abc_joint_position_controller")

if __name__ == '__main__':
    publish_position(20)

