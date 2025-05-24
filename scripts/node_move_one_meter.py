#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time

class MoveForwardOneMeter:
    def __init__(self):
        rospy.init_node('move_forward_one_meter')

        self.pub_right = rospy.Publisher('/right_wheel_effort_controller/command', Float64, queue_size=10)
        self.pub_left = rospy.Publisher('/left_wheel_effort_controller/command', Float64, queue_size=10)
        self.pub_caster = rospy.Publisher('/caster_effort_controller/command', Float64, queue_size=10)

        rospy.sleep(1.0)

        self.effort = 1.5
        self.duration = 5.0

    def move(self):
        effort_msg = Float64(data=self.effort)

        self.pub_right.publish(effort_msg)
        self.pub_left.publish(effort_msg)
        self.pub_caster.publish(effort_msg)

        rospy.loginfo("Робот двигается вперёд...")
        time.sleep(self.duration)

        stop_msg = Float64(data=0.0)
        self.pub_right.publish(stop_msg)
        self.pub_left.publish(stop_msg)
        self.pub_caster.publish(stop_msg)

        rospy.loginfo("Робот остановился.")

if __name__ == '__main__':
    try:
        mover = MoveForwardOneMeter()
        mover.move()
    except rospy.ROSInterruptException:
        pass
