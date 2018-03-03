#!/usr/bin/env python
import rospy
import numpy as np
from numpy import pi
from geometry_msgs.msg import Twist
from cortex_driver.srv import *

wheelbase = (21 * .5) * 0.0254  # meters
wheel_circum = (4*pi) * 0.0254  # meters
encoder_conv_factor = 627.2  # ticks per revolution
max_vel = 850  # ticks/sec

def send_motor_velocity(left_vel, right_vel):
    rospy.wait_for_service('motor_vel')
    try:
        mp_prox = rospy.ServiceProxy('motor_vel', MotorVel)
        mp_prox(left_vel, right_vel)
    except rospy.ServiceException as e:
        rospy.logwarn("motor_vel service call failed: {}".format(str(e)))

def handle_cmd_vel(twist):
    rospy.loginfo("Got message over cmd_vel: ".format(str(twist)))

    max_omega = ((2 * max_vel) / wheelbase) * (wheel_circum / encoder_conv_factor)
    max_vfwd = max_vel * (wheel_circum / encoder_conv_factor)

    omega = np.clip(twist.angular.z, -max_omega, max_omega)
    v_fwd = np.clip(twist.linear.x, -max_vfwd, max_vfwd)

    v_left = (v_fwd - omega * wheelbase / 2.0) * (encoder_conv_factor / wheel_circum)
    v_right = (v_fwd + omega * wheelbase / 2.0) * (encoder_conv_factor / wheel_circum)

    send_motor_velocity(v_left, v_right)

def main():
    rospy.init_node('base_control')

    rospy.loginfo("Waiting for motor_vel service...")
    rospy.wait_for_service('motor_vel')

    rospy.loginfo("Listening on cmd_vel...")
    sub = rospy.Subscriber("cmd_vel",  Twist, handle_cmd_vel)

    rospy.spin()

if __name__ == '__main__':
    main()
