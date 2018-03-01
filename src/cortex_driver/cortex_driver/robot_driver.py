#!/usr/bin/env python

import math
import numpy as np
from numpy import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import serial

def main():
	rospy.init_node('robot_driver')

	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
	odom_broadcaster = tf.TransformBroadcaster()

	cur_pose = np.zeros(3)  # x, y, hdg
	cur_vel = np.zeros(3)  # same order as above

	last_enc_left = None
	last_enc_right = None

	t = rospy.Time.now()
	last_t = rospy.Time.now()
	r = rospy.Rate(10.0)

	wheelbase = (21 * .5) * 0.0254  # meters
	wheel_circum = (4*pi) * 0.0254  # meters
	encoder_conv_factor = 627.2  # ticks per revolution

	def wait_for_start_byte(ser):
		start_t = rospy.Time.now()
		while True:
			start_byte = ser.read(1)
			if start_byte == b'\x55':
				return True

			if (rospy.Time.now() - start_t).to_sec() >= 0.050:
				return False

	with serial.Serial('/dev/serial0', 9600, timeout=0.050) as ser:
	    while not rospy.is_shutdown():
	        t = rospy.Time.now()
	        dt = (t - last_t).to_sec()

	        ser.write([0xAA, 0x02])
			ser.flush()

	        if wait_for_start_byte(ser):
	            # Response starting...
	            data = bytearray(ser.read(4))

				if len(data) < 4:
	                # timed out waiting for response data
	                continue

	            # reassemble encoder data
	            enc_right = data[0] | (data[1] << 8)
				enc_left = data[2] | (data[3] << 8)

				if enc_right > 0x7FFF:
					enc_right = -(0x10000 - enc_right)

				if enc_left > 0x7FFF:
					enc_left = -(0x10000 - enc_left)

	            if (last_enc_left is None) or (last_enc_right is None):
	                last_enc_left = enc_left
	                last_enc_right = enc_right
	                continue

	            # rotational velocity in ticks
	            rv_left = (enc_left - last_enc_left) / dt
	            rv_right = (enc_right - last_enc_right) / dt

	            last_enc_left = enc_left
	            last_enc_right = enc_right

	            # linear velocity (m/sec)
	            lv_left = rv_left * (wheel_circum / encoder_conv_factor)
	            lv_right = rv_right * (wheel_circum / encoder_conv_factor)

	            cur_vel = None
	            new_pose = None
	            if abs(lv_left - lv_right) <= (0.0254 / 2):
	                # wheels are within 0.5 in/sec of each other: assume straight-line motion
	                v_fwd = (lv_left + lv_right) / 2

	                rot_matx = np.array([
	                    cos(cur_pose[2]), -sin(cur_pose[2]), 0
	                    sin(cur_pose[2]), cos(cur_pose[2]), 0
	                    0, 0, 0
	                ])

	                new_pose = (rot_matx @ np.array([v_fwd, 0,0])) + cur_pose
	            else:
	                # non-straight line motion
	                # see: https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
	                R = ((lv_left + lv_right) / (lv_right - lv_left)) * (wheelbase / 2)
	                ang_vel = (lv_right - lv_left) / wheelbase

	                icc = cur_pose + np.array([
	                    -R * sin(cur_pose[2]),
	                    R * cos(cur_pose[2]),
	                    ang_vel * dt
	                ])

	                rot_matx = np.array([
	                    [cos(ang_vel * dt), -sin(ang_vel * dt), 0],
	                    [sin(ang_vel * dt), cos(ang_vel * dt), 0],
	                    [0, 0, 1],
	                ])

	                # update pose
	                new_pose = (rot_matx @ np.array([
	                    cur_pose[0] - icc[0],
	                    cur_pose[1] - icc[1],
	                    cur_pose[2]
	                ])) + icc

	            cur_vel = (new_pose - cur_pose) / dt
	            cur_pose = new_pose

	            odom_quat = tf.transformations.quaternion_from_euler(0, 0, cur_pose[2])

	            # publish odometry transform
	            odom_broadcaster.sendTransform(
	                (cur_pose[0], cur_pose[1], 0),
	                odom_quat,
	                t,
	                "base_link",
	                "odom"
	            )

	            # publish odometry message
	            odom_msg = Odometry()
	            odom.header.stamp = t
	            odom.header.frame_id = "odom"
	            odom.pose.pose = Pose(Point(cur_pose[0], cur_pose[1], 0), Quaternion(*odom_quat))

	            odom.child_frame_id = "base_link"
	            odom.twist.twist = Twist(
	                Vector3(cur_vel[0], cur_vel[1], 0),
	                Vector3(0, 0, cur_vel[2])
	            )

	            odom_pub.publish(odom)

	            last_t = t
	            r.sleep()

if __name__ == '__main__':
	main()
