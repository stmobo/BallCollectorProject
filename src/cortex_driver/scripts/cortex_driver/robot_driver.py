#!/usr/bin/env python

import math
import numpy as np
from numpy import sin, cos, pi
import struct

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Int16, Float64
from cortex_driver.srv import *

import serial

def main():
    rospy.init_node('robot_driver')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    enc_left_pub = rospy.Publisher("enc_left", Int16, queue_size=10)
    enc_right_pub = rospy.Publisher("enc_right", Int16, queue_size=10)

    dist_left_pub = rospy.Publisher("dist_left", Float64, queue_size=10)
    dist_right_pub = rospy.Publisher("dist_right", Float64, queue_size=10)

    vel_left_pub = rospy.Publisher("vel_left", Float64, queue_size=10)
    vel_right_pub = rospy.Publisher("vel_right", Float64, queue_size=10)

    cur_pose = np.zeros(3, dtype=np.float64)  # x, y, hdg
    cur_vel = np.zeros(3, dtype=np.float64)  # same order as above

    last_enc_left = None
    last_enc_right = None

    t = rospy.Time.now()
    last_t = rospy.Time.now()
    r = rospy.Rate(10.0)

    wheelbase = 0.28  # meters
    wheel_circum = 2 * pi * .051 # meters
    encoder_conv_factor = 627.2  # ticks per revolution
    max_vel = 850  # ticks/sec

    enc_wrap_low = 0.3 * (2**16) - (2**15)
    enc_wrap_high = 0.7 * (2**16) - (2**15)

    def wait_for_start_byte(ser):
        start_t = rospy.Time.now()
        while True:
            try:
                start_byte = ser.read(1)
                if start_byte == b'\x55':
                    return True

                if (rospy.Time.now() - start_t).to_sec() >= 0.050:
                    return False
            except serial.serialutil.SerialException as e:
                rospy.logwarn("Caught serial exception: {}".format(str(e)))

    with serial.Serial('/dev/serial0', 9600, timeout=0.050) as ser:
        def set_motor_velocity(left_vel, right_vel):
            data = bytearray(struct.pack('<BBff', 0xAA, 0x03, left_vel, right_vel))
            checksum = 0
            for b in data:
                checksum ^= b

            data.append(checksum)

            rospy.loginfo_throttle(15, "Sending motor velocity request: {} / {}".format(
                left_vel, right_vel
            ))

            n = 0
            while n < 5:
                n += 1
                try:
                    ser.write(data)
                    ser.flush()
                    # wait for two start bytes as acknowledgement
                    if wait_for_start_byte(ser) and wait_for_start_byte(ser):
                        break
                except serial.serialutil.SerialException as e:
                    rospy.logwarn("Caught serial exception: {}".format(str(e)))

        def handle_motor_velocity_request(req):
            set_motor_velocity(req.left_vel, req.right_vel)
            return MotorVelResponse()

        def handle_motor_power_request(req):
            m1 = req.left_power
            m2 = req.right_power

            if m1 < 0:
                m1 = 256 + m1

            if m2 < 0:
                m2 = 256 + m2

            checksum = 0xAA ^ 0x01 ^ m1 ^ m2

            rospy.loginfo_throttle(15, "Sending motor power request: {} / {} ({:x} / {:x})".format(
                req.left_power, req.right_power, m1, m2
            ))

            n = 0
            while n < 5:
                n += 1
                try:
                    ser.write([0xAA, 0x01, m1, m2, checksum])
                    ser.flush()
                    # wait for two start bytes as acknowledgement
                    if wait_for_start_byte(ser) and wait_for_start_byte(ser):
                        break
                except serial.serialutil.SerialException as e:
                    rospy.logwarn("Caught serial exception: {}".format(str(e)))

            return MotorPowerResponse()

        mp_serv = rospy.Service('motor_power', MotorPower, handle_motor_power_request)
        mv_serv = rospy.Service('motor_vel', MotorVel, handle_motor_velocity_request)

        while not rospy.is_shutdown():
            try:
                t = rospy.Time.now()
                dt = (t - last_t).to_sec()

                ser.write([0xAA, 0x02, (0xAA ^ 0x02)])
                ser.flush()

                if wait_for_start_byte(ser):
                    # Response starting...
                    data = bytearray(ser.read(5))

                    if len(data) < 5:
                        # timed out waiting for response data
                        rospy.logwarn_throttle(10, "Timed out waiting for encoder repsonse data...")
                        continue

                    checksum = 0x55
                    for b in data:
                        checksum ^= b

                    if checksum != 0:
                        # got invalid checksum in response data
                        rospy.logwarn_throttle(10, "Got invalid checksum in encoder repsonse data...")
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

                    # handle wraparound:
                    d_left = 0
                    d_right = 0

                    if last_enc_left > enc_wrap_high and enc_left < enc_wrap_low:
                        # wrap around in positive direction
                        d_left = (0x7FFF - last_enc_left) + (0x7FFF + enc_left)
                    elif last_enc_left < enc_wrap_low and enc_left > enc_wrap_high:
                        # wrap around in negative direction
                        d_left = (0x7FFF + last_enc_left) + (0x7FFF - enc_left)
                    else:
                        d_left = enc_left - last_enc_left

                    if last_enc_right > enc_wrap_high and enc_right < enc_wrap_low:
                        d_right = (0x7FFF - last_enc_right) + (0x7FFF + enc_right)
                    elif last_enc_right < enc_wrap_low and enc_right > enc_wrap_high:
                        d_right = (0x7FFF + last_enc_right) + (0x7FFF - enc_right)
                    else:
                        d_right = enc_right - last_enc_right

                    # rotational velocity in ticks
                    rv_left = d_left / dt
                    rv_right = d_right / dt

                    last_enc_left = enc_left
                    last_enc_right = enc_right

                    # linear velocity (m/sec)
                    lv_left = rv_left * (wheel_circum / encoder_conv_factor)
                    lv_right = rv_right * (wheel_circum / encoder_conv_factor)

                    vel_rx = (lv_left + lv_right) / 2
                    vel_r_rot = (lv_right - lv_left) / wheelbase

                    cur_vel = np.array([
                        vel_rx * cos(cur_pose[2]),
                        vel_rx * sin(cur_pose[2]),
                        vel_r_rot
                    ])

                    cur_pose = cur_pose + (cur_vel * dt)

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
                    odom = Odometry()
                    odom.header.stamp = t
                    odom.header.frame_id = "odom"
                    odom.pose.pose = Pose(Point(cur_pose[0], cur_pose[1], 0), Quaternion(*odom_quat))

                    odom.child_frame_id = "base_link"
                    odom.twist.twist = Twist(
                        Vector3(cur_vel[0], cur_vel[1], 0),
                        Vector3(0, 0, cur_vel[2])
                    )

                    odom_pub.publish(odom)

                    # publish debug data
                    enc_left_pub.publish(enc_left)
                    dist_left_pub.publish(enc_left * (wheel_circum / encoder_conv_factor))
                    vel_left_pub.publish(lv_left)

                    enc_right_pub.publish(enc_right)
                    dist_right_pub.publish(enc_right * (wheel_circum / encoder_conv_factor))
                    vel_right_pub.publish(lv_right)

                    last_t = t
                    r.sleep()
                else:
                    rospy.logwarn_throttle(10, "Timed out waiting for encoder frame response")
            except serial.serialutil.SerialException as e:
                rospy.logwarn("Caught serial exception: {}".format(str(e)))

if __name__ == '__main__':
    main()
