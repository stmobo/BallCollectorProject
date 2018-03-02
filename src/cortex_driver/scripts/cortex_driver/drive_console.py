#!/usr/bin/env python
import rospy
import re
from cortex_driver.srv import *

ptn = re.compile(r'\s*(\-?\d+)\s*[/|,]\s*(\-?\d+)')

def make_motor_call(m1, m2):
    rospy.wait_for_service('motor_vel')
    try:
        mp_prox = rospy.ServiceProxy('motor_vel', MotorVel)
        mp_prox(m1, m2)
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(str(e)))

def main():
    while True:
        i = raw_input("m> ")
        if i == 'e' or i == 'exit':
            break
        elif i == '':
            make_motor_call(0, 0)
        elif i == 'f' or i == 'w':
            make_motor_call(300, 300)
        elif i == 'b' or i == 's':
            make_motor_call(-300, -300)
        elif i == 'l' or i == 'a':
            make_motor_call(300, -300)
        elif i == 'r' or i == 'd':
            make_motor_call(-300, 300)
        else:
            m = ptn.match(i)

            m1 = int(m.group(1))
            m2 = int(m.group(2))

            make_motor_call(m1, m2)

if __name__ == '__main__':
    main()
