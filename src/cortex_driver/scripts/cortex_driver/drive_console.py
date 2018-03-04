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
        elif len(i) <= 2:
            mult = 1
            if len(i) == 2:
                if i[1] == 'f':
                    mult = 2
                else:
                    try:
                        mult = 1 + ((int(i[1]) - 5) / 10)
                    except ValueError as e:
                        # ignore invalid values of i[1]
                        mult = 1

            if i[0] == 'f' or i[0] == 'w':
                make_motor_call(300*mult, 300*mult)
            elif i[0] == 'b' or i[0] == 's':
                make_motor_call(-300*mult, -300*mult)
            elif i[0] == 'l' or i[0] == 'a':
                make_motor_call(300*mult, -300*mult)
            elif i[0] == 'r' or i[0] == 'd':
                make_motor_call(-300*mult, 300*mult)
        else:
            m = ptn.match(i)

            if m is not None:
                m1 = int(m.group(1))
                m2 = int(m.group(2))

                make_motor_call(m1, m2)

if __name__ == '__main__':
    main()
