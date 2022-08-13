#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import sys
import select
import termios
import tty
import rospy

msg = """
Reading from the keyboard  
---------------------------
Moving around:
   q    w    e
   a    s    d

HIT SPACE TO BRAKE

q/z : increase/decrease max speeds by 1 unit

anything else : stop

CTRL-C to quit
"""

moveBindings = {
    #(Front, left, Rear, Right)
    'w': (0, -1, 0, 1),
    's': (0, 1, 0, -1),
    'd': (-1, 0, 1, 0),
    'a': (1, 0, -1, 0),
    ' ': (0, 0, 0, 0),
}

speedBindings = {
    'q': (1, 1),
    'z': (-1, -1),
}

robots = ["empty", "/redbot", "/greenbot", "/bluebot", "/greybot"]

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('swarm_teleop')

    robot_no = int(input("Choose Robot: 1. Redbot 2. Greenbot 3. Bluebot 4. Greybot"))

    robot_name = robots[robot_no]
    
    pub_front = rospy.Publisher(
        '{}/front/command'.format(robot_name), Float64, queue_size=1)
    pub_left = rospy.Publisher(
        '{}/left/command'.format(robot_name), Float64, queue_size=1)
    pub_rear = rospy.Publisher(
        '{}/rear/command'.format(robot_name), Float64, queue_size=1)
    pub_right = rospy.Publisher(
        '{}/right/command'.format(robot_name), Float64, queue_size=1)

    x = 0
    y = 0
    z = 0
    speed = 10.0
    print(msg)

    while(1):
        key = getKey()
        if key in moveBindings.keys():
            front = moveBindings[key][0]
            left = moveBindings[key][1]
            rear = moveBindings[key][2]
            right = moveBindings[key][3]

        elif key in speedBindings.keys():
            speed += speedBindings[key][0]
            print(speed)

        else:
            front = 0
            left = 0
            rear = 0
            right = 0
            if (key == '\x03'):
                break

        front_vel = Float64()
        left_vel = Float64()
        rear_vel = Float64()
        right_vel = Float64()

        front_vel = front * speed
        left_vel = left * speed
        rear_vel = rear * speed
        right_vel = right * speed

        pub_front.publish(front_vel)
        pub_left.publish(left_vel)
        pub_rear.publish(rear_vel)
        pub_right.publish(right_vel)
