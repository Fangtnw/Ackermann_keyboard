#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

# Initialize ROS node
rospy.init_node('keyboard_teleop')


# Create publisher for Twist messages
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Set terminal settings
settings = termios.tcgetattr(sys.stdin)

# Initialize rate object
rate = rospy.Rate(10) # 10 Hz

drive_spd = 30
drive_state = 0
servo_position = 90
servo_increment = 10
spd_increment = 5

try:
    tty.setcbreak(sys.stdin.fileno())

    print("Use w/a/s/d to move the robot. l/; to adjust spd Press 'q' to exit.")

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key = sys.stdin.read(1)
            if key == '\x1b': # Arrow keys havle 3 bytes of input
                key = sys.stdin.read(2)

            if key == 'q':
                break

            # Map keys to velocity commands
            if key == 'w': #'\x1b[A': # Up arrow key
                drive_state = 1
            elif key == 's': #'\x1b[B': # Down arrow keys
                drive_state = -1
            elif key == 'd' : #'\x1b[C': # Right arrow key
                #servo_position = 160
                 servo_position += servo_increment
            elif key == 'a' : #'\x1b[D': # Left arrow key
                #servo_position = 20
                servo_position -= servo_increment
            elif key == 'l' : #'\x1b[C': # Right arrow key
                #servo_position = 160
                 drive_spd += spd_increment
            elif key == ';' : #'\x1b[D': # Left arrow key
                #servo_position = 20
                drive_spd -= spd_increment
            elif key == 'o' : #'\x1b[D': # Left arrow key
                drive_state = 0
            elif key == 'p' : #'\x1b[D': # Left arrow key
                servo_position = 90    
            else:
                drive_state = 0
                
            servo_position = max(0, min(180, servo_position))
            # Create Twist message
            twist_msg = Twist()
            twist_msg.linear.x = drive_state
            twist_msg.linear.y = drive_spd
            #twist_msg.linear.y = right_speed
            twist_msg.angular.z = servo_position

            # Publish Twist message
            pub.publish(twist_msg)
        ###else:
            # No key pressed, stop the robot motion
            #drive_spd = 0
            #servo_position = 90
            #twist_msg = Twist()
            #twist_msg.linear.x = drive_spd
            #twist_msg.angular.z = servo_position
            #pub.publish(twist_msg)
        # # Sleep to maintain desired rate
        rate.sleep()

except Exception as e:
    print(str(e))

finally:
    # Stop robot motion
    twist_msg = Twist()
    twist_msg.linear.x = 0
    #twist_msg.linear.y = 0
    twist_msg.angular.z = 90
    pub.publish(twist_msg)

    # Reset terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
