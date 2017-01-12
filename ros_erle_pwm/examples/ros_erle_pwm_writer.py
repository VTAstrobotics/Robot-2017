#!/usr/bin/python
import rospy
import sys
from ros_erle_pwm.msg import *

PWM = 0
index = ""
def talker():
    rospy.init_node("pwm_writer", anonymous=True)
    pub = rospy.Publisher("PWM_"+index, pwm, queue_size=10)
    msg = pwm();
    msg.PWM = PWM
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

def usage():
    return "%s [index PWM]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) == 3:
        PWM = int(sys.argv[2])
        index = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    talker();
