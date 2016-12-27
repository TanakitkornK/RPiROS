#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import time
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('echoSineWave', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        amp = math.sin(time.time())
        hello_str = "hello world %s" % amp
        rospy.loginfo(hello_str)
        pub.publish(str(amp))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
