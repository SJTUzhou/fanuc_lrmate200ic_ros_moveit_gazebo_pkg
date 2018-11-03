#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def callback_remap(data):
    joint_states = rospy.Publisher(name="/joint_states/", data_class=JointState, queue_size=10)
    if not rospy.is_shutdown():
        joint_states.publish(data)

def main():
    rospy.init_node("joint_states_remapper", anonymous=True)
    rospy.Subscriber(name="/lrmate200ic/joint_states", data_class=JointState, callback=callback_remap)
    rospy.spin()

if __name__ == "__main__":
    main()
