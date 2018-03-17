#!/usr/bin/env python
'''
enables the robot to move as a mannequin- each joint can be moved with a bit of force, and it will keep that position.
'''
import rospy
from sensor_msgs.msg import JointState
from time import sleep

def make_gain_cmd(gain):
    gain_cmd = JointState()
    joint_name =\
        ["r_ankle_pitch",
      "l_ankle_yaw",
      "r_knee",
      "l_knee",
      "r_hip_pitch",
      "l_hip_pitch",
      "r_hip_yaw",
      "l_hip_yaw",
      "l_ankle_pitch",
      "r_ankle_yaw"]
    for i in range(len(joint_name)):
        gain_cmd.name.append(joint_name[i])
        gain_cmd.effort.append(gain)
    return gain_cmd


def cb(msg):
    global pub
    js_cmd = JointState()
    js_cmd.header = msg.header
    for i in range(len(msg.name)):
        js_cmd.name.append(msg.name[i])
        js_cmd.position.append(msg.position[i])
    pub.publish(js_cmd)

if __name__ == '__main__':
    rospy.init_node('mannequin')
    pub = rospy.Publisher("/kondo_driver/command/joint_state", JointState)
    gain_pub = rospy.Publisher("/kondo_driver/command/gain", JointState)
    sub = rospy.Subscriber("/kondo_driver/joint_state", JointState, cb)

    sleep(3)
    gain_pub.publish(make_gain_cmd(0x01))

rospy.spin()
