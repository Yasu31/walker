#! /usr/bin/python3
'''
loads the .yaml file generated by lipm.py, then plays that back at an appropriate rate to /kondo_driver/command/joint_state
'''
import yaml
import rospy
from sensor_msgs.msg import JointState

f=open('output.yaml','r')
data=yaml.load(f)
f.close()
l_joint_names=["l_ankle_roll", "l_ankle_pitch", "l_knee", "l_hip_roll", "l_hip_pitch"]
r_joint_names=["r_ankle_roll", "r_ankle_pitch", "r_knee", "r_hip_roll", "r_hip_pitch"]

coefficients=[1,-1,-1,1,-1] # used to flip joint angles from left to right

pub=rospy.Publisher('/kondo_driver/command/joint_state', JointState, queue_size=10)
rospy.init_node('loadYAML', anonymous=True)
i=1
interval=data['interval']

rate=rospy.Rate(1.0/interval/1.2)
# rate=rospy.Rate(1.0)

while not rospy.is_shutdown():
    for i in range(1,len(data['joints_L'])):
        if rospy.is_shutdown():
            break
        js=JointState()
        js.name=l_joint_names+r_joint_names
        js.position=data['joints_L'][i][1:6]
        for j in range(5):
            js.position.append(data['joints_R'][i][j+1]*coefficients[j])
        js.header.stamp=rospy.get_rostime()
        pub.publish(js)
        print(js.position)
        print(js.name)
        rate.sleep()