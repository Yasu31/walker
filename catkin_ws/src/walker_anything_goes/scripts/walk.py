#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import math

def thingy_upper_joint(normalized_time):
    # first, sanitize the phase to a value between 0<=phase<2*pi
    normalized_time-=math.floor(normalized_time)
    print("normalized_time is "+str(normalized_time))
    a=0.4
    b=-0.4
    e=0.9
    g=2*a
    if normalized_time < 0.5:
        return a + (b-a) * normalized_time / 0.5
    elif normalized_time<e:
        return b + (g-b) * (normalized_time - 0.5) / (e-0.5)
    else:
        return g + (a- g) * (normalized_time - e) / (1.0-e)

def upper_lower_joint(normalized_time):
    # first, sanitize the phase to a value between 0<=phase<2*pi
    normalized_time-=math.floor(normalized_time)
    print("normalized_time is "+str(normalized_time))
    c=1.5
    d=0.13
    f=0.8
    h=0.7
    if normalized_time < 0.25:
        return normalized_time*d/0.25
    elif normalized_time < 0.5:
        return d + (0-d) * (normalized_time-0.25) / 0.25
    elif normalized_time < h:
        tmp_phase=(normalized_time-0.5)*(2*math.pi)*(0.25/(h-0.5))
        return c * math.sin(tmp_phase)
    elif normalized_time < f:
        return c
    else:
        tmp_phase=(normalized_time-f)*(2*math.pi)*2*(0.25/(1.0-f))
        return c * (1 + math.cos(tmp_phase)) / 2

if __name__=='__main__':
    pub=rospy.Publisher("/kondo_driver/command/joint_state", JointState, queue_size=100)
    rospy.init_node('walk_publisher', anonymous=True)
    rate=rospy.Rate(40)
    normalized_time=0.0
    cycle=0.7
    while not rospy.is_shutdown():
        normalized_time=rospy.get_time()/cycle
        joint_state=JointState()

        # all the joints must be published, because of the code for kondo_driver.

        joint_state.name.append("r_thingy_r_upper_joint")
        joint_state.position.append(thingy_upper_joint(normalized_time))

        joint_state.name.append("l_thingy_l_upper_joint")
        joint_state.position.append(thingy_upper_joint(normalized_time+0.5))
        # joint_state.position.append(0)

        joint_state.name.append("r_upper_r_lower_joint")
        joint_state.position.append(upper_lower_joint(normalized_time))

        joint_state.name.append("l_upper_l_lower_joint")
        joint_state.position.append(upper_lower_joint(normalized_time+0.5))
        # joint_state.position.append(0)

        joint_state.name.append("waist_r_thingy_joint")
        joint_state.position.append(0)

        joint_state.name.append("waist_l_thingy_joint")
        joint_state.position.append(0)

        pub.publish(joint_state)
        rate.sleep()
