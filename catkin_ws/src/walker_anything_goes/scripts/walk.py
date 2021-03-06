#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Thread


def thingy_upper_joint(normalized_time, rl=0):
    # first, sanitize the phase to a value between 0<=phase<2*pi
    normalized_time-=math.floor(normalized_time)
    # print("normalized_time is "+str(normalized_time))
    a=0.4
    b=-0.4
    e=0.9
    g=(7.0*a-2.0*b)/5.0
    if normalized_time < 0.5:
        return a + (b-a) * normalized_time / 0.5
    elif normalized_time<e:
        return b + (g-b) * (normalized_time - 0.5) / (e-0.5)
    else:
        return g + (a- g) * (normalized_time - e) / (1.0-e)

def upper_lower_joint(normalized_time, rl=0):
    # first, sanitize the phase to a value between 0<=phase<2*pi
    normalized_time-=math.floor(normalized_time)
    c=1.5
    d=0.13
    f=0.8
    h=0.7
    i=0.9
    if normalized_time < 0.25:
        return normalized_time*d/0.25
    elif normalized_time < 0.5:
        return d + (0-d) * (normalized_time-0.25) / 0.25
    elif normalized_time < h:
        tmp_phase=(normalized_time-0.5)*(2*math.pi)*(0.25/(h-0.5))
        return c * np.sin(tmp_phase)
    elif normalized_time < f:
        return c
    elif normalized_time<i:
        tmp_phase=(normalized_time-f)*(2*math.pi)*2*(0.25/(i-f))
        return c * (1 + np.cos(tmp_phase)) / 2
    else:
        return 0
def waist_thingy_joint(normalized_time, rl=0):
    normalized_time-=math.floor(normalized_time)
    j=[0.1,-0.1]
    if normalized_time<0.5 or 0.9<normalized_time:
        return j[rl]
    elif normalized_time<0.6:
        return j[rl]+(j[1-rl]-j[rl])*(normalized_time-0.5)/0.1
    elif normalized_time<0.8:
        return j[1-rl]
    elif normalized_time<0.9:
        return j[1-rl]+(j[rl]-j[1-rl])*(normalized_time-0.8)/0.1

def animate(i):
    print("oh hai mark")
    right_normalized_time=normalized_time-math.floor(normalized_time)
    left_normalized_time=(normalized_time+0.5)-math.floor(normalized_time+0.5)
    rtopdata.set_ydata([thingy_upper_joint(t) for t in time_axe])
    rtopnow.set_xdata(right_normalized_time)
    ltopdata.set_ydata([thingy_upper_joint(t) for t in time_axe])
    ltopnow.set_xdata(left_normalized_time)
    rmiddledata.set_ydata([upper_lower_joint(t) for t in time_axe])
    rmiddlenow.set_xdata(right_normalized_time)
    lmiddledata.set_ydata([upper_lower_joint(t) for t in time_axe])
    lmiddlenow.set_xdata(left_normalized_time)
    rbottomdata.set_ydata([waist_thingy_joint(t) for t in time_axe])
    rbottomnow.set_xdata(right_normalized_time)
    lbottomdata.set_ydata([waist_thingy_joint(t) for t in time_axe])
    lbottomnow.set_xdata(left_normalized_time)
    print("hey")
    # fig.canvas.show()



if __name__=='__main__':
    pub=rospy.Publisher("/kondo_driver/command/joint_state", JointState, queue_size=100)
    rospy.init_node('walk_publisher', anonymous=True)
    rate=rospy.Rate(30)
    normalized_time=0.0
    cycle=2.6
    time_axe=np.arange(0.0,1.0,0.01)
    while not rospy.is_shutdown():
        rospy.loginfo("normalized_time is "+str(normalized_time))
        normalized_time=rospy.get_time()/cycle
        joint_state=JointState()

        # all the joints must be published, because of the code for kondo_driver.

        joint_state.name.append("r_thingy_r_upper_joint")
        joint_state.position.append(thingy_upper_joint(normalized_time))
        joint_state.name.append("l_thingy_l_upper_joint")
        joint_state.position.append(thingy_upper_joint(normalized_time+0.5))

        joint_state.name.append("r_upper_r_lower_joint")
        joint_state.position.append(upper_lower_joint(normalized_time))

        joint_state.name.append("l_upper_l_lower_joint")
        joint_state.position.append(upper_lower_joint(normalized_time+0.5))

        joint_state.name.append("waist_r_thingy_joint")
        joint_state.position.append(waist_thingy_joint(normalized_time, 0))

        joint_state.name.append("waist_l_thingy_joint")
        joint_state.position.append(waist_thingy_joint(normalized_time+0.5, 1))

        pub.publish(joint_state)
        rate.sleep()
