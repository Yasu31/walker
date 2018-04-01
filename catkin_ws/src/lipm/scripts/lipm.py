'''
use the walker_env Conda environment. To do so,
$ export PATH="/home/yasu/anaconda3/bin:$PATH" (this enables Conda)
$ source activate walker_env
'''
import sys, os, time, yaml
ikpy_path=os.path.abspath("../../../../ikpy/src")
sys.path.append(ikpy_path)
import ikpy
import numpy as np

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

# generate a URDF from xacro beforehand with
# $ rosrun xacro xacro --inorder walker_left_leg.urdf.xacro > walker_left_leg.urdf
left_chain=ikpy.chain.Chain.from_urdf_file("../urdf/walker_left_leg.urdf",  base_elements=['lA'], last_link_vector=[-0.01,-0.025, 0.04], active_links_mask=[True]*7, name="left_chain")
right_chain=ikpy.chain.Chain.from_urdf_file("../urdf/walker_left_leg.urdf", base_elements=['lA'], last_link_vector=[-0.01,-0.025, 0.04], active_links_mask=[True]*7, name="right_chain")
# It's confusing I know, the right and left legs use the same kinematic chain because there's no reason to implement it twice when I can "flip" the results from one side.

print("left chain contains "+str(left_chain.links)+" links")
print("right chain contains "+str(right_chain.links)+" links")
# Unless the "last_link_vector" is specified when creating the chain from the URDF file, the final joint is set as the position of the final link, and therefore it does not move and does not contribute to the IK.
# See the wiki of ikpy to see how "links" are implemented in ikpy. The first and last links always do not move. Very confusing...(especially so when used together with URDFs)
# here, the last_link_vector is set so that the tip of the chain is at the pelvis of the robot.

height_com=0.14 #height of center of mass of robot.
omega=np.sqrt(9.8/0.height_com)
T=0.906 # cycle time. (one step lasts T/2)
start_pos=0.035
start_vel=0.28
N=20 #interpolate a step sequence into how many steps?
M=2# how many steps to take

vels_L=[np.array([0,0])]*(2*M)
pos_L=[np.array([0,0])]*(2*M)
vels_R=[np.array([0,0])]*(2*M)
pos_R=[np.array([0,0])]*(2*M)
vels_L[0]=np.array([0,start_vel])
pos_L[0]=np.array([0,-start_pos])
vels_R[0]=np.array([0,start_vel]) #never used
pos_R[0]=np.array([0,-start_pos])

joints_R=[]
joints_L=[]
joints_L.append([0.0]*len(left_chain.links))
joints_R.append([0.0]*len(right_chain.links))


def lipm_pos(omega, pos0, vel0, t):
    return pos0*np.cosh(omega*t)+vel0/omega*np.sinh(omega*t)
def lipm_vel(omega, pos0, vel0, t):
    return pos0*omega*np.sinh(omega*t)+vel0*np.cosh(omega*t)
def vel_L2R(vel_L):
    '''convert the velocity in the right foot's coordinate to that of the left foot.
    '''
    vel_R=vel_L
    return -vel_R
def vel_R2L(vel_R):
    '''convert the velocity in the left foot's coordinate to that of the right foot.
    '''
    vel_L=vel_R
    return -vel_R
def z_high(t):
    '''how high the pelvis should be in relation to the foot, when it is the support foot.
    '''
    return 0.18
def z_low(t):
    '''how high the pelvis should be in relation to the foot, when it is the swing foot.
    '''
    val=0.15
    if t<T/4:
        return (val-z_high(t))/(T/4)*t+z_high(t)
    else:
        return (z_high(t)-val)/(T/4)*(t-T/4)+val

i=0
while True:
    # calculate where(and how fast) the pelvis will end up at the end of this step, beforehand.
    vels_L[i+1]=lipm_vel(omega, pos_L[i], vels_L[i], T/2)
    pos_L[i+1]=lipm_pos(omega, pos_L[i], vels_L[i], T/2)
    vels_R[i+1]=vel_L2R(vels_L[i+1])
    pos_R[i+1]=np.array([0,-start_pos])
    print("calculating step #"+str(i)+"left foot is support foot, right foot is swing foot")
    for j in range(N):
        t=j*(T/2)/N
        print("calculating for time "+str(t))
        target_frame=np.eye(4)

        # calculate position of pelvis in relation to left foot, then calculate IK for left foot
        target_frame[:3,3]=[0.0, lipm_pos(omega, pos_L[i][1], vels_L[i][1], t), z_high(t)]
        print(target_frame[:3,3])
        ik_l=left_chain.inverse_kinematics(target_frame, initial_position=joints_L[len(joints_L)-1])
        joints_L.append(ik_l.tolist())

        # do the same for the right foot
        target_frame[:3,3]=[0.0, (pos_R[i+1][1]-pos_R[i][1])/(T/2)*t+pos_R[i][1], z_low(t)]
        print(target_frame[:3,3])
        ik_r=right_chain.inverse_kinematics(target_frame, initial_position=joints_R[len(joints_R)-1])
        joints_R.append(ik_r.tolist())

    vels_R[i+2]=lipm_vel(omega, pos_R[i+1], vels_R[i+1], T/2)
    pos_R[i+2]=lipm_pos(omega, pos_R[i+1], vels_R[i+1], T/2)
    vels_L[i+2]=vel_R2L(vels_R[i+2])
    pos_L[i+2]=np.array([0,-start_pos])
    print("calculating step #"+str(i+1)+"right foot is support foot, left foot is swing foot")
    for j in range(N):
        t=j*(T/2)/N
        print("calculating for time "+str(t))
        target_frame=np.eye(4)
        target_frame[:3,3]=[0.0, (pos_L[i+2][1]-pos_L[i+1][1])/(T/2)*t+pos_L[i+1][1], z_low(t)]
        print(target_frame[:3,3])
        ik_l=left_chain.inverse_kinematics(target_frame, initial_position=joints_L[len(joints_L)-1])
        joints_L.append(ik_l.tolist())
        target_frame[:3,3]=[0.0, lipm_pos(omega, pos_R[i+1][1], vels_R[i+1][1], t), z_high(t)]
        print(target_frame[:3,3])
        ik_r=right_chain.inverse_kinematics(target_frame, initial_position=joints_R[len(joints_R)-1])
        joints_R.append(ik_r.tolist())

    i+=2
    if i>=M-1:
        break

print("writing to YAML file...")
mylist={'joints_L': joints_L, 'joints_R': joints_R, 'interval': T/2/N}
f=open('output.yaml', 'w')
f.write(yaml.dump(mylist))
f.close()