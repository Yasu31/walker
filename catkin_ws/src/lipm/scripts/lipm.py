'''
use the walker_env Conda environment. To do so,
$ export PATH="/home/yasu/anaconda3/bin:$PATH" (this enables Conda)
$ source activate walker_env
'''
import sys, os, time, yaml
ikpy_path=os.path.abspath("../../../../ikpy/src")
sys.path.append(ikpy_path)
import numpy as np

import matplotlib.pyplot as plt

quick_vertical_motion=False
height_com=0.12 #height of center of mass of robot.
omega=np.sqrt(9.8/height_com)
N=20 #interpolate a step sequence into how many steps?
M=2# how many steps to take

# calculate the initial conditions that would ensure symmetrical movement(see 4/8 entry for details)
x0=-0.002
y0=-0.005
vx0=0.02
vy0=x0*y0*omega*omega/vx0
print("initial positions:\nx\t"+str(x0)+"\ty\t"+str(y0))
print("initial velocities:\nx\t"+str(vx0)+"\ty\t"+str(vy0))
T=2/omega*np.arcsinh(2*x0*vx0*omega/((x0*omega)**2-vx0**2))
print("cycle time\t"+str(T))

vels_L=[np.array([0,0])]*(2*M)
pos_L=[np.array([0,0])]*(2*M)
vels_R=[np.array([0,0])]*(2*M)
pos_R=[np.array([0,0])]*(2*M)
vels_L[0]=np.array([vx0,vy0])
pos_L[0]=np.array([x0,y0])
vels_R[0]=np.array([vx0,-vy0]) #never used
pos_R[0]=np.array([-x0,y0])

coords_R=np.zeros((N*M, 3))
coords_L=np.zeros((N*M, 3))



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
    val=0.16
    if quick_vertical_motion:
        # raise and lower feet quickly. Might cause jerky motion.
        if t<T/8:
            return (val-z_high(t))/(T/8)*t+z_high(t)
        elif t<T*3/8:
            return val
        else:
            return (z_high(t)-val)/(T/8)*(t-(T*3/8))+val
    else:
        # aise and lower feet slowly. Feet that's supposed to be in swing phase may touch the ground.
        if t<T/4:
            return (val-z_high(t))/(T/4)*t+z_high(t)
        else:
            return (z_high(t)-val)/(T/4)*(t-(T/4))+val


print("calculating coordinates of the pelvis relative to each foot...")
i=0
while True:
    # calculate where(and how fast) the pelvis will end up at the end of this step, beforehand.
    vels_L[i+1]=np.array([vx0,-vy0])#lipm_vel(omega, pos_L[i], vels_L[i], T/2)
    pos_L[i+1]=np.array([-x0,y0])#lipm_pos(omega, pos_L[i], vels_L[i], T/2)
    vels_R[i+1]=np.array([vx0,vy0])
    pos_R[i+1]=np.array([x0,y0])
    print("calculating step #"+str(i)+"; left foot is support foot, right foot is swing foot")
    for j in range(N):
        t=j*(T/2)/N
        # calculate position of pelvis in relation to left foot
        coords_L[N*i+j]=np.append(lipm_pos(omega, pos_L[i], vels_L[i], t), z_high(t))
    vels_R[i+2]=lipm_vel(omega, pos_R[i+1], vels_R[i+1], T/2)
    pos_R[i+2]=lipm_pos(omega, pos_R[i+1], vels_R[i+1], T/2)
    vels_L[i+2]=np.array([vx0, vy0])
    pos_L[i+2]=np.array([x0, y0])
    print("calculating step #"+str(i+1)+"; right foot is support foot, left foot is swing foot")
    for j in range(N):
        t=j*(T/2)/N
        coords_L[N*(i+1)+j]=np.append( (pos_L[i+2]-pos_L[i+1])/(T/2)*t+pos_L[i+1] , z_low(t))
    coords_R[i*N:(i+1)*N]=coords_L[(i+1)*N:(i+2)*N]
    coords_R[(i+1)*N:(i+2)*N]=coords_L[(i)*N:(i+1)*N]
    i+=2
    if i>=M-1:
        break

print("plotting...")
fig=plt.figure()
ax=fig.add_subplot(111)
ax.scatter(coords_L[0:N+1, 1], coords_L[0:N+1, 0], c='r', linewidth=2)
ax.scatter([0],[0],c='r', marker='s', linewidths=5)

# ax.scatter(-coords_R[N:2*N+1, 1]+y0+coords_L[N][1], coords_L[N:2*N+1, 0]-x0+coords_L[N][0], c='b', linewidth=2)
ax.scatter(-coords_R[N:2*N, 1]+ coords_L[N][1]+y0, coords_R[N:2*N, 0]+ coords_L[N][0]-x0, c='b', linewidth=2)
ax.scatter(coords_L[N][1]+y0, coords_L[N][0]-x0, c='b', marker='s', linewidths=5)

ax.set_xlim(0.01, 2*y0-0.01)
print("close the plot window to start calculating IK")
plt.show()

import ikpy

# generate a URDF from xacro beforehand with
# $ rosrun xacro xacro --inorder walker_left_leg.urdf.xacro > walker_left_leg.urdf
left_chain=ikpy.chain.Chain.from_urdf_file("../urdf/walker_left_leg.urdf",  base_elements=['lA'], last_link_vector=[0.0,-0.025, 0.04], active_links_mask=[True]*7, name="left_chain")
# It's confusing I know, the right and left legs use the same kinematic chain because there's no reason to implement it twice when I can "flip" the results from one side.

print("left chain contains "+str(left_chain.links)+" links")
# Unless the "last_link_vector" is specified when creating the chain from the URDF file, the final joint is set as the position of the final link, and therefore it does not move and does not contribute to the IK.
# See the wiki of ikpy to see how "links" are implemented in ikpy. The first and last links always do not move. Very confusing...(especially so when used together with URDFs)
# here, the last_link_vector is set so that the tip of the chain is at the pelvis of the robot.

joints_R=[]
joints_L=[]
joints_L.append([0.2]*len(left_chain.links))
joints_R.append([0.0]*len(left_chain.links))

print("calculating IK for left foot...")
for coord_L in coords_L:
    target_frame=np.eye(4)
    target_frame[:3,3]=coord_L
    ik_l=left_chain.inverse_kinematics(target_frame, initial_position=joints_L[len(joints_L)-1])
    joints_L.append(ik_l.tolist())
print("shuffling the IK of the left foot to get the IK for the right foot...")
i=0
while(i<M):
    joints_R+=(joints_L[N*(i+1)+1:N*(i+2)+1])
    joints_R+=(joints_L[N*(i)+1:N*(i+1)+1])
    i+=2

print("writing to YAML file...")
mylist={'joints_L': joints_L, 'joints_R': joints_R, 'interval': T/2/N}
f=open('output.yaml', 'w')
f.write(yaml.dump(mylist))
f.close()
