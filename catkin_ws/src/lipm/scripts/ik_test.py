#! /usr/bin/python3
'''
use the walker_env Conda environment. To do so,
$ export PATH="/home/yasu/anaconda3/bin:$PATH" (this enables Conda)
$ source activate walker_env
'''
import sys, os
ikpy_path=os.path.abspath("../../../../ikpy/src")
sys.path.append(ikpy_path)
print(ikpy_path)
import ikpy
import numpy as np
from ikpy import plot_utils

my_chain=ikpy.chain.Chain.from_urdf_file("../../walker_description/urdf/walker.urdf", base_elements=["lA"])
# generate a URDF from xacro beforehand with
# $ rosrun xacro xacro --inorder walker.urdf.xacro > walker.urdf
target_vector = [ 0.0, 0.0, 0.0]
target_frame = np.eye(4)
target_frame[:3, 3] = target_vector
print("number of links is"+str(len(my_chain.links)))
print("The angles of each joints are : ", my_chain.inverse_kinematics(target_frame))
real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_frame))
print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))
# %matplotlib inline
import matplotlib.pyplot as plt
ax = plot_utils.init_3d_figure()
my_chain.plot(my_chain.inverse_kinematics(target_frame), ax, target=target_vector)
plt.xlim(-0.1, 0.1)
plt.ylim(-0.1, 0.1)
