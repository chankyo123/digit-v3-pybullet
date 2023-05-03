import pybullet as p
import numpy as np
# print("Matrix : ")
# print(np.array(p.getMatrixFromQuaternion([0,0,0,1])).reshape(3,3))

#### Calculate anchor point from child frame ####
rot_world_parent = np.array(p.getMatrixFromQuaternion([0.4304454513997003, 0.1948882644766686, 0.6797194747580058, 0.5609961794640683])).reshape(3,3).T
rot_world_child = np.array(p.getMatrixFromQuaternion([0.39685905856538267, 0.7332376185308868, -0.513741206979532, 0.20232512117126158])).reshape(3,3).T

print("X axis child : ")
print(np.matmul(rot_world_child,np.array([1,0,0])[:,np.newaxis]))

print("Matrix : ")
print(rot_world_parent)

d1 = np.array([[0.0179, -0.009551, -0.054164]]).T   #anchor point
d01_world = np.array((0.060050850267780814, 0.4159408911984945, -0.8262503561542451))[:,np.newaxis]+np.matmul(rot_world_parent.T,d1)
# print("check : ",np.matmul(rot_world_parent,d1))
print("d01_world : ")
print(d01_world.flatten())
d2_world = d01_world - np.array((-0.196145597409118, 0.1891019282811404, -0.3021878367723899))[:,np.newaxis]
d2_child = np.matmul(rot_world_child,d2_world)
print("p2_child : ")
print(d2_child.flatten())