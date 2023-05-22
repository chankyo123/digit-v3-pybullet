#Chankyo Kim#
import numpy as np
import pybullet as p
import pybullet_data
import time

###### Connect to PyBullet physics server ######
p.connect(p.GUI) #or p.DIRECT
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

###### Load Model ######
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])  #asset about ground plane. position and quaternion
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model_closed.urdf",[0,0,10],[0,0,0,1], useFixedBase = False) 
robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit-v3.urdf", [0,0,3],[0,0,0,1], useFixedBase = True) 

nJoints = p.getNumJoints(robot_id)
print("nJoints: ", nJoints)
MassMatrix = np.array(p.calculateMassMatrix(robot_id,[0 for i in range(12)]))
print()
print("Mass matrix")
print(MassMatrix.shape)
np.savetxt('MassMatrix.txt', MassMatrix, fmt='%.6f')
for joint in range(nJoints):
    print(p.getJointInfo(robot_id, joint))

###### Simulation ######
focus_position, _ = p.getBasePositionAndOrientation(robot_id)  #return position and orientation of robot_id(focus)
cdist = 3;cyaw=-100;cpitch=-20;cubePos=focus_position
while(1):
# for step in range(200):
    # joint_zero_targ = np.random.uniform(jlower,jupper)
    # joint_zero_targ = -1
    # p.setJointMotorControlArray(robot_id, [0], p.POSITION_CONTROL, targetPositions = [joint_zero_targ])
    p.stepSimulation()
    # p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=100, cameraPitch=-20, cameraTargetPosition = focus_position)
    # p.stepSimulation()
    # time.sleep(.01)
    
    p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)
    p.getMouseEvents()
    keys = p.getKeyboardEvents()
    #Keys to change camera
    if keys.get(100):  #D
        cyaw+=0.1
    if keys.get(97):   #A
        cyaw-=0.1
    if keys.get(99):   #C
        cpitch+=0.2
    if keys.get(102):  #F
        cpitch-=0.2
    if keys.get(122):  #Z
        cdist+=.01
    if keys.get(120):  #X
        cdist-=.01
    p.stepSimulation()
    
    