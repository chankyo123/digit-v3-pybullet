import pybullet as p
import pybullet_data
import numpy as np
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/humanoid_torso.urdf", [0, 0, 2], [0, 0, 0, 1], useFixedBase=False)
# robot_id = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 2], [0.7071, 0, 0, 0.7071], useFixedBase=False, globalScaling=0.4)
robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model_closed.urdf",[0,0,2],[0,0,0,1],useFixedBase = False)

# return position and orientation of robot_id(focus)
focus_position, _ = p.getBasePositionAndOrientation(robot_id)
cdist = 3;cyaw = 100;cpitch = -20;cubePos = focus_position

numJoints = p.getNumJoints(robot_id)

p.resetDebugVisualizerCamera(
    cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)

cnt = 0
for step in range(10000):
    p.stepSimulation()
    time.sleep(1. / 240.)
    contact = p.getContactPoints(robot_id)
    array_contact = np.array(contact)
    if(contact):
        if cnt < 4:
        #   if cnt > -1:
            # print(array_contact)
            print(array_contact[:,8])
            cnt = cnt + 1
        else:
            break
    
for joint in range(numJoints):
    print(p.getJointInfo(robot_id, joint))

p.disconnect()
