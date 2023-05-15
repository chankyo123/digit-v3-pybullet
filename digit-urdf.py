#Chankyo Kim#
import numpy as np
import pybullet as p
import pybullet_data
import time

###### Connect to PyBullet physics server ######
p.connect(p.GUI) #or p.DIRECT
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

###### Load Model ######
# p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])  #asset about ground plane. position and quaternion
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model_closed.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 
robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model_closed.urdf",useFixedBase = False)
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/digit-v3_base.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/humanoid_torso.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 
# robot_id = p.loadURDF("quadruped/minitaur_v1.urdf")

for i in range(p.getNumJoints(robot_id)):
    print(p.getJointInfo(robot_id,i))    

# ###### Setup for Joints #######
nJoints = p.getNumJoints(robot_id)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(robot_id, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
print(jointNameToId)
print('num joint:',p.getNumJoints(robot_id))  

hip_abduction_left = jointNameToId['hip_abduction_left']
hip_rotation_left = jointNameToId['hip_rotation_left']
hip_flexion_left = jointNameToId['hip_flexion_left']
knee_joint_left = jointNameToId['knee_joint_left']
knee_to_shin_left = jointNameToId['knee_to_shin_left']
shin_to_tarsus_left = jointNameToId['shin_to_tarsus_left']
toe_pitch_joint_left = jointNameToId['toe_pitch_joint_left']
toe_roll_joint_left = jointNameToId['toe_roll_joint_left']
shoulder_roll_joint_left = jointNameToId['shoulder_roll_joint_left']
shoulder_pitch_joint_left = jointNameToId['shoulder_pitch_joint_left']
shoulder_yaw_joint_left = jointNameToId['shoulder_yaw_joint_left']
elbow_joint_left = jointNameToId['elbow_joint_left']
# shoulder_roll_cap_left = jointNameToId['shoulder_roll_cap_left']
hip_abduction_right = jointNameToId['hip_abduction_right']
hip_rotation_right = jointNameToId['hip_rotation_right']
hip_flexion_right = jointNameToId['hip_flexion_right']
knee_joint_right = jointNameToId['knee_joint_right']
knee_to_shin_right = jointNameToId['knee_to_shin_right']
shin_to_tarsus_right = jointNameToId['shin_to_tarsus_right']
toe_pitch_joint_right = jointNameToId['toe_pitch_joint_right']
toe_roll_joint_right = jointNameToId['toe_roll_joint_right']
shoulder_roll_joint_right = jointNameToId['shoulder_roll_joint_right']
shoulder_pitch_joint_right = jointNameToId['shoulder_pitch_joint_right']
shoulder_yaw_joint_right = jointNameToId['shoulder_yaw_joint_right']
elbow_joint_right = jointNameToId['elbow_joint_right']
# shoulder_cap_joint_right = jointNameToId['shoulder_cap_joint_right']
# waist_cap_joint_right = jointNameToId['waist_cap_joint_right']
# waist_cap_joint_left = jointNameToId['waist_cap_joint_left']
hip_pitch_achilles_rod_left = jointNameToId['hip_pitch_achilles_rod_left']
hip_pitch_achilles_rod_right = jointNameToId['hip_pitch_achilles_rod_right']
tarsus_heel_spring_left = jointNameToId['tarsus_heel_spring_left']
tarsus_heel_spring_right = jointNameToId['tarsus_heel_spring_right']
tarsus_toe_A_left = jointNameToId['tarsus_toe_A_left']
tarsus_toe_A_right = jointNameToId['tarsus_toe_A_right']
toe_A_toe_A_rod_left = jointNameToId['toe_A_toe_A_rod_left']
toe_A_toe_A_rod_right = jointNameToId['toe_A_toe_A_rod_right']
tarsus_toe_B_left = jointNameToId['tarsus_toe_B_left']
tarsus_toe_B_right = jointNameToId['tarsus_toe_B_right']
toe_B_toe_B_rod_left = jointNameToId['toe_B_toe_B_rod_left']
toe_B_toe_B_rod_right = jointNameToId['toe_B_toe_B_rod_right']


print()
print("joint info")
print("toe_roll_joint_left joint info : ", p.getJointInfo(robot_id,toe_roll_joint_left))
print("hip_pitch_achilles_rod_left joint info : ", p.getJointInfo(robot_id,hip_pitch_achilles_rod_left))
# print("jointfix_13_4 joint info : ", p.getJointInfo(robot_id,3))
print()
print("link info")
print("toe_roll_joint_left link state : ",p.getLinkState(robot_id,toe_roll_joint_left))
print("hip_pitch_achilles_rod_left link state : ",p.getLinkState(robot_id,hip_pitch_achilles_rod_left))

# Get the link state of the two joints
link_state_1 = p.getLinkState(robot_id, tarsus_heel_spring_left)
link_state_2 = p.getLinkState(robot_id, hip_pitch_achilles_rod_left)
# link_state_3 = p.getLinkState(robot_id, right_toe_roll)
# link_state_4 = p.getLinkState(robot_id, right_toe_A_rod)
# link_state_5 = p.getLinkState(robot_id, right_toe_roll)
# link_state_6 = p.getLinkState(robot_id, right_toe_B_rod)

# Extract the position vectors from the link states
pos1 = link_state_1[4]
pos2 = link_state_2[4]
# pos3 = link_state_3[0]
# pos4 = link_state_4[0]
# pos5 = link_state_5[0]
# pos6 = link_state_6[0]
# print("worldlink position - left_heel_spring : ", link_state_1[0])
# print("worldlink position - left_achilles_rod : ", link_state_2[0])
# print("linkworld position - left_heel_spring : ", link_state_1[4])
# print("linkworld position - left_achilles_rod : ", link_state_2[4])

# Compute the relative distance vector
rel_dist = [pos2[i] - pos1[i] for i in range(3)]
# print("anchor - pos1: ", np.subtract(np.array([0.113789, -0.011056, 0]),np.array(pos1))) 
# print("anchor - pos2: ", np.subtract(np.array([0.113789, -0.011056, 0]),np.array(pos2))) 
# print("anchor - pos3: ", np.subtract(np.array([0.0179, 0.009551, -0.054164]),np.array(pos3))) 
# print("anchor - pos4: ", np.subtract(np.array([0.0179, 0.009551, -0.054164]),np.array(pos4))) 
# print("anchor - pos5: ", np.subtract(np.array([-0.0181, 0.009551, -0.054164]),np.array(pos5))) 
# print("anchor - pos6: ", np.subtract(np.array([-0.0181, 0.009551, -0.054164]),np.array(pos6))) 

# ###### Initial condition ######
# jntIdx=[i for i in range(p.getNumJoints(robot_id))]
# initCond=[0.361490,0.000770, 0.286025,0.373352, 0, -0.346032,0.091354, -0.013601,-0.1506, 1.0922, 0.0017, -0.1391,0,-0.360407, -0.000561, -0.286076,-0.372723, 0, 0.347843,-0.092658, 0.019828,0.1506, -1.0922, -0.0017, 0.1391,0,0,0]
# for i in range(len(jntIdx)):
#     p.resetJointState(robot_id,jntIdx[i],initCond[i])
# # print(p.getNumJoints(robot_id))  #28 different joint





# ###### Provide Constraint #######
# print("joint state: ", p.getJointState(robot_id, tarsus_heel_spring_left))
# p.resetJointState(robot_id,tarsus_heel_spring_left,0.6)
# print("joint state: ", p.getJointState(robot_id, tarsus_heel_spring_left))
print()
print("print joint states : ")
for i in range(nJoints):
    jointState = p.getJointStateMultiDof(robot_id,i)
    jointType = p.getJointInfo(robot_id,i)[2]
    print("joint state pos [",i,"]",jointState[0])
    
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        p.resetJointStateMultiDof(robot_id, i, targetValue=[0.2], targetVelocity=[0])
    if (jointType == p.JOINT_SPHERICAL):
        p.resetJointStateMultiDof(robot_id, i, targetValue=[0.009485003066, -0.04756001538, -0.004475001447, 1], targetVelocity=[0,0,0])
print()
print("print joint states updated : ")
for i in range(nJoints):
    jointState = p.getJointStateMultiDof(robot_id,i)
    jointType = p.getJointInfo(robot_id,i)[2]
    print("joint state pos [",i,"]",jointState[0])
# jointStates = p.getJointStatesMultiDof(robot_id, [i for i in range(nJoints)])
# print(p.getJointStates(robot_id, [i for i in range(nJoints)]))


# print(p.getJointStates(robot_id, [i for i in range(nJoints)]))

# ###### Check Fixed Joint by fixedbase attribute #######
# print()
# print("Check Fixed Joint")
# for i in range(nJoints):
#     ji =  p.getJointInfo(robot_id,i)
#     jointType = ji[2]
#     if (jointType == p.JOINT_FIXED):
#         print(p.getJointInfo(robot_id,i)[1].decode('UTF-8'),end="/ ")
#         ->> knee_to_shin_left/ shoulder_roll_cap_left/ knee_to_shin_right/ shoulder_cap_joint_right/ waist_cap_joint_right/ waist_cap_joint_left

# ###### MassMatrix #######
print("nJoints: ")
print(nJoints)
MassMatrix = np.array(p.calculateMassMatrix(robot_id,[0 for i in range(nJoints)]))
print()
print(MassMatrix)
print(MassMatrix.shape)

# cid1= p.createConstraint(robot_id, tarsus_heel_spring_left, robot_id, hip_pitch_achilles_rod_left, p.JOINT_POINT2POINT, [0, 0, 1], [0.113789, -0.011056, 0], [0.23773861, -0.05066087, 0])
# cid2= p.createConstraint(robot_id, toe_roll_joint_left, robot_id, toe_A_toe_A_rod_left, p.JOINT_POINT2POINT, [0, 0, 1], [0.0179, -0.009551, -0.054164], [0.08, 0, 0.0])
# cid3= p.createConstraint(robot_id, toe_roll_joint_left, robot_id, toe_B_toe_B_rod_left, p.JOINT_POINT2POINT, [0, 0, 1], [-0.98839898, -0.131757, -1.91135701], [-0.74839899, -0.137757, -1.93095701])
# cid4= p.createConstraint(robot_id, tarsus_heel_spring_right, robot_id, hip_pitch_achilles_rod_right, p.JOINT_POINT2POINT, [0, 0, 1], [0.113789, 0.011056, 0], [0.23773861,  0.05066087, 0])
# cid5= p.createConstraint(robot_id, toe_roll_joint_right, robot_id, toe_A_toe_A_rod_right, p.JOINT_POINT2POINT, [0, 0, 1], [0.0179, 0.009551, -0.054164], [0.1, 0, 0.0])
# cid6= p.createConstraint(robot_id, toe_roll_joint_right, robot_id, toe_B_toe_B_rod_right, p.JOINT_POINT2POINT, [0, 0, 1], [-0.98839898, 0.132157, -1.91135701], [-0.74839899, 0.138157, -1.93095701])



###### getJointInfo ######
# for i in range(p.getNumJoints(robot_id)):
    # print(p.getJointInfo(robot_id,i)[12].decode(),end="/") 
    #left_hip_roll/left_hip_yaw/left_hip_pitch/left_knee/left_shin/left_tarsus/left_toe_pitch/left_toe_roll/left_shoulder_roll/left_shoulder_pitch/left_shoulder_yaw/left_elbow/left_shoulder_cap/right_hip_roll/right_hip_yaw/right_hip_pitch/right_knee/right_shin/right_tarsus/right_toe_pitch/right_toe_roll/right_shoulder_roll/right_shoulder_pitch/right_shoulder_yaw/right_elbow/right_shoulder_cap/left_waist_cap/right_waist_cap/

# print(p.getJointInfo(robot_id, 0)) ## left_hip_roll
# jtype = p.getJointInfo(robot_id, jointid)[2]
# jlower = p.getJointInfo(robot_id, jointid)[8]
# jupper = p.getJointInfo(robot_id, jointid)[9]
jlower, jupper = -1.0472, 1.0472
# print(jlower,jupper)




###### Change Joint Angles ######
# for step in range(10000):
#     joint_zero_targ = np.random.uniform(jlower,jupper)
#     joint_zero_targ = -1
#     # joint_four_targ = np.random.uniform(jlower,jupper)
#     p.setJointMotorControlArray(robot_id, [0], p.POSITION_CONTROL, targetPositions = [joint_zero_targ])
#     p.stepSimulation()
#     # if you need to query joint states or link states to update observation
#     # print(p.getLinkStates(robot_id, [2,4]))
#     if step ==100:
#         print(p.getJointStates(robot_id, [0,13]))   #getJointStates -> RETURN jointPosition, jointVelocity, [jointReactionForces], appliedJointMotorTorque


###### Simulation / Set Default Camera Position with zero action, lasting 5 secs ######

# for step in range(200):
focus_position, _ = p.getBasePositionAndOrientation(robot_id)  #return position and orientation of robot_id(focus)
cdist = 3;cyaw=100;cpitch=-20;cubePos=focus_position


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
        cyaw+=0.5
    if keys.get(97):   #A
        cyaw-=0.5
    if keys.get(99):   #C
        cpitch+=1
    if keys.get(102):  #F
        cpitch-=1
    if keys.get(122):  #Z
        cdist+=.01
    if keys.get(120):  #X
        cdist-=.01
    p.stepSimulation()
    
    
    
# class DigitEnv():
#     def __init__(self):
#         self.state = self.init_state()
#         self.step_count = 0
        
#     def init_state(self):
        
#         p.connect(p.GUI)
#         p.resetSimulation()
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         p.setGravity(0,0,-9.8)
#         self.pandaUid = p.loadURDF("franka_panda/panda.urdf",[0,0,0],[0,0,0,1], useFixedBase = True)
#         p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])
#         self.focus_pos, _ = p.getBasePositionAndOrientation(self.pandaUid)
#         p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-20, cameraPitch=-30, cameraTargetPosition = self.focus_pos)            
#         finger_pos = p.getLinkState(self.pandaUid, 9)[0]  #3 value of position of hand and 3dim space
#         obs = np.array([finger_pos]).flatten()
#         return obs
        
#     def reset(self):
#         p.disconnect()
#         self.state = self.init_state()
#         self.step_count = 0
        
#     def step(self, action):
#         self.step_count += 1
#         p.setJointMotorControlArray(self.pandaUid, [4], p.POSITION_CONTROL, [action]) #action is new target position
#         p.stepSimulation()
#         finger_pos = p.getLinkState(self.pandaUid, 9 )[0]
        
#         if (self.step_count >= 50):
#             self.reset()
#             finger_pos = p.getLinkState(self.pandaUid, 9)[0]
#             obs = np.array([finger_pos]).flatten()
#             self.state = obs
#             reward = -1
#             done = True
#             return reward, done
        
#         obs = np.array([finger_pos]).flatten()
#         self.state = obs
#         done = False
#         reward = -1 
#         return reward, done
            

# env = DigitEnv()
# for step in range(500):
#     action = np.random.uniform(jlower,jupper)
#     a, b = env.step(action)
#     print(env.state)