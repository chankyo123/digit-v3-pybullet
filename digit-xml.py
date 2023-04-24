#Chankyo Kim#
import pybullet as p
import time
import pybullet_data
import numpy as np

###### Connect to PyBullet physics server ######
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #/Users/ckkim/.local/lib/python3.10/site-packages/pybullet_data

p.setGravity(0,0,-9.8)
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])  #asset about ground plane. position and quaternion
print(pybullet_data.getDataPath())

###### Load MJCF model #######
# mjcf_file = "example.xml"
# mjcf_file = "digit-v3-armfixed-springFixed.xml"
# mjcf_file = "xml/humanoid_symmetric.xml"
# mjcf_file = "xml/humanoid.xml"
# mjcf_file = "xml/digit-v3-armfixed-springfixed-bullet.xml"
# mjcf_file = "xml/diamond.xml"
# mjcf_file = "xml/hinge-digit-v3-basepinned-armfixed.xml"
# mjcf_file = "xml/hinge-digit-v3-basepinned-armfixed.xml"
mjcf_file = "xml/digit-v3-basepinned-armfixed.xml"
robot_id = p.loadMJCF(mjcf_file)[0]

###### Check Base Position and Orientation #######
# basePos, baseOrn = p.getBasePositionAndOrientation(robot_id)
# print("base info")
# print("base position : ",basePos)
# print("base orientation : ",baseOrn)

###### Setup for Joints #######
nJoints = p.getNumJoints(robot_id)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(robot_id, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
print(jointNameToId)
print('num joint:',p.getNumJoints(robot_id))  


left_hip_roll = jointNameToId['left-hip-roll']
left_hip_yaw = jointNameToId['left-hip-yaw']
left_hip_pitch = jointNameToId['left-hip-pitch']
left_achilles_rod = jointNameToId['left-achilles-rod']
left_knee = jointNameToId['left-knee']
left_shin = jointNameToId['left-shin']
left_tarsus = jointNameToId['left-tarsus']
left_heel_spring = jointNameToId['left-heel-spring']
left_toe_A = jointNameToId['left-toe-A']
left_toe_A_rod = jointNameToId['left-toe-A-rod']    
left_toe_B = jointNameToId['left-toe-B']
left_toe_B_rod = jointNameToId['left-toe-B-rod']    
left_toe_pitch = jointNameToId['left-toe-pitch']
left_toe_roll = jointNameToId['left-toe-roll']
left_shoulder_roll = jointNameToId['left-shoulder-roll']
left_shoulder_pitch = jointNameToId['left-shoulder-pitch']
left_shoulder_yaw = jointNameToId['left-shoulder-yaw']
left_elbow = jointNameToId['left-elbow']
right_hip_roll = jointNameToId['right-hip-roll']
right_hip_yaw = jointNameToId['right-hip-yaw']
right_hip_pitch = jointNameToId['right-hip-pitch']
right_achilles_rod = jointNameToId['right-achilles-rod']
right_knee = jointNameToId['right-knee']
right_shin = jointNameToId['right-shin']
right_tarsus = jointNameToId['right-tarsus']
right_heel_spring = jointNameToId['right-heel-spring']
right_toe_A = jointNameToId['right-toe-A']
right_toe_A_rod = jointNameToId['right-toe-A-rod']
right_toe_B = jointNameToId['right-toe-B']
right_toe_B_rod = jointNameToId['right-toe-B-rod']
right_toe_pitch = jointNameToId['right-toe-pitch']
right_toe_roll = jointNameToId['right-toe-roll']
right_shoulder_roll = jointNameToId['right-shoulder-roll']
right_shoulder_pitch = jointNameToId['right-shoulder-pitch']
right_shoulder_yaw = jointNameToId['right-shoulder-yaw']
right_elbow = jointNameToId['right-elbow']

print()
print("joint info")
print("left-hip-roll joint info : ", p.getJointInfo(robot_id,0))
print("jointfix_14_2 joint info : ", p.getJointInfo(robot_id,1))
print("jointfix_13_4 joint info : ", p.getJointInfo(robot_id,3))
print()
print("link info")
print("left-hip-roll link state : ",p.getLinkState(robot_id,0))
print("jointfix_14_2 link state : ",p.getLinkState(robot_id,1))

# Get the link state of the two joints
link_state_1 = p.getLinkState(robot_id, right_heel_spring)
link_state_2 = p.getLinkState(robot_id, right_achilles_rod)
link_state_3 = p.getLinkState(robot_id, right_toe_roll)
link_state_4 = p.getLinkState(robot_id, right_toe_A_rod)
link_state_5 = p.getLinkState(robot_id, right_toe_roll)
link_state_6 = p.getLinkState(robot_id, right_toe_B_rod)

# Extract the position vectors from the link states
pos1 = link_state_1[0]
pos2 = link_state_2[0]
pos3 = link_state_3[0]
pos4 = link_state_4[0]
pos5 = link_state_5[0]
pos6 = link_state_6[0]
# print("worldlink orientation - left_heel_spring : ", link_state_1[1])
# print("worldlink orientation - left_achilles_rod : ", link_state_2[1])
# print("linkworld orientation - left_heel_spring : ", link_state_1[5])
# print("linkworld orientation - left_achilles_rod : ", link_state_2[5])

# Compute the relative distance vector
rel_dist = [pos2[i] - pos1[i] for i in range(3)]
# print("anchor - pos1: ", np.subtract(np.array([0.113789, 0.011056, 0]),np.array(pos1))) 
# print("anchor - pos2: ", np.subtract(np.array([0.113789, 0.011056, 0]),np.array(pos2))) 
# print("anchor - pos3: ", np.subtract(np.array([0.0179, 0.009551, -0.054164]),np.array(pos3))) 
# print("anchor - pos4: ", np.subtract(np.array([0.0179, 0.009551, -0.054164]),np.array(pos4))) 
# print("anchor - pos5: ", np.subtract(np.array([-0.0181, 0.009551, -0.054164]),np.array(pos5))) 
# print("anchor - pos6: ", np.subtract(np.array([-0.0181, 0.009551, -0.054164]),np.array(pos6))) 
# cid = p.createConstraint(robot_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, -1], [0, 0, 1])  # Fix Base In The Air
cid1= p.createConstraint(robot_id, left_heel_spring, robot_id, left_achilles_rod, p.JOINT_POINT2POINT, [0, 0, 1], [-0.43084999, -0.143806, -1.85823301], [ 0.166926, -0.105856, -1.89869301])
cid2= p.createConstraint(robot_id, left_toe_roll, robot_id, left_toe_A_rod, p.JOINT_POINT2POINT, [0, 0, 1], [-0.95239898, -0.131757, -1.91135701], [-0.66039899, -0.137757, -1.87575701])
cid3= p.createConstraint(robot_id, left_toe_roll, robot_id, left_toe_B_rod, p.JOINT_POINT2POINT, [0, 0, 1], [-0.98839898, -0.131757, -1.91135701], [-0.74839899, -0.137757, -1.93095701])
cid4= p.createConstraint(robot_id, right_heel_spring, robot_id, right_achilles_rod, p.JOINT_POINT2POINT, [0, 0, 1], [-0.43084999, 0.144206, -1.85823301], [0.166926, 0.106256, -1.89869301])
cid5= p.createConstraint(robot_id, right_toe_roll, robot_id, right_toe_A_rod, p.JOINT_POINT2POINT, [0, 0, 1], [-0.95239898, 0.132157, -1.91135701], [-0.66039899, 0.138157, -1.87575701])
cid6= p.createConstraint(robot_id, right_toe_roll, robot_id, right_toe_B_rod, p.JOINT_POINT2POINT, [0, 0, 1], [-0.98839898, 0.132157, -1.91135701], [-0.74839899, 0.138157, -1.93095701])

print("xml file:", mjcf_file)

print('num joint:',p.getNumJoints(robot_id))  #12 different joint
print('num constraint:',p.getNumConstraints())
focus_position, _ = p.getBasePositionAndOrientation(robot_id)  #return position and orientation of targid(focus)
cdist = 3;cyaw=100;cpitch=-20;cubePos=focus_position


###### Initial condition ######
jntIdx=[i for i in range(p.getNumJoints(robot_id))]
# initCond=[0.,0.000770, 0.286025,0.373352, 0, -0.346032,0.091354, -0.013601,-0.1506, 1.0922, 0.0017, -0.1391,0,-0.360407, -0.000561, -0.286076,-0.372723, 0, 0.347843,-0.092658, 0.019828,0.1506, -1.0922, -0.0017, 0.1391,0,0,0]
# # print("init cond:", len(initCond)) #28
for i in range(len(jntIdx)):
    # p.resetJointState(robot_id,jntIdx[i],initCond[i])
    p.resetJointState(robot_id,jntIdx[i],0.0)
# p.resetJointState(robot_id,left_hip_roll,0.361490)
# p.resetJointState(robot_id,left_hip_yaw,0.000770)
# p.resetJointState(robot_id,left_hip_pitch,0.286025)
# p.resetJointState(robot_id,left_achilles_rod,1)
# p.resetJointState(robot_id,left_knee,0.373352)
# p.resetJointState(robot_id,left_shin,0)
# p.resetJointState(robot_id,left_tarsus,-0.346032)
# p.resetJointState(robot_id,left_heel_spring,-0.009752)
# p.resetJointState(robot_id,left_toe_A,-0.092650)
# p.resetJointState(robot_id,left_toe_A_rod,1)
# p.resetJointState(robot_id,left_toe_B,0.084274)
# p.resetJointState(robot_id,left_toe_B_rod,1)
# p.resetJointState(robot_id,left_toe_pitch,0.091354)
# p.resetJointState(robot_id,left_toe_roll,-0.013601)
# p.resetJointState(robot_id,left_shoulder_roll,-0.1506)
# p.resetJointState(robot_id,left_shoulder_pitch,1.0922)
# p.resetJointState(robot_id,left_shoulder_yaw,0.0017)
# p.resetJointState(robot_id,left_elbow,-0.1391)


# p.resetJointState(robot_id,right_hip_roll,-0.360407)
# p.resetJointState(robot_id,right_hip_yaw, -0.000561)
# p.resetJointState(robot_id,right_hip_pitch,-0.286076)
# p.resetJointState(robot_id,right_achilles_rod,1)
# p.resetJointState(robot_id,right_knee,-0.372723)
# p.resetJointState(robot_id,right_shin,0)
# p.resetJointState(robot_id,right_tarsus,0.347843)
# p.resetJointState(robot_id,right_heel_spring,0.008955)
# p.resetJointState(robot_id,right_toe_A,0.095860)
# p.resetJointState(robot_id,right_toe_A_rod,1)    #-1
# p.resetJointState(robot_id,right_toe_B,-0.083562)
# p.resetJointState(robot_id,right_toe_B_rod,1)    #-1
# p.resetJointState(robot_id,right_toe_pitch,-0.092658)
# p.resetJointState(robot_id,right_toe_roll,0.019828)
# p.resetJointState(robot_id,right_shoulder_roll,0.1506)
# p.resetJointState(robot_id,right_shoulder_pitch, -1.0922)
# p.resetJointState(robot_id,right_shoulder_yaw,-0.0017)
# p.resetJointState(robot_id,right_elbow,0.1391)


###### Simulation ######
for i in range(10000):
    p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)
    # p.setJointMotorControlArray(robot_id, [left_tarsus], p.POSITION_CONTROL, targetPositions = [-0.346032])
    
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
    # time.sleep(.01)
    
    # print(p.getLinkStates(robot_id, [left_knee]))
    # print(p.getJointStates(robot_id, [left_hip_yaw]))
    

# Disconnect from the physics server
p.disconnect()