#Chankyo Kim#
import numpy as np
import pybullet as p
import pybullet_data
import time

###### Connect to PyBullet physics server ######
p.connect(p.GUI) #or p.DIRECT
# p.connect(p.DIRECT)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
timestep = 1000
p.setTimeStep(1. / timestep)

###### Load Model ######
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])  #asset about ground plane. position and quaternion
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/model/urdf/digit_model.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/model/urdf/digit_model_closed.urdf",[0,0,10],[0,0,0,1], useFixedBase = False) 
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/model/urdf/digit-v3.urdf", [0,0,3],[0,0,0,1], useFixedBase = True) 
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/model/urdf/digit-v3/digit-v3-armfixed-zerofriction.urdf", [0,0,1],[0,0,0,1], useFixedBase = False) 
robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/model/urdf/digit-v3/digit-v3-armfixed.urdf", [0,0,0.93399],[0,0,0,1], useFixedBase = False) 

###### Setup for Joints #######
nJoints = p.getNumJoints(robot_id) 
print("nJoints: ", nJoints)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(robot_id, i);jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
left_hip_roll = jointNameToId['left_hip_roll'];left_hip_yaw = jointNameToId['left_hip_yaw'];left_hip_pitch = jointNameToId['left_hip_pitch'];left_knee = jointNameToId['left_knee'];left_shin = jointNameToId['left_shin'];left_tarsus = jointNameToId['left_tarsus'];left_toe_pitch = jointNameToId['left_toe_pitch'];left_toe_roll = jointNameToId['left_toe_roll'];left_heel_spring = jointNameToId['left_heel_spring'];left_toe_A = jointNameToId['left_toe_A'];left_toe_A_rod = jointNameToId['left_toe_A_rod'];left_A2 = jointNameToId['left_A2']
left_toe_B = jointNameToId['left_toe_B'];left_toe_B_rod = jointNameToId['left_toe_B_rod'];left_B2 = jointNameToId['left_B2'];left_achilles_rod = jointNameToId['left_achilles_rod'];left_ach2 = jointNameToId['left_ach2'];left_shoulder_roll = jointNameToId['left_shoulder_roll'];left_shoulder_pitch = jointNameToId['left_shoulder_pitch'];left_shoulder_yaw = jointNameToId['left_shoulder_yaw'];left_elbow = jointNameToId['left_elbow'];right_hip_roll = jointNameToId['right_hip_roll'];right_hip_yaw = jointNameToId['right_hip_yaw'];right_hip_pitch = jointNameToId['right_hip_pitch']
right_knee = jointNameToId['right_knee'];right_shin = jointNameToId['right_shin'];right_tarsus = jointNameToId['right_tarsus'];right_toe_pitch = jointNameToId['right_toe_pitch'];right_toe_roll = jointNameToId['right_toe_roll'];right_heel_spring = jointNameToId['right_heel_spring'];right_toe_A = jointNameToId['right_toe_A'];right_toe_A_rod = jointNameToId['right_toe_A_rod'];right_A2 = jointNameToId['right_A2'];right_toe_B = jointNameToId['right_toe_B'];right_toe_B_rod = jointNameToId['right_toe_B_rod'];right_B2 = jointNameToId['right_B2']
right_achilles_rod = jointNameToId['right_achilles_rod'];right_ach2 = jointNameToId['right_ach2'];right_shoulder_roll = jointNameToId['right_shoulder_roll'];right_shoulder_pitch = jointNameToId['right_shoulder_pitch'];right_shoulder_yaw = jointNameToId['right_shoulder_yaw'];right_elbow = jointNameToId['right_elbow']

# ###### Get Inertia Matrix #######
# MassMatrix = np.array(p.calculateMassMatrix(robot_id,[0 for i in range(12)]))
# print()
# print("Mass matrix")
# print(MassMatrix.shape)
# np.savetxt('MassMatrix.txt', MassMatrix, fmt='%.6f')
# for joint in range(nJoints):
#     print(p.getJointInfo(robot_id, joint))

    
###### Calculate Position of Closed Looped Anchors (Equation attached in Document: Refer to Chankyo Kim) ######
cid1= p.createConstraint(robot_id, left_heel_spring, robot_id, left_ach2, p.JOINT_POINT2POINT, [0, 0, 1], [0.113789, -0.011056, 0.], [0.260039048764136, -0.055149132747197,  0.001072511047511])
cid2= p.createConstraint(robot_id, left_toe_roll, robot_id, left_A2, p.JOINT_POINT2POINT, [0, 0, 1], [0.0179, -0.009551, -0.054164], [0.188700873535645, 0.031025632095949, 0.001961040077076])
cid3= p.createConstraint(robot_id, left_toe_roll, robot_id, left_B2, p.JOINT_POINT2POINT, [0, 0, 1], [-0.0181, -0.009551, -0.054164], [0.164841711315360, -0.030867345479882, -0.002471140022936])
cid4= p.createConstraint(robot_id, right_heel_spring, robot_id, right_ach2, p.JOINT_POINT2POINT, [0, 0, 1], [0.113789, 0.011056, 0], [0.260039048764136, 0.055149132747197,  0.001072511047511])
cid5= p.createConstraint(robot_id, right_toe_roll, robot_id, right_A2, p.JOINT_POINT2POINT, [0, 0, 1], [0.0179, 0.009551, -0.054164], [0.188700873535645, -0.031025632095949, 0.001961040077076])
cid6= p.createConstraint(robot_id, right_toe_roll, robot_id, right_B2, p.JOINT_POINT2POINT, [0, 0, 1], [-0.0181, 0.009551, -0.054164], [0.164841711315360, 0.030867345479882, -0.002471140022936])

###### Simulation ######
focus_position, _ = p.getBasePositionAndOrientation(robot_id)  #return position and orientation of robot_id(focus)
cdist = 3;cyaw=100;cpitch=-20;cubePos=focus_position
cnt = 0

# while(cnt < 10):
while(1):

    # p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=100, cameraPitch=-20, cameraTargetPosition = focus_position)
    p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)
    p.getMouseEvents()
    keys = p.getKeyboardEvents()
    #Keys to change camera
    if keys.get(100):  #D
        cyaw+=0.3
    if keys.get(97):   #A
        cyaw-=0.3
    if keys.get(99):   #C
        cpitch+=0.2
    if keys.get(102):  #F
        cpitch-=0.2
    if keys.get(122):  #Z
        cdist+=.01
    if keys.get(120):  #X
        cdist-=.01
        
    ####### Get Contact Data > #######
    contact = p.getContactPoints(robot_id)
    array_contact = np.array(contact)
    
    if(contact):
    # while( cnt < 4 ):
    #   if cnt > -1:
        print("contact")
        # print(array_contact)
        print(array_contact[:,3])
        print(array_contact[:,8])
        print()
        # print("Get collision shape of left/right foot")
        # print(p.getCollisionShapeData(robot_id,7))
        # print(p.getCollisionShapeData(robot_id,25))
        # print()
        
    else:
        print("no contact ")

    p.stepSimulation()
    # cnt = cnt + 1
    # time.sleep(.01)
    
    