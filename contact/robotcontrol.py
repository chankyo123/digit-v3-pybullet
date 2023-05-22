import pybullet as p
import pybullet_data
import numpy as np
import time
import csv
import os
p.connect(p.GUI)
# p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf",[0,0,0])
p.setGravity(0, 0, -9.8)
timestep = 1000
p.setTimeStep(1. / timestep)
contacterp = 10
contactbreakingthreshold=0.02
contactprocessingthreshold = 0.005
p.setDefaultContactERP(contacterp)
p.setPhysicsEngineParameter(contactBreakingThreshold=contactbreakingthreshold)
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/humanoid_torso.urdf", [0, 0, 2], [0, 0, 0, 1], useFixedBase=False)
# robot_id = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 2], [0.7071, 0, 0, 0.7071], useFixedBase=False, globalScaling=0.4)
robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit-v3.urdf",[0,0,1.5],[0,0,0,1],useFixedBase = False)
# robot_id = p.loadURDF("cube.urdf",[0,0,1.5],[0,0,0,1],useFixedBase = False)


###### Setup for Joints #######
nJoints = p.getNumJoints(robot_id)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(robot_id, i);jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
left_hip_roll = jointNameToId['left_hip_roll'];left_hip_yaw = jointNameToId['left_hip_yaw'];left_hip_pitch = jointNameToId['left_hip_pitch'];left_knee = jointNameToId['left_knee'];left_shin = jointNameToId['left_shin'];left_tarsus = jointNameToId['left_tarsus'];left_toe_pitch = jointNameToId['left_toe_pitch'];left_toe_roll = jointNameToId['left_toe_roll'];left_heel_spring = jointNameToId['left_heel_spring'];left_toe_A = jointNameToId['left_toe_A'];left_toe_A_rod = jointNameToId['left_toe_A_rod'];left_A2 = jointNameToId['left_A2']
left_toe_B = jointNameToId['left_toe_B'];left_toe_B_rod = jointNameToId['left_toe_B_rod'];left_B2 = jointNameToId['left_B2'];left_achilles_rod = jointNameToId['left_achilles_rod'];left_ach2 = jointNameToId['left_ach2'];left_shoulder_roll = jointNameToId['left_shoulder_roll'];left_shoulder_pitch = jointNameToId['left_shoulder_pitch'];left_shoulder_yaw = jointNameToId['left_shoulder_yaw'];left_elbow = jointNameToId['left_elbow'];right_hip_roll = jointNameToId['right_hip_roll'];right_hip_yaw = jointNameToId['right_hip_yaw'];right_hip_pitch = jointNameToId['right_hip_pitch']
right_knee = jointNameToId['right_knee'];right_shin = jointNameToId['right_shin'];right_tarsus = jointNameToId['right_tarsus'];right_toe_pitch = jointNameToId['right_toe_pitch'];right_toe_roll = jointNameToId['right_toe_roll'];right_heel_spring = jointNameToId['right_heel_spring'];right_toe_A = jointNameToId['right_toe_A'];right_toe_A_rod = jointNameToId['right_toe_A_rod'];right_A2 = jointNameToId['right_A2'];right_toe_B = jointNameToId['right_toe_B'];right_toe_B_rod = jointNameToId['right_toe_B_rod'];right_B2 = jointNameToId['right_B2']
right_achilles_rod = jointNameToId['right_achilles_rod'];right_ach2 = jointNameToId['right_ach2'];right_shoulder_roll = jointNameToId['right_shoulder_roll'];right_shoulder_pitch = jointNameToId['right_shoulder_pitch'];right_shoulder_yaw = jointNameToId['right_shoulder_yaw'];right_elbow = jointNameToId['right_elbow']

p.resetJointState(robot_id,left_hip_roll,0.360407)
p.resetJointState(robot_id,right_hip_roll,-0.360407)
p.resetJointState(robot_id,left_hip_pitch,-0.3)
p.resetJointState(robot_id,right_hip_pitch,0.3)

focus_position, _ = p.getBasePositionAndOrientation(robot_id)
cdist = 3;cyaw = 100;cpitch = -20;cubePos = focus_position
numJoints = p.getNumJoints(robot_id)

####### Change ContactProcessingThreshold ######
for i in range(nJoints):
    p.changeDynamics(robot_id,i,contactProcessingThreshold = contactprocessingthreshold)

cnt = 0
base_name = ''
extension = '.csv';file_number = 1
logFile = f"{'erp-'}{contacterp}{'-'}{base_name}{timestep}{'hz'}{'-backdrop-'}{'thres-'}{contactbreakingthreshold}{'-'}{'process-'}{contactprocessingthreshold}{'-'}{file_number}{extension}"
# logFile = f"{base_name}{timestep}{'hz-'}{'erp-'}{contacterp}{'-'}{file_number}{extension}"
while os.path.isfile(logFile):
    file_number += 1
    logFile = f"{'erp-'}{contacterp}{'-'}{base_name}{timestep}{'hz'}{'-backdrop-'}{'thres-'}{contactbreakingthreshold}{'-'}{'process-'}{contactprocessingthreshold}{'-'}{file_number}{extension}"
    # logFile = f"{base_name}{timestep}{'hz-'}{'erp-'}{contacterp}{'-'}{file_number}{extension}"
    

    
# for step in range(1000):
with open(logFile, 'w', newline='') as csvfile:
    # Create a CSV writer object
    writer = csv.writer(csvfile)
    # writer.writerow(['Start'])
        
    while( cnt < 2000 ):
        p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=(0.001637, 0.0002, 1.259547))
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
        p.setJointMotorControlArray(robot_id, [i for i in range(numJoints)], controlMode=p.TORQUE_CONTROL, forces=[0.0 for i in range(numJoints)])
        p.stepSimulation()
        # time.sleep(1.0/timestep)
        
        contact = p.getContactPoints(robot_id)
        array_contact = np.array(contact)
        
        if(contact):
            # while( cnt < 4 ):
            #   if cnt > -1:
                print("contact")
                print(array_contact)
                print()
                # print(array_contact[:,8])
                print("Get collision shape of left/right foot")
                # print(p.getCollisionShapeData(robot_id,7))
                # print(p.getCollisionShapeData(robot_id,25))
                print()
                writer.writerow(array_contact[:,8])
                cnt = cnt + 1
        else:
            writer.writerow(np.array([0]))
                
            
    
for joint in range(numJoints):
    print(p.getJointInfo(robot_id, joint))

p.disconnect()
print(cubePos)
