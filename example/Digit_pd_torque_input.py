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
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/model/urdf/digit-v3/digit-v3-armfixed.urdf", [0,0,1.5],[0,0,0,1], useFixedBase = True) 
# robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/model/urdf/digit-v3/digit-v3-armfixed-zerofriction.urdf", [0,0,1.5],[0,0,0,1], useFixedBase = True) 
robot_id = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/model/urdf/digit-v3/digit-v3-armfixed-zeromotor.urdf", [0,0,1.5],[0,0,0,1], useFixedBase = True) 


###### Setup for Joints #######
nJoints = p.getNumJoints(robot_id)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(robot_id, i);jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
left_hip_roll = jointNameToId['left_hip_roll'];left_hip_yaw = jointNameToId['left_hip_yaw'];left_hip_pitch = jointNameToId['left_hip_pitch'];left_knee = jointNameToId['left_knee'];left_shin = jointNameToId['left_shin'];left_tarsus = jointNameToId['left_tarsus'];left_toe_pitch = jointNameToId['left_toe_pitch'];left_toe_roll = jointNameToId['left_toe_roll'];left_heel_spring = jointNameToId['left_heel_spring'];left_toe_A = jointNameToId['left_toe_A'];left_toe_A_rod = jointNameToId['left_toe_A_rod'];left_A2 = jointNameToId['left_A2']
left_toe_B = jointNameToId['left_toe_B'];left_toe_B_rod = jointNameToId['left_toe_B_rod'];left_B2 = jointNameToId['left_B2'];left_achilles_rod = jointNameToId['left_achilles_rod'];left_ach2 = jointNameToId['left_ach2'];left_shoulder_roll = jointNameToId['left_shoulder_roll'];left_shoulder_pitch = jointNameToId['left_shoulder_pitch'];left_shoulder_yaw = jointNameToId['left_shoulder_yaw'];left_elbow = jointNameToId['left_elbow']
right_hip_roll = jointNameToId['right_hip_roll'];right_hip_yaw = jointNameToId['right_hip_yaw'];right_hip_pitch = jointNameToId['right_hip_pitch']
right_knee = jointNameToId['right_knee'];right_shin = jointNameToId['right_shin'];right_tarsus = jointNameToId['right_tarsus'];right_toe_pitch = jointNameToId['right_toe_pitch'];right_toe_roll = jointNameToId['right_toe_roll'];right_heel_spring = jointNameToId['right_heel_spring'];right_toe_A = jointNameToId['right_toe_A'];right_toe_A_rod = jointNameToId['right_toe_A_rod'];right_A2 = jointNameToId['right_A2'];right_toe_B = jointNameToId['right_toe_B'];right_toe_B_rod = jointNameToId['right_toe_B_rod'];right_B2 = jointNameToId['right_B2']
right_achilles_rod = jointNameToId['right_achilles_rod'];right_ach2 = jointNameToId['right_ach2'];right_shoulder_roll = jointNameToId['right_shoulder_roll'];right_shoulder_pitch = jointNameToId['right_shoulder_pitch'];right_shoulder_yaw = jointNameToId['right_shoulder_yaw'];right_elbow = jointNameToId['right_elbow']

for i in range(nJoints) :
    p.resetJointState(robot_id, i, targetValue = 0, targetVelocity = 0)
    
p.resetJointState(robot_id,left_hip_pitch,targetValue=0.02,targetVelocity=0)
p.resetJointState(robot_id,left_hip_roll,targetValue=0.360407,targetVelocity=0)
p.resetJointState(robot_id,left_hip_yaw,targetValue=0.01,targetVelocity=0)
p.resetJointState(robot_id,left_knee,targetValue=0.03,targetVelocity=0)
p.resetJointState(robot_id,left_toe_A,targetValue=0.001,targetVelocity=0)
p.resetJointState(robot_id,left_toe_B,targetValue=0.001,targetVelocity=0)

p.resetJointState(robot_id,right_hip_pitch,targetValue=-0.02,targetVelocity=0)
p.resetJointState(robot_id,right_hip_roll,targetValue=-0.360407,targetVelocity=0)
p.resetJointState(robot_id,right_hip_yaw,targetValue=-0.01,targetVelocity=0)
p.resetJointState(robot_id,right_knee,targetValue=-0.03,targetVelocity=0)
p.resetJointState(robot_id,right_toe_A,targetValue=-0.001,targetVelocity=0)
p.resetJointState(robot_id,right_toe_B,targetValue=-0.001,targetVelocity=0)
        
focus_position, _ = p.getBasePositionAndOrientation(robot_id)
cdist = 3;cyaw = 100;cpitch = -20;cubePos = focus_position
numJoints = p.getNumJoints(robot_id)

####### Change ContactProcessingThreshold ######
for i in range(nJoints):
    p.changeDynamics(robot_id,i,contactProcessingThreshold = contactprocessingthreshold)
    

###### Logging #######
cnt = 0
extension = '.csv';file_number_ch = 1
logFile_checkJoints = f"{'/Users/ckkim/Chankyo Kim/Michigan/pybullet/data/'}{'pd-'}{file_number_ch}{extension}"
    
# logFile = f"{'erp-'}{contacterp}{'-'}{base_name}{timestep}{'hz'}{'-backdrop-'}{'thres-'}{contactbreakingthreshold}{'-'}{'process-'}{contactprocessingthreshold}{'-'}{file_number}{extension}"
# logFile = f"{base_name}{timestep}{'hz-'}{'erp-'}{contacterp}{'-'}{file_number}{extension}"

while os.path.isfile(logFile_checkJoints):
    file_number_ch += 1
    logFile_checkJoints = f"{'/Users/ckkim/Chankyo Kim/Michigan/pybullet/data/'}{'pd-'}{file_number_ch}{extension}"
    
    
    
###### MassMatrix #######
print("nJoints: ")
print(nJoints)
MassMatrix = np.array(p.calculateMassMatrix(robot_id,[0 for i in range(nJoints)]))
print()
print(MassMatrix)

####### Create Constraints #######

#Calculate Position of Closed Looped Anchors (Equation attached in Document: Refer to Chankyo Kim)
cid1= p.createConstraint(robot_id, left_heel_spring, robot_id, left_ach2, p.JOINT_POINT2POINT, [0, 0, 1], [0.113789, -0.011056, 0.], [0.260039048764136, -0.055149132747197,  0.001072511047511])
cid2= p.createConstraint(robot_id, left_toe_roll, robot_id, left_A2, p.JOINT_POINT2POINT, [0, 0, 1], [0.0179, -0.009551, -0.054164], [0.188700873535645, 0.031025632095949, 0.001961040077076])
cid3= p.createConstraint(robot_id, left_toe_roll, robot_id, left_B2, p.JOINT_POINT2POINT, [0, 0, 1], [-0.0181, -0.009551, -0.054164], [0.164841711315360, -0.030867345479882, -0.002471140022936])

cid4= p.createConstraint(robot_id, right_heel_spring, robot_id, right_ach2, p.JOINT_POINT2POINT, [0, 0, 1], [0.113789, 0.011056, 0], [0.260039048764136, 0.055149132747197,  0.001072511047511])
cid5= p.createConstraint(robot_id, right_toe_roll, robot_id, right_A2, p.JOINT_POINT2POINT, [0, 0, 1], [0.0179, 0.009551, -0.054164], [0.188700873535645, -0.031025632095949, 0.001961040077076])
cid6= p.createConstraint(robot_id, right_toe_roll, robot_id, right_B2, p.JOINT_POINT2POINT, [0, 0, 1], [-0.0181, 0.009551, -0.054164], [0.164841711315360, 0.030867345479882, -0.002471140022936])

filename = '/Users/ckkim/Chankyo Kim/Michigan/pybullet/data/checkJoints-17.csv'
# filename = 'euler-Joints-4.csv'

row_lists = []
check_joints = [left_hip_roll, left_hip_yaw, left_hip_pitch, left_knee, right_hip_roll, right_hip_yaw, right_hip_pitch, right_knee, left_toe_A, left_toe_B, right_toe_A, right_toe_B]
time_data = 0


with open(filename, 'r') as file, open(logFile_checkJoints, 'w', newline='') as csvfile_check:
# with open(filename, 'r') as file:

    # writer_check = csv.writer(csvfile_check)
    reader = csv.reader(file)
    for row in reader:
        if row:  # Skip empty rows if any
            cols  = int(len(row[:-1])/3)   #pos, vel, torq data
        print('cols : ', cols)
        row_torq = row[-cols-1:-1]  # Get the row without the last element
        row_torq = [float(num_str) for num_str in row_torq]            

# with open(logFile_checkJoints, 'w', newline='') as csvfile_check:
#     writer_check = csv.writer(csvfile_check)
#     while( cnt < 2000 ):
#         row_torq = [-100,100,1000,-100,100,-100,-1000,100,5,2,-5,-2]
        
        # row_lists.append(row_torq)  # Add it to the list
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
        
        check_J_list = p.getJointStates(robot_id, check_joints)
        check_list = []
        for i in range(len(check_joints)):
            check_list.append(check_J_list[i][0])
        for i in range(len(check_joints)):
            check_list.append(check_J_list[i][1])
        for i in range(len(check_joints)):
            check_list.append(check_J_list[i][3])
        check_list.append(time_data)    
        # print("init torq for matlab : ",torq_list)    
        # writer_check.writerow(check_list) 
    
        p.setJointMotorControlArray(robot_id, check_joints, p.TORQUE_CONTROL, forces = row_torq)
        # p.setJointMotorControlArray(robot_id,[left_hip_pitch], p.TORQUE_CONTROL, forces = [2000])
        
        p.stepSimulation()
        time_data = time_data + 1.0/timestep
        
        # time.sleep(1.0/timestep)

p.disconnect()
# print(cubePos)
