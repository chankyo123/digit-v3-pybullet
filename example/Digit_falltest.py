import pybullet as p
import pybullet_data
import numpy as np
import time
import csv
import os

####### Connect to the physics simulation 'PyBullet' #######
p.connect(p.GUI)
# p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())   #import pybullet_data and register the directory

####### Load Robot Models, Set simulation parameters #######
p.loadURDF("plane.urdf",[0,0,0])
p.setGravity(0, 0, -9.8)
timestep = 2000
p.setTimeStep(1. / timestep)
contacterp = 10
contactbreakingthreshold=0.02
contactprocessingthreshold = 0.005
p.setDefaultContactERP(contacterp)
p.setPhysicsEngineParameter(contactBreakingThreshold=contactbreakingthreshold)
urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
# robot_id = p.loadURDF("../model/urdf/digit-v3/digit-v3-armfixed.urdf", [0,0,1.5],[0,0,0,1], useFixedBase = True, flags=urdfFlags) 
# robot_id = p.loadURDF("../model/urdf/digit-v3/digit-v3-armfixed-zeromotor.urdf", [0,0,1.5],[0,0,0,1], useFixedBase = True, flags=urdfFlags) 
robot_id = p.loadURDF("../model/urdf/digit-v3/digit-v3-armfixed.urdf", [0,0,1],[0,0,0,1], useFixedBase = False) 


####### Setup for Joints #######
nJoints = p.getNumJoints(robot_id)  #42
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(robot_id, i);jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
left_hip_roll = jointNameToId['left_hip_roll'];left_hip_yaw = jointNameToId['left_hip_yaw'];left_hip_pitch = jointNameToId['left_hip_pitch'];left_knee = jointNameToId['left_knee'];left_shin = jointNameToId['left_shin'];left_tarsus = jointNameToId['left_tarsus'];left_toe_pitch = jointNameToId['left_toe_pitch'];left_toe_roll = jointNameToId['left_toe_roll'];left_heel_spring = jointNameToId['left_heel_spring'];left_toe_A = jointNameToId['left_toe_A'];left_toe_A_rod = jointNameToId['left_toe_A_rod'];left_A2 = jointNameToId['left_A2']
left_toe_B = jointNameToId['left_toe_B'];left_toe_B_rod = jointNameToId['left_toe_B_rod'];left_B2 = jointNameToId['left_B2'];left_achilles_rod = jointNameToId['left_achilles_rod'];left_ach2 = jointNameToId['left_ach2'];left_shoulder_roll = jointNameToId['left_shoulder_roll'];left_shoulder_pitch = jointNameToId['left_shoulder_pitch'];left_shoulder_yaw = jointNameToId['left_shoulder_yaw'];left_elbow = jointNameToId['left_elbow'];right_hip_roll = jointNameToId['right_hip_roll'];right_hip_yaw = jointNameToId['right_hip_yaw'];right_hip_pitch = jointNameToId['right_hip_pitch']
right_knee = jointNameToId['right_knee'];right_shin = jointNameToId['right_shin'];right_tarsus = jointNameToId['right_tarsus'];right_toe_pitch = jointNameToId['right_toe_pitch'];right_toe_roll = jointNameToId['right_toe_roll'];right_heel_spring = jointNameToId['right_heel_spring'];right_toe_A = jointNameToId['right_toe_A'];right_toe_A_rod = jointNameToId['right_toe_A_rod'];right_A2 = jointNameToId['right_A2'];right_toe_B = jointNameToId['right_toe_B'];right_toe_B_rod = jointNameToId['right_toe_B_rod'];right_B2 = jointNameToId['right_B2']
right_achilles_rod = jointNameToId['right_achilles_rod'];right_ach2 = jointNameToId['right_ach2'];right_shoulder_roll = jointNameToId['right_shoulder_roll'];right_shoulder_pitch = jointNameToId['right_shoulder_pitch'];right_shoulder_yaw = jointNameToId['right_shoulder_yaw'];right_elbow = jointNameToId['right_elbow']


p.resetJointState(robot_id,left_hip_pitch,targetValue=0,targetVelocity=0)
p.resetJointState(robot_id,right_hip_pitch,targetValue=0,targetVelocity=0)
p.resetJointState(robot_id,left_hip_roll,targetValue=0.360407,targetVelocity=0)
p.resetJointState(robot_id,right_hip_roll,targetValue=-0.360407,targetVelocity=0)

for i in range(nJoints) :
    if i != left_hip_roll and i != right_hip_roll and i != left_hip_pitch and i != right_hip_pitch:
        p.resetJointState(robot_id, i, targetValue = 0, targetVelocity = 0)

focus_position, _ = p.getBasePositionAndOrientation(robot_id)
cdist = 3;cyaw = 100;cpitch = -20;cubePos = focus_position
numJoints = p.getNumJoints(robot_id)

####### Change ContactProcessingThreshold #######
for i in range(nJoints):
    p.changeDynamics(robot_id,i,contactProcessingThreshold = contactprocessingthreshold)

####### Logging #######
cnt = 0
base_name = ''
extension = '.csv';file_number = 1
logFile = f"{'../data/'}{'test-drop-'}{file_number}{extension}"

while os.path.isfile(logFile):
    file_number += 1
    logFile = f"{'../data/'}{'test-drop-'}{file_number}{extension}"
    
####### Create Constraints #######
# Calculate Position of Closed Looped Anchors (Equation attached in Document: Refer to Chankyo Kim)
cid1= p.createConstraint(robot_id, left_heel_spring, robot_id, left_ach2, p.JOINT_POINT2POINT, [0, 0, 1], [0.113789, -0.011056, 0.], [0.260039048764136, -0.055149132747197,  0.001072511047511])
cid2= p.createConstraint(robot_id, left_toe_roll, robot_id, left_A2, p.JOINT_POINT2POINT, [0, 0, 1], [0.0179, -0.009551, -0.054164], [0.188700873535645, 0.031025632095949, 0.001961040077076])
cid3= p.createConstraint(robot_id, left_toe_roll, robot_id, left_B2, p.JOINT_POINT2POINT, [0, 0, 1], [-0.0181, -0.009551, -0.054164], [0.164841711315360, -0.030867345479882, -0.002471140022936])
cid4= p.createConstraint(robot_id, right_heel_spring, robot_id, right_ach2, p.JOINT_POINT2POINT, [0, 0, 1], [0.113789, 0.011056, 0], [0.260039048764136, 0.055149132747197,  0.001072511047511])
cid5= p.createConstraint(robot_id, right_toe_roll, robot_id, right_A2, p.JOINT_POINT2POINT, [0, 0, 1], [0.0179, 0.009551, -0.054164], [0.188700873535645, -0.031025632095949, 0.001961040077076])
cid6= p.createConstraint(robot_id, right_toe_roll, robot_id, right_B2, p.JOINT_POINT2POINT, [0, 0, 1], [-0.0181, 0.009551, -0.054164], [0.164841711315360, 0.030867345479882, -0.002471140022936])



check_joints = [left_hip_roll, left_hip_yaw, left_hip_pitch, left_knee, right_hip_roll, right_hip_yaw, right_hip_pitch, right_knee, left_toe_A, left_toe_B, right_toe_A, right_toe_B]
once = 0
og_joints = [i for i in range(nJoints)]
other_joints = [x for x in og_joints if x not in check_joints]
matlab_joint = [left_hip_roll, left_hip_yaw, left_hip_pitch, left_knee, left_shin, left_tarsus, left_toe_pitch, left_toe_roll, right_hip_roll, right_hip_yaw, right_hip_pitch, right_knee, right_shin, right_tarsus, right_toe_pitch, right_toe_roll, left_toe_A, left_toe_B, right_toe_A, right_toe_B, left_toe_A_rod, left_A2, left_toe_B_rod, left_B2, right_toe_A_rod, right_A2, right_toe_B_rod, right_B2, left_achilles_rod, left_ach2, left_heel_spring, right_achilles_rod, right_ach2, right_heel_spring]

init_pos = p.getJointStates(robot_id, matlab_joint)
init_pos_list = []
for i in range(len(matlab_joint)):
    init_pos_list.append(init_pos[i][0])
print("init pos for matlab : ", init_pos_list)

pos_vel = p.getJointStates(robot_id, [i for i in range(nJoints)])
# print("pos : ", jointpos, "vel : ", jointvel)

for i in range(nJoints):
    print(p.getJointInfo(robot_id,i)[1].decode('UTF-8'), " ", i,  " pos : ", pos_vel[i][0], "vel : ", pos_vel[i][1])
    print("damping, friction is : ",p.getJointInfo(robot_id,i)[6],p.getJointInfo(robot_id,i)[7])
    

time_data = 0    
enableControl = False

####### Run Simulation #######

# for step in range(1000):
with open(logFile, 'w', newline='') as csvfile:
    # Create a CSV writer object
    writer = csv.writer(csvfile)
    # writer.writerow(['Start'])
    # writer.writerow(list(jointNameToId.keys()))
        
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
            
        check_J_list = p.getJointStates(robot_id, check_joints)
        check_list = []
        ####### Logging #######
        for i in range(len(check_joints)):
            check_list.append(check_J_list[i][0])
        for i in range(len(check_joints)):
            check_list.append(check_J_list[i][1])
        check_list.append(time_data)    
        # writer.writerow(check_list)
        
        ####### Provide Desired Control #######
        if enableControl:
            p.setJointMotorControlArray(robot_id, og_joints, p.VELOCITY_CONTROL, forces = [0 for i in range(len(og_joints))])


        # ####### Get Contact Data > #######
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
                
                writer.writerow(array_contact[:,8])
                # writer.writerow(np.append(np.append(array_contact[:,3],array_contact[:,5]),array_contact[:,8]))
                cnt = cnt + 1
        else:
            writer.writerow(np.array([0]))
        # ####### < Get Contact Data #######            
        
        p.stepSimulation()
        time_data = time_data + 1.0/timestep
        
        
p.disconnect()
