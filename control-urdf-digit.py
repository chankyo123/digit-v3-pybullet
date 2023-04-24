import numpy as np
import pybullet as p
import pybullet_data
import time
import csv

#logFile
logFile = "./data/mjc-bullet/digitBasePinned_test.csv"
#init logFile
with open(logFile,'w') as f:
    0

#bioler plate
p.connect(p.GUI) #or p.DIRECT
# p.connect(p.DIRECT) #or p.GUI
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# quadruped = p.loadURDF("quadruped/quadruped.urdf")

p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
p.setTimeStep(0.0005)
# Load assets
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])  #asset about ground plane. position and quaternion
targid = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 
# targid = p.loadURDF("quadruped/minitaur_v1.urdf", [1, -1, .3])

nJoints = p.getNumJoints(targid)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(targid, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
print(jointNameToId)

print('num joint:',p.getNumJoints(targid))  #12 different joint
print('num constraint:',p.getNumConstraints())

#### Initial condition ####
jntIdx=[i for i in range(p.getNumJoints(targid))]
initCond=[0.,0.000770, 0.286025,0.373352, 0, -0.346032,0.091354, -0.013601,-0.1506, 1.0922, 0.0017, -0.1391,0,-0.360407, -0.000561, -0.286076,-0.372723, 0, 0.347843,-0.092658, 0.019828,0.1506, -1.0922, -0.0017, 0.1391,0,0,0]
# print("init cond:", len(initCond)) #28
for i in range(len(jntIdx)):
    p.resetJointState(targid,jntIdx[i],initCond[i])

# print(p.getNumJoints(targid))  #28 different joint

#### getJointInfo
# for i in range(p.getNumJoints(targid)):
    # print(p.getJointInfo(targid,i)[12].decode(),end="/") 
    #left_hip_roll[0]/left_hip_yaw[1]/left_hip_pitch[2]/left_knee[3]/left_shin[4]/left_tarsus[5]/left_toe_pitch[6]/left_toe_roll[7]/left_shoulder_roll[8]/left_shoulder_pitch[9]/left_shoulder_yaw[10]/left_elbow[11]/left_shoulder_cap[12]/
    #right_hip_roll[13]/right_hip_yaw[14]/right_hip_pitch[15]/right_knee[16]/right_shin[17]/right_tarsus[18]/right_toe_pitch[19]/right_toe_roll[20]/right_shoulder_roll[21]/right_shoulder_pitch[22]/right_shoulder_yaw[23]/right_elbow[24]/right_shoulder_cap[25]/left_waist_cap[26]/right_waist_cap[27]/

# print(p.getJointInfo(targid, 0)) ## left_hip_roll

##### CONTROL INPUT AND CAMERA SETTINGS #####
focus_position, _ = p.getBasePositionAndOrientation(targid)  #return position and orientation of targid(focus)
cdist = 3;cyaw=100;cpitch=-20;cubePos=focus_position
# desiredPosition=[0.36149,0.00077, -1.0,0.373352,0.1,-0.360407,-0.000561,-1.0,`0.372723,-0.1]
desiredPosition=[0.36149,0.00077, \
                -0.4488 , \
                # -0.286076, \
                0.373352,-0.360407,-0.000561,
                -1 , \
                # -0.286076,  \ #right-hip-pitch
                    -0.372723]  #same condition in mujoco simulation
# desiredPosition=[0.36149,0.00077, -0.5,0.373352,-0.360407,-0.000561,-0.286076,-0.372723]
desiredVelocity=[0 for i in range(len(desiredPosition))]
# jointIdx= [0,1,2,3,7,13,14,15,16,20]
jointIdx= [0,1,2,3,13,14,15,16]
gP=[400 for i in range(len(desiredPosition))]
gV=[50 for i in range(len(desiredPosition))]
# def writeState():
#     with open(logFile,'a') as f:
#         row=[]
#         writer = csv.writer(f)
#         for i in range(28):
#             row.append(str(p.getJointStates(targid,[i for i in range(p.getNumJoints(targid))])[i][0])+' ')
#             # row.append(' ')
#         print(row)
#         writer.writerow(row)
#     # logId = p.startStateLogging(p.STATE_LOGGING_MINITAUR, "./data/mjc-bullet/LOG001.TXT", [quadruped])
#     # k =p.getJointStates(targid,[i for i in range(p.getNumJoints(targid))])[0]
#     # k =p.getJointStates(targid,[0])[0][0]  #get jointPosition
#     # print(k)
#     # return logId
#     return 0
    
while(1):
    
    
    p.setJointMotorControlArray(targid,jointIdx, p.POSITION_CONTROL, targetPositions = desiredPosition,targetVelocities=desiredVelocity,positionGains=gP,velocityGains=gV)
    p.stepSimulation()
    # logId = writeState()
    # print(writeState())
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
    # p.stopStateLogging(logId)
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

