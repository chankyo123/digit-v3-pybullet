import numpy as np
import pybullet as p
import pybullet_data
import time

#bioler plate
p.connect(p.GUI) #or p.DIRECT
# p.connect(p.DIRECT) #or p.GUI
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
p.setTimeStep(0.0005)
# Load assets
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])  #asset about ground plane. position and quaternion
targid = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 

#### Initial condition ####
jntIdx=[i for i in range(p.getNumJoints(targid))]
initCond=[0.361490,0.000770, 0.286025,0.373352, 0, -0.346032,0.091354, -0.013601,-0.1506, 1.0922, 0.0017, -0.1391,0,-0.360407, -0.000561, -0.286076,-0.372723, 0, 0.347843,-0.092658, 0.019828,0.1506, -1.0922, -0.0017, 0.1391,0,0,0]
for i in range(len(jntIdx)):
    p.resetJointState(targid,jntIdx[i],initCond[i])

# print(p.getNumJoints(targid))  #28 different joint

### getJointInfo
# for i in range(p.getNumJoints(targid)):
    # print(p.getJointInfo(targid,i)[12].decode(),end="/") 
    #left_hip_roll/left_hip_yaw/left_hip_pitch/left_knee/left_shin/left_tarsus/left_toe_pitch/left_toe_roll/left_shoulder_roll/left_shoulder_pitch/left_shoulder_yaw/left_elbow/left_shoulder_cap/right_hip_roll/right_hip_yaw/right_hip_pitch/right_knee/right_shin/right_tarsus/right_toe_pitch/right_toe_roll/right_shoulder_roll/right_shoulder_pitch/right_shoulder_yaw/right_elbow/right_shoulder_cap/left_waist_cap/right_waist_cap/

# print(p.getJointInfo(targid, 0)) ## left_hip_roll
# jtype = p.getJointInfo(targid, jointid)[2]
# jlower = p.getJointInfo(targid, jointid)[8]
# jupper = p.getJointInfo(targid, jointid)[9]
jlower, jupper = -1.0472, 1.0472
# print(jlower,jupper)
    
##### CONTROL INPUT AND CAMERA SETTINGS #####
focus_position, _ = p.getBasePositionAndOrientation(targid)  #return position and orientation of targid(focus)
cdist = 3;cyaw=100;cpitch=-20;cubePos=focus_position
while(1):
    
    joint_zero_targ = np.random.uniform(jlower,jupper)
    joint_zero_targ = -1
    p.setJointMotorControlArray(targid, [0], p.POSITION_CONTROL, targetPositions = [joint_zero_targ])
    p.stepSimulation()
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