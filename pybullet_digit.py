import numpy as np
import pybullet as p
import pybullet_data
import time

#bioler plate
p.connect(p.GUI) #or p.DIRECT
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

# Load assets
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])  #asset about ground plane. position and quaternion
# targid = p.loadMJCF("/Users/ckkim/Chankyo Kim/ROAHM/ROAHM_Robust_Control/MuJoCo/bin/digit-v3-basepinned-armfixed-springFixed_v2.xml")
# targid = p.loadMJCF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/digit.xml")[0] 
targid = p.loadURDF("/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/digit_model.urdf",[0,0,10],[0,0,0,1], useFixedBase = True) 
# obj_of_focus = targid  #make camera focus on specified target(or assent)

# print(p.getNumJoints(targid))  #12 different joint
# jointid = 4
# jtype = p.getJointInfo(targid, jointid)[2]
# jlower = p.getJointInfo(targid, jointid)[8]
# jupper = p.getJointInfo(targid, jointid)[9]
# jlower, jupper = -2.9671, 2.9671
# print(jlower,jupper)

# changing joint angles

'''for step in range(10000):
    joint_two_targ = np.random.uniform(jlower,jupper)
    joint_four_targ = np.random.uniform(jlower,jupper)
    p.setJointMotorControlArray(targid, [2,4], p.POSITION_CONTROL, targetPositions = [joint_two_targ, joint_four_targ])
    p.stepSimulation()
    #if you need to query joint states or link states to update observation
    #print(p.getLinkStates(targid, [2,4]))
    # print(p.getJointStates(targid, [2,4]))
'''    
# step default camera position with zero action, lasting 5 secs
for step in range(200):
    focus_position, _ = p.getBasePositionAndOrientation(targid)  #return position and orientation of targid(focus)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition = focus_position)
    p.stepSimulation()
    time.sleep(.01)
    
    
    
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