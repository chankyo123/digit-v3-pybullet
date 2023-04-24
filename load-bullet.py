import pybullet as p
import time
import pybullet_data
import numpy as np

# Connect to PyBullet physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #/Users/ckkim/.local/lib/python3.10/site-packages/pybullet_data

p.setGravity(0,0,-9.8)
p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])  #asset about ground plane. position and quaternion
shift = [0, -0.02, 0]
meshScale = [0.1, 0.1, 0.1]
fileName = "/Users/ckkim/Chankyo Kim/Michigan/pybullet/xml/obj/hip-roll-housing.obj"
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=fileName, rgbaColor=[1, 1, 1, 1], specularColor=[0.4, .4, 0], visualFramePosition=shift, meshScale=meshScale)
rangex = 1
rangey = 1
for i in range(rangex):
  for j in range(rangey):
    p.createMultiBody(baseMass=1, baseInertialFramePosition=[0, 0, 0], baseCollisionShapeIndex=visualShapeId, baseVisualShapeIndex=visualShapeId, basePosition=[((-rangex / 2) + i) * meshScale[0] * 2, (-rangey / 2 + j) * meshScale[1] * 2, 1], useMaximalCoordinates=True)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
print("loaded obj file : ",fileName)

# mjcf_file = "mjcf/half_cheetah.xml"
# mjcf_file = "xml/diamond.xml"
# robot_id = p.loadMJCF(mjcf_file)[0]

# basePos, baseOrn = p.getBasePositionAndOrientation(robot_id)
# print("base info")
# print("base position : ",basePos)
# print("base orientation : ",baseOrn)

# print(pybullet_data.getDataPath())

# nJoints = p.getNumJoints(robot_id)
# jointNameToId = {}
# for i in range(nJoints):
#   jointInfo = p.getJointInfo(robot_id, i)
#   jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
# print(jointNameToId)
# print('num joint:',p.getNumJoints(robot_id))  #12 different joint


# focus_position, _ = p.getBasePositionAndOrientation(robot_id)  #return position and orientation of targid(focus)
# cdist = 3;cyaw=100;cpitch=-20;cubePos=focus_position

# Simulate the robot for some time
for i in range(10000):
    p.stepSimulation()
    time.sleep(.01)
    # p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)
    p.getMouseEvents()
    keys = p.getKeyboardEvents()
    #Keys to change camera
    # if keys.get(100):  #D
    #     cyaw+=0.5
    # if keys.get(97):   #A
    #     cyaw-=0.5
    # if keys.get(99):   #C
    #     cpitch+=1
    # if keys.get(102):  #F
    #     cpitch-=1
    # if keys.get(122):  #Z
    #     cdist+=.01
    # if keys.get(120):  #X
    #     cdist-=.01
    p.stepSimulation()
    

# Disconnect from the physics server
p.disconnect()