import mujoco
import mujoco_viewer

model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/Michigan/pybullet/xml/digit-v3.xml')
# model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/Michigan/pybullet/xml/hinge-digit-v3-basepinned-armfixed.xml')
# model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/ROAHM/ROAHM_Robust_Control/MuJoCo/bin/digit-v3-basepinned-armfixed.xml')
# model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/ROAHM/ROAHM_Robust_Control/MuJoCo/bin/digit-v3-armfixed-springfixed.xml')
data = mujoco.MjData(model)

print("nq: ",model.nq)
print("nv: ",model.nv)
print("nname: ",model.nnames)
print("njnt: ",model.njnt)
print("nqpos: ",len(data.qpos))
try:
  print(model.geom())
except KeyError as e:
  print("error: ", e)
# print(model.jnt())
# print("joint name: ",model.jnt())
# for  i in range(len(data.qpos)):
#     print(data.qpos[i], " ")

# for  i in range(len(data.qpos)):
#     data.qpos[i] = 0
# data.qpos[0] = 0.361490
# data.qpos[1] = 0.000770
# data.qpos[2] = 0.286025
# data.qpos[3] = 1
# data.qpos[4] = 0.373352
# data.qpos[5] = 0
# data.qpos[6] = -0.346032
# data.qpos[7] = -0.009752
# data.qpos[8] = -0.092650
# data.qpos[9] = 1
# data.qpos[10] = 0.084274
# data.qpos[11] = 1
# data.qpos[12] = 0.091354
# data.qpos[13] = -0.013601
# data.qpos[14] = -0.1506
# data.qpos[15] = 1.0922
# data.qpos[16] = 0.0017
# data.qpos[17] = -0.1391
# data.qpos[18] = -0.360407
# data.qpos[19] = -0.000561
# data.qpos[20] = -0.286076
# data.qpos[21] = 1
# data.qpos[22] = -0.372723
# data.qpos[23] = 0
# data.qpos[24] = 0.347843
# data.qpos[25] = 0.008955
# data.qpos[26] = 0.095860
# data.qpos[27] = 1    #-1
# data.qpos[28] = -0.083562
# data.qpos[29] = 1    #-1
# data.qpos[30] = -0.092658
# data.qpos[31] = 0.019828
# data.qpos[32] = 0.1506
# data.qpos[33] = -1.0922
# data.qpos[34] = -0.0017
# data.qpos[35] = 0.1391
# print("attribute of model : ",dir(model))
# print("attribute of data : ",dir(data))

# joint_names = [data.id2name(joint_id) for joint_id in range(model.njnt)]
# print("joint names: ",joint_names)
# joint_indices = [model.get_joint_qposadr(joint_name) for joint_name in joint_names]

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data,width=1000,height=500)

# simulate and render
for _ in range(10000):
    if viewer.is_alive:
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break

# close
viewer.close()