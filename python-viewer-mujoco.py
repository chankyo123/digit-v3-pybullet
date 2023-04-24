import mujoco
import mujoco_viewer

# model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/Michigan/pybullet/xml/humanoid.xml')  #nj=27
# model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/Michigan/pybullet/xml/diamond.xml')
# model = mujoco.MjModel.from_xml_path('/Users/ckkim/.local/lib/python3.10/site-packages/pybullet_data/mjcf/inverted_double_pendulum.xml')
model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/Michigan/pybullet/xml/digit-v3-basepinned-armfixed.xml')
# model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/Michigan/pybullet/xml/hinge-digit-v3-basepinned-armfixed.xml')
# model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/ROAHM/ROAHM_Robust_Control/MuJoCo/bin/digit-v3-basepinned-armfixed.xml')
# model = mujoco.MjModel.from_xml_path('/Users/ckkim/Chankyo Kim/ROAHM/ROAHM_Robust_Control/MuJoCo/bin/digit-v3-armfixed-springfixed.xml')
print("nq: ",model.nq)
print("nv: ",model.nv)
print("nname: ",model.nnames)
# print(model.jnt())
# print("joint name: ",model.jnt())
data = mujoco.MjData(model)

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