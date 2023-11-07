import mujoco
import mujoco.viewer
xml_path = "mujoco_menagerie/franka_emika_panda/scene.xml"
model = mujoco.MjModel.from_xml_path(xml_path)


# xml = """
# <mujoco>
#   <worldbody>
#     <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
#     <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
#   </worldbody>
# </mujoco>
# """
# model2 = mujoco.MjModel.from_xml_string(xml)

data = mujoco.MjData(model)

mujoco.mj_kinematics(model, data)


while True:
    mujoco.mj_step(model, data)
    print('Total number of DoFs in the model:', model.nv)
    print('Generalized positions:', data.qpos)
    print('Generalized velocities:', data.qvel)
    viewer = mujoco.viewer.launch(model,data)

