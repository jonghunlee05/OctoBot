import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_string("<mujoco/>")
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
