import os
import numpy as np
import mujoco
import mujoco.viewer

# In case macOS gets moody about GL
os.environ.setdefault("MUJOCO_GL", "glfw")

# Path to the XML (run this from the OctoBot folder)
MODEL_PATH = "models/spirob_2c_planar.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

n_joints = 6  # joint0 ... joint5

def set_wave_pose(data, t):
    """Simple standing wave that curls and uncurls the tentacle."""
    amp_deg = 18      # max angle at the tip
    freq = 0.25       # Hz

    phase = np.sin(2 * np.pi * freq * t)

    # More bend near the tip, less near the base
    for i in range(n_joints):
        weight = (i + 1) / n_joints      # 1/6 ... 6/6
        target_angle = np.deg2rad(amp_deg) * phase * weight
        data.ctrl[i] = target_angle

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t = data.time

        # set desired joint positions through actuators
        set_wave_pose(data, t)

        mujoco.mj_step(model, data)
        viewer.sync()
