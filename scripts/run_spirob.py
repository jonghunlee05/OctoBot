import os
import mujoco
import mujoco.viewer
import math

os.environ.setdefault("MUJOCO_GL", "glfw")

MODEL_PATH = "models/stick_simple.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Get actuator ID
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "hinge_actuator")

# Current target angle in degrees
target_angle_deg = 0.0

# Angle limits (degrees)
MIN_ANGLE = -90.0
MAX_ANGLE = 90.0

def clamp(x, lo, hi):
    """Clamp value between lo and hi"""
    return max(lo, min(hi, x))

def key_callback(keycode):
    global target_angle_deg
    
    if 32 <= keycode <= 126:
        c = chr(keycode).lower()
        
        if c == "a":
            # Press A: turn -10 degrees
            target_angle_deg = clamp(target_angle_deg - 10.0, MIN_ANGLE, MAX_ANGLE)
            print(f"Target angle: {target_angle_deg:.1f} degrees")
        elif c == "d":
            # Press D: turn +10 degrees
            target_angle_deg = clamp(target_angle_deg + 10.0, MIN_ANGLE, MAX_ANGLE)
            print(f"Target angle: {target_angle_deg:.1f} degrees")

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    while viewer.is_running():
        # Convert degrees to radians and set control
        target_angle_rad = math.radians(target_angle_deg)
        data.ctrl[actuator_id] = target_angle_rad
        
        # Step simulation
        mujoco.mj_step(model, data)
        viewer.sync()
