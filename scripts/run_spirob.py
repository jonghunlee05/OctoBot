import os
import mujoco
import mujoco.viewer
import math

os.environ.setdefault("MUJOCO_GL", "glfw")

MODEL_PATH = "models/stick_simple.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Get actuator IDs for left and right cables
left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_motor")
right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_motor")

# Current cable length changes (negative = shorter cable = that side bends)
left_cable = 0.0
right_cable = 0.0

# Cable limits (how much we can shorten/lengthen the cables)
# Smaller range = more restrictive bending
MIN_CABLE = -0.05
MAX_CABLE = 0.05

# How much one key press changes cable length
# Smaller step = more precise control
STEP = 0.002

def clamp(x, lo, hi):
    """Clamp value between lo and hi"""
    return max(lo, min(hi, x))

def key_callback(keycode):
    global left_cable, right_cable
    
    if 32 <= keycode <= 126:
        c = chr(keycode).lower()
        
        if c == "a":
            # Press A: shorten right cable (go more negative) = bend right
            # When right cable shortens, left cable lengthens (antagonistic)
            right_cable = clamp(right_cable - STEP, MIN_CABLE, MAX_CABLE)
            left_cable = clamp(left_cable + STEP, MIN_CABLE, MAX_CABLE)
            print(f"Left cable: {left_cable:.5f}, Right cable: {right_cable:.5f} (range: {MIN_CABLE} to {MAX_CABLE})")
        elif c == "d":
            # Press D: shorten left cable (go more negative) = bend left
            # When left cable shortens, right cable lengthens (antagonistic)
            left_cable = clamp(left_cable - STEP, MIN_CABLE, MAX_CABLE)
            right_cable = clamp(right_cable + STEP, MIN_CABLE, MAX_CABLE)
            print(f"Left cable: {left_cable:.5f}, Right cable: {right_cable:.5f} (range: {MIN_CABLE} to {MAX_CABLE})")
        elif c == "s":
            # Press S: reset cables to neutral
            left_cable = 0.0
            right_cable = 0.0
            print("Reset to neutral position")

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    print("Controls:")
    print("  A = shorten right cable (bend right)")
    print("  D = shorten left cable (bend left)")
    print("  S = reset to neutral")
    print("Starting simulation...")
    
    while viewer.is_running():
        # Apply cable controls
        # The control value represents length change: negative = shorten cable = that side bends
        data.ctrl[left_id] = left_cable
        data.ctrl[right_id] = right_cable
        
        # Step simulation (mj_step handles forward dynamics internally)
        mujoco.mj_step(model, data)
        viewer.sync()
