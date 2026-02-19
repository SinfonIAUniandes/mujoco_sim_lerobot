import mujoco
import mujoco.viewer
import time
import math
from robot_descriptions.loaders.mujoco import load_robot_description

model = load_robot_description("so_arm101_mj_description")

# 2. Create the Data
# MjData holds the state (qpos, qvel, etc.) and scratch space for computations.
data = mujoco.MjData(model)

# --- DEBUG: CHECK ACTUATORS ---
# It is vital to know if MuJoCo actually sees motors in your XML.
# This prints the number of motors and their names.
print(f"Total Actuators found: {model.nu}")
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"Actuator ID {i}: {name}")

# 2. Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    # Close the viewer to stop the script
    while viewer.is_running():
        step_start = time.time()
        
        # Calculate simulation time for animation
        sim_time = step_start - start_time

        # --- CONTROL LOGIC ---
        # Modify data.ctrl to move the robot.
        # Check if we have at least one actuator to control
        if model.nu > 0:
            # Example: Move the first joint (ID 0) in a sine wave pattern
            # Amplitude: 1.0, Speed: 2.0 rad/s
            data.ctrl[0] = 1.0 * math.sin(2.0 * sim_time)

            # Example: Keep the second joint (ID 1) fixed at a specific angle (if it exists)
            if model.nu > 1:
                data.ctrl[1] = 0.5 
        # ---------------------

        # Step the physics
        mujoco.mj_step(model, data)

        # Sync the viewer with the new physics state
        viewer.sync()