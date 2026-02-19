import time
import mujoco
import mujoco.viewer
from robot_descriptions.loaders.mujoco import load_robot_description

# If you are not importing the loader and want to use the function 
# you pasted, paste that function definition here instead of the import above.

def simulate_robot(robot_name: str, variant: str = None):
    print(f"Loading {robot_name}...")
    
    # 1. Load the Model
    # Uses the loader from robot_descriptions to get the MJCF XML path
    # and compile it into a binary MjModel.
    try:
        model = load_robot_description(robot_name, variant=variant)
    except ValueError as e:
        print(f"Error: {e}")
        print(f"Tip: Ensure '{robot_name}' actually has an MJCF (MuJoCo) description available.")
        return

    # 2. Create the Data
    # MjData holds the state (qpos, qvel, etc.) and scratch space for computations.
    data = mujoco.MjData(model)

    # 3. Launch the Viewer
    # launch_passive allows us to control the stepping loop ourselves.
    print("Launching viewer... (Press ESC in the viewer to quit)")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # Close the viewer automatically if the window is closed
        while viewer.is_running():
            step_start = time.time()

            # 4. Step the Physics
            # This advances the simulation by one timestep (usually 2ms)
            mujoco.mj_step(model, data)

            # Optional: Apply simple control (e.g., gravity compensation or zero actions)
            # data.ctrl[:] = 0 

            # 5. Sync the Viewer
            # Updates the graphical representation with the new physics state
            viewer.sync()

            # Time keeping to match real-time (approximate)
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    # Example: Loading the Cassie robot (which typically includes MJCF)
    # You can change this string to other robots available in the library
    simulate_robot("so_arm101_mj_description")
    
    # Example 2: Loading a specific variant (if supported by the description)
    # simulate_robot("go1_mj_description", variant="go1")