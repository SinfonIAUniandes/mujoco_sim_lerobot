import mujoco
import mujoco.viewer
import time

def load_robot_description(model_path: str) -> mujoco.MjModel:
    """Load a robot description in MuJoCo.

    Args:
        model_path: Path to the MJCF XML file describing the robot.

    Returns:
        Robot model for MuJoCo.
    """
    try:
        return mujoco.MjModel.from_xml_path(model_path)
    except ValueError:
        print(f"{model_path} not found. Loading default robot model.")
        return None

PACKAGE_PATH: str = "./robotstudio_so101/"

MJCF_FILE: str = "so101.xml" #can be so101.xml, scene.xml or scene_box.xml


model = load_robot_description(PACKAGE_PATH+MJCF_FILE)
data = mujoco.MjData(model)

# 2. Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer to stop the script
    while viewer.is_running():
        step_start = time.time()

        # Step the physics
        mujoco.mj_step(model, data)

        # Sync the viewer with the new physics state
        viewer.sync()

        # Rough timing to keep it real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)