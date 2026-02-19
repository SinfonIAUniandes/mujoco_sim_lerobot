import mujoco
import mujoco.viewer
import time

PACKAGE_PATH: str = "./robotstudio_so101/"

MJCF_FILE: str = "scene.xml" #can be so101.xml, scene.xml or scene_box.xml

model = mujoco.MjModel.from_xml_path(PACKAGE_PATH+MJCF_FILE)
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