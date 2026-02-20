import mujoco
import mujoco.viewer
import time
import numpy as np
import cv2


PACKAGE_PATH: str = "./robotstudio_so101/"

MJCF_FILE: str = "depth_arm.xml"  # can be so101.xml, scene.xml or scene_box.xml

model = mujoco.MjModel.from_xml_path(PACKAGE_PATH+MJCF_FILE)
data = mujoco.MjData(model)

# Get the internal IDs for the camera and the robot's mount
cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "d435i")
# Ensure this matches the name in so101.xml
mount_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "camera_mount")

# Setup the offscreen renderer
camera_name = "realsense_d435i"
width, height = 640, 480
renderer = mujoco.Renderer(model, height=height, width=width)

# Frame timing for visualization (30 FPS)
render_fps = 30
render_interval = 1.0 / render_fps
last_render_time = time.time()

# --- NEW: Define Camera Offsets in Degrees ---
# Local position offset [X, Y, Z] in meters relative to the mount
local_pos_offset = np.array([0.05, 0.0, 0.05]) 

# Local rotation offset in degrees [Roll (X), Pitch (Y), Yaw (Z)]
# Tweak these numbers to point the camera exactly where you want
local_euler_degrees = np.array([0.0, 180.0, 90.0]) 

# 1. Convert those degrees into radians
local_euler_radians = np.radians(local_euler_degrees)

# 2. Create an empty array for the quaternion
local_quat_offset = np.zeros(4)

# 3. Use MuJoCo's built-in math engine to convert the Euler radians to a Quaternion
mujoco.mju_euler2Quat(local_quat_offset, local_euler_radians, "xyz")
# ---------------------------------------------

# Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():
        step_start = time.time()

        # 1. Step the physics forward
        mujoco.mj_step(model, data)

        # --- NEW: Offset Snapping Logic ---
        # Get the mount's 3x3 rotation matrix
        mount_mat = data.xmat[mount_id].reshape(3, 3)

        # Multiply the local offset by the rotation matrix to get the global offset vector
        global_offset = mount_mat @ local_pos_offset

        # Apply the offset to the mount's position
        model.body_pos[cam_id] = data.xpos[mount_id] + global_offset

        # Multiply the mount's quaternion by our offset quaternion to rotate the camera
        mujoco.mju_mulQuat(
            model.body_quat[cam_id], data.xquat[mount_id], local_quat_offset)

        # Update the kinematic tree so MuJoCo knows the camera moved before rendering
        mujoco.mj_kinematics(model, data)
        # -----------------------------------

        viewer.sync()

        # Render RGB and Depth at specified FPS
        current_time = time.time()
        if current_time - last_render_time >= render_interval:

            # Update and Render RGB
            renderer.update_scene(data, camera=camera_name)
            renderer.disable_depth_rendering()
            rgb_image = renderer.render()
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            # Update and Render Depth
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera_name)
            depth_image = renderer.render()

            # Robust Depth Normalization
            max_depth = 3.0
            depth_visual = depth_image.copy()

            # Fix Background
            bg_mask = (depth_visual == 0.0) | np.isinf(
                depth_visual) | np.isnan(depth_visual)
            depth_visual[bg_mask] = max_depth

            depth_normalized = np.clip(depth_visual, 0, max_depth) / max_depth
            depth_8bit = (depth_normalized * 255).astype(np.uint8)
            depth_8bit = 255 - depth_8bit

            depth_colormap = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

            # Show the images
            cv2.imshow("RGB Camera", bgr_image)
            cv2.imshow("Depth Camera", depth_colormap)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            last_render_time = current_time

        # Rough timing to keep it real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

cv2.destroyAllWindows()
