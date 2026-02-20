import mujoco
import mujoco.viewer
import time
import numpy as np
import open3d as o3d
import cv2 

PACKAGE_PATH: str = "./robotstudio_so101/realsense_d435i/"
MJCF_FILE: str = "scene_box.xml" 

model = mujoco.MjModel.from_xml_path(PACKAGE_PATH+MJCF_FILE)

# Get the internal IDs for the camera and the mount
cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "d435i")
mount_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "camera_mount")

# Snap the camera's position and orientation to match the mount perfectly
model.body_pos[cam_id] = model.body_pos[mount_id]
model.body_quat[cam_id] = model.body_quat[mount_id]

data = mujoco.MjData(model)
mujoco.mj_forward(model, data) 

# Setup the offscreen renderer
camera_name = "realsense_d435i"
width, height = 640, 480
renderer = mujoco.Renderer(model, height=height, width=width)

# Frame timing for visualization (30 FPS)
render_fps = 30
render_interval = 1.0 / render_fps
last_render_time = time.time()

# Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()

        # Step the physics
        mujoco.mj_step(model, data)
        viewer.sync()

        # Render RGB and Depth at specified FPS
        current_time = time.time()
        if current_time - last_render_time >= render_interval:
            
            # 1. Update and Render RGB
            renderer.update_scene(data, camera=camera_name)
            renderer.disable_depth_rendering()
            rgb_image = renderer.render()
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            
            # 2. Update and Render Depth
            renderer.enable_depth_rendering()
            # Force scene update again so the depth flags correctly apply
            renderer.update_scene(data, camera=camera_name) 
            depth_image = renderer.render() 
            
            # 3. Robust Depth Normalization
            max_depth = 3.0 
            depth_visual = depth_image.copy()
            
            # Fix Background: MuJoCo sets empty space to 0.0 or inf. 
            # We force it to max_depth so it correctly registers as "far away".
            bg_mask = (depth_visual == 0.0) | np.isinf(depth_visual) | np.isnan(depth_visual)
            depth_visual[bg_mask] = max_depth
            
            # Clip and normalize
            depth_normalized = np.clip(depth_visual, 0, max_depth) / max_depth
            depth_8bit = (depth_normalized * 255).astype(np.uint8)
            
            # Invert the depth map: Make close objects warm (bright) and far objects cold (dark)
            depth_8bit = 255 - depth_8bit
            
            # Apply color map
            depth_colormap = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

            # 4. Show the images
            cv2.imshow("RGB Camera", bgr_image)
            cv2.imshow("Depth Camera", depth_colormap)
            
            # Press 'q' in the OpenCV window to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            last_render_time = current_time

        # Rough timing to keep it real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

cv2.destroyAllWindows()