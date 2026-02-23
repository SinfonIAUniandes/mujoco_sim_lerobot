import mujoco
import mujoco.viewer
import time
import numpy as np
import rerun as rr  # Importar Rerun
import uuid

PACKAGE_PATH: str = "./robotstudio_so101/"
MJCF_FILE: str = "so101_camera_mount.xml"

model = mujoco.MjModel.from_xml_path(PACKAGE_PATH + MJCF_FILE)
data = mujoco.MjData(model)

# Get the internal IDs for the camera and the mount
cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "d435i")
mount_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "air_camera_mount")

# Snap the camera's position and orientation to match the mount perfectly
model.body_pos[cam_id] = model.body_pos[mount_id]
model.body_quat[cam_id] = model.body_quat[mount_id]

mujoco.mj_forward(model, data) 

# --- INICIO SETUP RENDERER ---
camera_name = "realsense_d435i" 
width, height = 640, 480
renderer = mujoco.Renderer(model, height=height, width=width)

render_fps = 30
render_interval = 1.0 / render_fps
last_render_time = time.time()
# --- FIN SETUP RENDERER ---

# Initialize Rerun and spawn the viewer alongside the script

rand_id = str(uuid.uuid4())
rr.init(rand_id, recording_id=rand_id, spawn=True)


rr.log(
    "world/realsense_d435i",
    rr.Pinhole(
        resolution=[width, height],
        focal_length=400, # Approximate focal length
    ), rr.ViewCoordinates.RUB, # Define the coordinate system for the camera. MuJoCo cameras look down -Z, with +X right and +Y up (Right-Up-Back).,
    static=True
)

sim_start_time = time.time()

# Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        real_elapsed_time = time.time() - sim_start_time

        while data.time < real_elapsed_time:
            mujoco.mj_step(model, data)
            
            # Keep your camera snapping math INSIDE the physics loop
            model.body_pos[cam_id] = model.body_pos[mount_id]
            model.body_quat[cam_id] = model.body_quat[mount_id]
            mujoco.mj_forward(model, data) 

        viewer.sync()

        # --- CODIGO DE VISUALIZACION RERUN START ---
        current_time = time.time()
        if current_time - last_render_time >= render_interval:
            
            # Bodies we want the system to be blind to
            IGNORED_BODIES = {"box"}
            
            # 1. Log the TF Tree (Global poses) and kinematic skeleton
            # We start the range at 1 to skip the implicit 'worldbody' (id 0)
            for i in range(1, model.nbody): 
                body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
                
                # Filter out ignored bodies or bodies without names
                if not body_name or body_name in IGNORED_BODIES:
                    continue
                    
                pos = data.xpos[i]
                quat_wxyz = data.xquat[i]
                quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
                
                # Log the coordinate frame (the X/Y/Z axes) for each valid part
                rr.log(f"world/tf/{body_name}", 
                    rr.Transform3D(
                        translation=pos,
                        rotation=rr.Quaternion(xyzw=quat_xyzw)
                    ),
                    rr.TransformAxes3D(0.05) 
                )

                # Draw an arrow from the parent body to this body to visualize the tree structure
                parent_id = model.body_parentid[i]
                
                # Ensure the parent isn't the worldbody (0) so we only draw robot connections
                if parent_id != 0: 
                    parent_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
                    
                    if parent_name and parent_name not in IGNORED_BODIES:
                        parent_pos = data.xpos[parent_id]
                        
                        # Calculate the vector pointing from the parent to the child
                        vector_to_child = pos - parent_pos
                        
                        # Log a 3D arrow connecting them
                        rr.log(f"world/tf_skeleton/{parent_name}_to_{body_name}",
                            rr.Arrows3D(
                                origins=[parent_pos],
                                vectors=[vector_to_child],
                                colors=[[150, 150, 150]], # Gray connecting arrows
                                radii=0.002
                            )
                        )

            # 1.5. Draw arrow from robot base to camera
            base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            # cam_id is already defined at the top of the script
            if base_id != -1 and cam_id != -1:
                base_pos = data.xpos[base_id]
                cam_body_pos = data.xpos[cam_id]
                
                rr.log("world/visuals/base_to_camera",
                    rr.Arrows3D(
                        origins=[base_pos],
                        vectors=[cam_body_pos - base_pos],
                        colors=[[255, 0, 0]], # Red arrow
                        radii=0.005 
                    )
                )
                
            # Add the 3d point cloud from the depth camera to the viewer
            renderer.update_scene(data, camera="realsense_d435i")
            renderer.disable_depth_rendering()
            rgb_image = renderer.render()
            renderer.enable_depth_rendering()
            depth_image = renderer.render()

            # Log the RGB and Depth images
            rr.log("world/realsense_d435i/rgb", rr.Image(rgb_image))
            rr.log("world/realsense_d435i/depth", rr.DepthImage(depth_image, meter=1.0))
            
            # --- UPDATE CAMERA TRANSFORM ---
            # 1. Get the internal ID for the CAMERA object (not the body)
            actual_cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "realsense_d435i")
            
            if actual_cam_id != -1:
                # 2. Extract global position
                cam_pos = data.cam_xpos[actual_cam_id]
                
                # 3. Extract 3x3 rotation matrix and convert to quaternion
                cam_mat = data.cam_xmat[actual_cam_id]
                cam_quat_wxyz = np.zeros(4)
                mujoco.mju_mat2Quat(cam_quat_wxyz, cam_mat)
                
                # Rerun expects xyzw format
                cam_quat_xyzw = [cam_quat_wxyz[1], cam_quat_wxyz[2], cam_quat_wxyz[3], cam_quat_wxyz[0]]
                
                # 4. Log the transform to move the Pinhole (and resulting point cloud)
                rr.log(
                    "world/realsense_d435i",
                    rr.Transform3D(
                        translation=cam_pos,
                        rotation=rr.Quaternion(xyzw=cam_quat_xyzw)
                    )
                )

            
        # --- CODIGO DE VISUALIZACION RERUN END ---