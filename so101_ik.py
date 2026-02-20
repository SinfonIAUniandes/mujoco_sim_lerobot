import time
import numpy as np
import cv2
import mujoco
import mujoco.viewer

# Pyroki imports
import pyroki as pk
from robot_descriptions.loaders.yourdfpy import load_robot_description as load_urdf_description
import pyroki_snippets as pks 

def main():
    # ==========================================
    # 1. SETUP KINEMATICS (Pyroki)
    # ==========================================
    urdf = load_urdf_description("so_arm101_description")
    robot = pk.Robot.from_urdf(urdf)
    target_link_name = "gripper" 

    # --- Define your sequence of targets here! ---
    target_pos_1 = np.array([0.3, 0, 0.2]) 
    target_quat_1 = np.zeros(4)
    mujoco.mju_euler2Quat(target_quat_1, np.radians([0, 0, 0]), "xyz")

    target_pos_2 = np.array([0.196, -0.171, 0.146]) 
    target_quat_2 = np.zeros(4)
    mujoco.mju_euler2Quat(target_quat_2, np.radians([0, 0, 0]), "xyz")
    
    # Define gripper states (You may need to tweak these depending on your URDF limits)
    # Usually 0.0 is closed and a positive/negative number is open (or vice versa)
    GRIPPER_OPEN = 1.75
    GRIPPER_CLOSED = -0.1 

    # Put them in a list so we can sequence through them
    # Format: (Position, Quaternion, Gripper_State)
    targets = [
        # 1. Go to target 1 with gripper open
        (target_pos_1, target_quat_1, GRIPPER_OPEN),
        
        # 2. Go to target 2 with gripper open
        (target_pos_2, target_quat_2, GRIPPER_OPEN),
        
        # 3. Stay at target 2, but CLOSE the gripper (Grab!)
        (target_pos_2, target_quat_2, GRIPPER_CLOSED),
        
        # 4. Go back to target 1, keeping gripper closed (Carry!)
        (target_pos_1, target_quat_1, GRIPPER_CLOSED),
        
        # 5. Open (Drop!)
        (target_pos_1, target_quat_1, GRIPPER_OPEN),
        # 6. Go Back
        (target_pos_2, target_quat_2, GRIPPER_OPEN)
    ]
    current_target_idx = 0
    
    # Set the initial active target
    target_pos, target_quat, target_gripper = targets[current_target_idx]
    # ---------------------------------------------

    # ==========================================
    # 2. SETUP PHYSICS (MuJoCo)
    # ==========================================
    PACKAGE_PATH: str = "./robotstudio_so101/"
    MJCF_FILE: str = "depth_arm.xml"  
    
    model = mujoco.MjModel.from_xml_path(PACKAGE_PATH + MJCF_FILE)
    data = mujoco.MjData(model)

    JOINT_MAPPING = {
        "1": "shoulder_pan",
        "2": "shoulder_lift",
        "3": "elbow_flex",
        "4": "wrist_flex",
        "5": "wrist_roll",
        "6": "gripper",
    }
    
    map_indices = []
    print("\n" + "="*30 + " MANUAL MAPPING " + "="*30)
    urdf_joints = [j.name for j in urdf.actuated_joints]
    
    for u_idx, u_name in enumerate(urdf_joints):
        if u_name in JOINT_MAPPING:
            m_name = JOINT_MAPPING[u_name]
            m_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, m_name)
            
            if m_id != -1:
                map_indices.append((u_idx, m_id))
                print(f"  ✅ Mapped: URDF '{u_name}' -> MuJoCo '{m_name}' (ID {m_id})")
            else:
                print(f"  ❌ Error: Actuator '{m_name}' not found in MuJoCo model!")
        else:
            print(f"  ⚠️ Warning: URDF joint '{u_name}' is missing from JOINT_MAPPING dictionary.")
    print("="*74 + "\n")

    # ==========================================
    # 3. SETUP CAMERA SENSOR
    # ==========================================
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "d435i")
    mount_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "camera_mount")

    camera_name = "realsense_d435i"
    width, height = 640, 480
    renderer = mujoco.Renderer(model, height=height, width=width)

    local_pos_offset = np.array([0.0, 0.04, -0.07])
    local_euler_degrees = np.array([-45.0, 180.0, 0.0])
    local_euler_radians = np.radians(local_euler_degrees)
    local_quat_offset = np.zeros(4)
    mujoco.mju_euler2Quat(local_quat_offset, local_euler_radians, "xyz")

    render_fps = 30
    render_interval = 1.0 / render_fps
    last_render_time = time.time()

    # ==========================================
    # 4. MAIN SIMULATION LOOP
    # ==========================================
    print(f"🚀 Starting simulation. Moving to Step {current_target_idx + 1}.")
    sim_start_time = time.time()
    last_command_time = time.time()

    with mujoco.viewer.launch_passive(model, data) as mj_viewer:
        while mj_viewer.is_running():
            
            # --- A. HIGH-LEVEL CONTROL ---
            q_sol = pks.solve_ik(
                robot=robot,
                target_link_name=target_link_name,
                target_position=target_pos,
                target_wxyz=target_quat,
            )
            
            if q_sol is not None:
                for u_idx, m_idx in map_indices:
                    u_name = urdf_joints[u_idx]
                    
                    # Intercept the gripper command and use our sequence target instead!
                    if u_name == "6": 
                        data.ctrl[m_idx] = target_gripper
                    else:
                        data.ctrl[m_idx] = q_sol[u_idx]

            # --- B. LOW-LEVEL PHYSICS ---
            real_elapsed_time = time.time() - sim_start_time
            while data.time < real_elapsed_time:
                mujoco.mj_step(model, data)
                
                # Snap camera
                mount_mat = data.xmat[mount_id].reshape(3, 3)
                global_offset = mount_mat @ local_pos_offset
                model.body_pos[cam_id] = data.xpos[mount_id] + global_offset
                mujoco.mju_mulQuat(model.body_quat[cam_id], data.xquat[mount_id], local_quat_offset)
                mujoco.mj_kinematics(model, data)

            mj_viewer.sync()

            # --- C. CHECK IF ARRIVED ---
            if (time.time() - last_command_time) > 1.0:
                vel_norm = np.linalg.norm(data.qvel)
                if vel_norm < 0.05:  
                    
                    if current_target_idx < len(targets) - 1:
                        print(f"✅ Finished Step {current_target_idx + 1}!")
                        
                        current_target_idx += 1
                        target_pos, target_quat, target_gripper = targets[current_target_idx]
                        print(f"🚀 Starting Step {current_target_idx + 1}...")
                        
                        last_command_time = time.time()

            # --- D. SENSOR RENDERING ---
            current_time = time.time()
            if current_time - last_render_time >= render_interval:
                
                renderer.update_scene(data, camera=camera_name)
                renderer.disable_depth_rendering()
                rgb_image = renderer.render()
                bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

                renderer.enable_depth_rendering()
                renderer.update_scene(data, camera=camera_name)
                depth_image = renderer.render()

                max_depth = 3.0
                depth_visual = depth_image.copy()
                bg_mask = (depth_visual == 0.0) | np.isinf(depth_visual) | np.isnan(depth_visual)
                depth_visual[bg_mask] = max_depth

                depth_normalized = np.clip(depth_visual, 0, max_depth) / max_depth
                depth_8bit = (depth_normalized * 255).astype(np.uint8)
                depth_8bit = 255 - depth_8bit
                depth_colormap = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

                cv2.imshow("RGB Camera", bgr_image)
                cv2.imshow("Depth Camera", depth_colormap)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                last_render_time = current_time

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()