import time
import numpy as np
import cv2
import mujoco
import mujoco.viewer

# Pyroki & Viser imports
import pyroki as pk
import viser
from robot_descriptions.loaders.yourdfpy import load_robot_description as load_urdf_description
from viser.extras import ViserUrdf
import pyroki_snippets as pks

def main():
    # ==========================================
    # 1. SETUP KINEMATICS (Pyroki & Viser)
    # ==========================================
    # We use the standard URDF just for the mathematical IK solver
    urdf = load_urdf_description("so_arm101_description")
    robot = pk.Robot.from_urdf(urdf)
    target_link_name = "gripper"

    # Start the web interface
    viser_server = viser.ViserServer()
    viser_server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(viser_server, urdf, root_node_name="/ghost_robot")

    # Add the interactive 3D drag handle
    ik_target = viser_server.scene.add_transform_controls(
        "/ik_target", scale=0.1, position=(0.3, 0.0, 0.2), wxyz=(1, 0, 0, 0)
    )

    # ==========================================
    # 2. SETUP PHYSICS (MuJoCo)
    # ==========================================
    # Load your custom combined scene for the actual physics simulation
    PACKAGE_PATH: str = "./robotstudio_so101/"  # Adjust if your path is different
    MJCF_FILE: str = "depth_arm.xml"

    model = mujoco.MjModel.from_xml_path(PACKAGE_PATH + MJCF_FILE)
    data = mujoco.MjData(model)

    # ==========================================
    # 2. MANUAL KINEMATIC MAPPING
    # ==========================================
    # Map the URDF joint names to the exact MuJoCo actuator names defined in so101.xml
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
            # Look up the ID using the MuJoCo name
            m_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, m_name)

            if m_id != -1:
                map_indices.append((u_idx, m_id))
                print(
                    f"  ✅ Mapped: URDF '{u_name}' -> MuJoCo '{m_name}' (ID {m_id})")
            else:
                print(
                    f"  ❌ Error: Actuator '{m_name}' not found in MuJoCo model!")
        else:
            print(
                f"  ⚠️ Warning: URDF joint '{u_name}' is missing from JOINT_MAPPING dictionary.")
    print("="*74 + "\n")

    # ==========================================
    # 3. SETUP CAMERA SENSOR
    # ==========================================
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "d435i")
    mount_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "camera_mount")

    camera_name = "realsense_d435i"
    width, height = 640, 480
    renderer = mujoco.Renderer(model, height=height, width=width)

    # Define camera offsets
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
    print("🚀 Starting simulation. Open http://localhost:8080 in your browser for Viser.")
    sim_start_time = time.time()

    with mujoco.viewer.launch_passive(model, data) as mj_viewer:
        while mj_viewer.is_running():

            # --- A. HIGH-LEVEL CONTROL (Viser -> IK -> MuJoCo Ctrl) ---
            # Solve IK based on the web interface target
            q_sol = pks.solve_ik(
                robot=robot,
                target_link_name=target_link_name,
                target_position=np.array(ik_target.position),
                target_wxyz=np.array(ik_target.wxyz),
            )

            if q_sol is not None:
                # Update web visualizer ghost
                urdf_vis.update_cfg(q_sol)
                # Send joint targets to MuJoCo motors
                for u_idx, m_idx in map_indices:
                    data.ctrl[m_idx] = q_sol[u_idx]

            # --- B. LOW-LEVEL PHYSICS (Catch-up Loop) ---
            real_elapsed_time = time.time() - sim_start_time
            while data.time < real_elapsed_time:
                # Step physics by 2ms
                mujoco.mj_step(model, data)

                # Snap camera to moving arm inside the fast-forward loop
                mount_mat = data.xmat[mount_id].reshape(3, 3)
                global_offset = mount_mat @ local_pos_offset
                model.body_pos[cam_id] = data.xpos[mount_id] + global_offset
                mujoco.mju_mulQuat(
                    model.body_quat[cam_id], data.xquat[mount_id], local_quat_offset)
                mujoco.mj_kinematics(model, data)

            # Sync desktop viewer
            mj_viewer.sync()

            # --- C. SENSOR RENDERING (Throttled to 30 FPS) ---
            current_time = time.time()
            if current_time - last_render_time >= render_interval:

                # RGB Render
                renderer.update_scene(data, camera=camera_name)
                renderer.disable_depth_rendering()
                rgb_image = renderer.render()
                bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

                # Depth Render
                renderer.enable_depth_rendering()
                renderer.update_scene(data, camera=camera_name)
                depth_image = renderer.render()

                # Depth Normalization
                max_depth = 3.0
                depth_visual = depth_image.copy()
                bg_mask = (depth_visual == 0.0) | np.isinf(
                    depth_visual) | np.isnan(depth_visual)
                depth_visual[bg_mask] = max_depth

                depth_normalized = np.clip(
                    depth_visual, 0, max_depth) / max_depth
                depth_8bit = (depth_normalized * 255).astype(np.uint8)
                depth_8bit = 255 - depth_8bit
                depth_colormap = cv2.applyColorMap(
                    depth_8bit, cv2.COLORMAP_JET)

                # Display
                cv2.imshow("RGB Camera", bgr_image)
                cv2.imshow("Depth Camera", depth_colormap)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                last_render_time = current_time

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
