import time
import numpy as np
import pyroki as pk
import viser
import mujoco
import mujoco.viewer
from robot_descriptions.loaders.yourdfpy import load_robot_description as load_urdf_description
from robot_descriptions.loaders.mujoco import load_robot_description as load_mj_description
from viser.extras import ViserUrdf
import pyroki_snippets as pks 

def main():
    # --- 1. SETUP ---
    urdf = load_urdf_description("so_arm101_description")
    robot = pk.Robot.from_urdf(urdf)
    target_link_name = "gripper" 

    viser_server = viser.ViserServer()
    viser_server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(viser_server, urdf, root_node_name="/ghost_robot")
    
    ik_target = viser_server.scene.add_transform_controls(
        "/ik_target", scale=0.1, position=(0.3, 0.0, 0.2), wxyz=(1, 0, 0, 0)
    )

    model = load_mj_description("so_arm101_mj_description")
    data = mujoco.MjData(model)

    # --- 2. AUTOMATIC DISCOVERY (No Manual Dict!) ---
    # We loop through the URDF joints and find the matching MuJoCo actuator.
    map_indices = []
    
    print("\n" + "="*30 + " AUTO-MAPPING " + "="*30)
    urdf_joints = [j.name for j in urdf.actuated_joints]

    for u_idx, name in enumerate(urdf_joints):
        # Ask MuJoCo: "Do you have an actuator named '1'?"
        m_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        
        if m_id != -1:
            map_indices.append((u_idx, m_id))
            print(f"  ✅ Matched: '{name}' (URDF idx {u_idx}) <--> (MuJoCo ID {m_id})")
        else:
            print(f"  ⚠️  Skipping: '{name}' found in URDF but NOT in MuJoCo.")
    print("="*74 + "\n")

    # --- 3. LOOP ---
    with mujoco.viewer.launch_passive(model, data) as mj_viewer:
        while mj_viewer.is_running():
            step_start = time.time()

            # A. SOLVE IK
            q_sol = pks.solve_ik(
                robot=robot,
                target_link_name=target_link_name,
                target_position=np.array(ik_target.position),
                target_wxyz=np.array(ik_target.wxyz),
            )
            
            if q_sol is not None:
                # B. UPDATE GHOST
                urdf_vis.update_cfg(q_sol)

                # C. APPLY TO MUJOCO (Using the auto-discovered indices)
                for u_idx, m_idx in map_indices:
                    data.ctrl[m_idx] = q_sol[u_idx]

            mujoco.mj_step(model, data)
            mj_viewer.sync()
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()