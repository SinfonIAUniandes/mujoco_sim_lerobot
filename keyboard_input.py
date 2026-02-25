from so101_sim import SO101Simulation
from threading import Thread
import numpy as np
import time

if __name__ == "__main__":
    PACKAGE_PATH = "./robotstudio_so101/"
    MJCF_FILE = "so101_camera_mount.xml"
    
    sim = SO101Simulation(
        xml_path=PACKAGE_PATH + MJCF_FILE,
        urdf_name="so_arm101_description",    # <--- Your URDF package name
        ik_target_link="gripper",             # <--- The link you want to control
        
        # Rerun toggles
        enable_rerun=True,    # Enable or disable for external visualization
        rerun_log_meshes=False,  # Show the 3D robot model in Rerun
        rerun_log_tf=True,      # Show the coordinate frames and skeletal arrows
        rerun_log_rgb=True,          # Disables the 2D RGB stream in Rerun
    )
    
    # Run the simulation in a separate thread to allow for real-time interaction
    sim_thread = Thread(target=sim.run)
    sim_thread.start()

    while True:
        # Get user input for target position and orientation
        # Let's make the end-effector draw a continuous circle in the Y/Z plane
        x_pos = float(input("Enter X position (e.g., 0.3): "))  # Get X position from user input
        y_pos = float(input("Enter Y position (e.g., 0): ")) # Get Y position from user input
        z_pos = float(input("Enter Z position (e.g., 0.3): "))
        
        # 1. Target Position: [X, Y, Z]
        target_pos = np.array([x_pos, y_pos, z_pos])
        
        # 2. Target Orientation: [Roll, Pitch, Yaw] in radians
        # We will keep it mostly level, with a slight pitch variation for effect
        roll = float(input("Enter Roll (in degrees, e.g., 0): "))
        pitch = float(input("Enter Pitch (in degrees, e.g., 0): "))
        yaw = float(input("Enter Yaw (in degrees, e.g., 0): "))

        roll = np.deg2rad(roll)  # Convert to radians
        pitch = np.deg2rad(pitch)  # Convert to radians
        yaw = np.deg2rad(yaw)  # Convert to radians

        target_rpy = np.array([roll, pitch, yaw])
    #target_rpy = np.array([0.0, math.sin(sim_time), 0])  # Add some pitch oscillation
        gripper_state = float(input("Enter Gripper state (0.0 closed, 1.75 open): "))


        sim.apply_ik({
            "pos": target_pos,
            "rpy": target_rpy,
            "gripper": gripper_state
        })

    sim_thread.join()