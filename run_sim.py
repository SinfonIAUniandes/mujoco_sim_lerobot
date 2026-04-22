from so101_sim import SO101Simulation
import math
import numpy as np # <--- Added numpy for arrays

# Example callback for RGB data
def on_rgb_frame(bgr_image):
    # E.g., pass this to an object detection model
    # print(f"Received RGB Frame with shape: {bgr_image.shape}")
    pass

# Example callback for Depth data
def on_depth_frame(raw_depth, colored_depth):
    # E.g., pass this to a point-cloud generator or obstacle avoidance logic
    # print(f"Received Depth Frame! Center distance: {raw_depth[240, 320]:.2f}m")
    pass

def my_joint_callback(joint_data):
    #print(f"Robot Joints: {joint_data}")
    # E.g. print(f"Base angle: {joint_data['joint1']}")
    pass

# A simple callback that commands the robot based on the simulation time
def my_control_logic(sim_time):
    # Example: Make a joint sweep back and forth using a sine wave
    target_angle = math.sin(sim_time) * 1.5 
    # Return a dictionary of commands. Keys MUST match the actuator names in your XML!
    return {
        'shoulder_pan': target_angle,
        'shoulder_lift': target_angle / 2,
        'elbow_flex': target_angle / 3,
        'wrist_flex': target_angle / 4,
        'wrist_roll': target_angle / 5,
        'gripper': 0.5 + 0.5 * math.sin(sim_time * 2)  # Open and close gripper over time
    }

# --- NEW IK CALLBACK ---
def my_ik_logic(sim_time):
    # Let's make the end-effector draw a continuous circle in the Y/Z plane
    x_pos = 0.3#0.25   Fixed X position
    y_pos = 0#math.sin(sim_time) * 0.1
    z_pos = 0.3#0.2 + math.cos(sim_time) * 0.1
    
    # 1. Target Position: [X, Y, Z]
    target_pos = np.array([x_pos, y_pos, z_pos])
    
    # 2. Target Orientation: [Roll, Pitch, Yaw] in radians
    # We will keep it mostly level, with a slight pitch variation for effect
    target_rpy = np.array([0.0, 0, 0])
    #target_rpy = np.array([0.0, math.sin(sim_time), 0])  # Add some pitch oscillation
    
    # 3. Gripper State (0.0 closed, 1.75 open) - let's make it open and close rhythmically
    gripper_state = 0#0.5 + 0.5 * math.sin(sim_time * 2)

    return {
        "pos": target_pos,
        "rpy": target_rpy,
        "gripper": gripper_state
    }


if __name__ == "__main__":
    PACKAGE_PATH = "./robotstudio_so101/"
    MJCF_FILE = "so101_camera_mount.xml"
    
    sim = SO101Simulation(
        xml_path=PACKAGE_PATH + MJCF_FILE,
        urdf_name="so_arm101_description",
        
        # Starting pose (raw MuJoCo radians)
        starting_angles={
            "shoulder_pan":   0.055,
            "shoulder_lift": -1.676,
            "elbow_flex":     1.571,
            "wrist_flex":     1.153,
            "wrist_roll":     1.619,
            "gripper":        0.0,
        },

        # Stream toggles
        enable_rgb=True,
        enable_depth=True,
        
        # Visualizers
        show_cv2=False,      # Turn off OpenCV since we use Rerun
        
        # Rerun toggles
        enable_rerun=True,    # Enable or disable for external visualization
        rerun_log_meshes=True,  # Show the 3D robot model in Rerun
        rerun_log_tf=True,      # Show the coordinate frames and skeletal arrows
        rerun_depth_mode="pointcloud", # Choose: "none", "depth", or "pointcloud"
        rerun_log_rgb=True,          # Disables the 2D RGB stream in Rerun
        #rgb_callback=on_rgb_frame,
        #depth_callback=on_depth_frame,
        #joint_callback=my_joint_callback,
        #control_callback=my_control_logic
        #ik_callback=my_ik_logic,              # <--- Hook up the new callback
    )
    
    sim.run()