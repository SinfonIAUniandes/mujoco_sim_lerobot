from so101_sim import SO101Simulation

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


if __name__ == "__main__":
    PACKAGE_PATH = "./robotstudio_so101/"
    MJCF_FILE = "so101_camera_mount.xml"
    
    sim = SO101Simulation(
        xml_path=PACKAGE_PATH + MJCF_FILE,
        
        # Stream toggles
        enable_rgb=True,
        enable_depth=True,
        
        # Visualizers
        show_cv2=True,      # Turn off OpenCV since we use Rerun
        
        # Rerun toggles
        enable_rerun=False,    # Enable or disable for external visualization
        rerun_log_meshes=True,  # Show the 3D robot model in Rerun
        rerun_log_tf=True,      # Show the coordinate frames and skeletal arrows
        rerun_depth_mode="pointcloud", # Choose: "none", "depth", or "pointcloud"
        rerun_log_rgb=True,          # Disables the 2D RGB stream in Rerun
        rgb_callback=on_rgb_frame,
        depth_callback=on_depth_frame
    )
    
    sim.run()