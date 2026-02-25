import time
import json
import numpy as np
import cv2
import mujoco
from threading import Thread
from so101_sim import SO101Simulation

class GreenBoxGrabber:
    def __init__(self):
        self.latest_bgr = None
        self.latest_depth = None
        self.target_world_pos = None
        self.box_found = False  # Track if the control thread can start
        
        self.fx = 400.0
        self.fy = 400.0
        self.cx = 320.0
        self.cy = 240.0
        
        try:
            with open("camera_calib.json", "r") as f:
                calib = json.load(f)
            self.cam_pos = np.array(calib["cam_pos"])
            self.cam_mat = np.array(calib["cam_mat"])
        except FileNotFoundError:
            print("ERROR: camera_calib.json not found. Run calibrate_camera.py first!")
            exit()
        
        self.gripper_open = 1.0
        self.gripper_closed = 0.0

    def on_depth_frame(self, raw_depth, colored_depth):
        self.latest_depth = raw_depth

    def on_rgb_frame(self, bgr_image):
        self.latest_bgr = bgr_image.copy()
        # By calling process_vision inside the callback, it runs on the MAIN thread,
        # perfectly in sync with the simulation rendering.
        self.process_vision()

    def process_vision(self):
        """Detects the box and draws the CV2 window."""
        if self.latest_bgr is None:
            return

        vis_frame = self.latest_bgr.copy()
        
        if self.latest_depth is not None:
            hsv = cv2.cvtColor(self.latest_bgr, cv2.COLOR_BGR2HSV)
            lower_green = np.array([40, 50, 50])
            upper_green = np.array([80, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) >= 50:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                        z = self.latest_depth[v, u]
                        
                        if z > 0.0 and not np.isnan(z) and not np.isinf(z):
                            x_local = (u - self.cx) * z / self.fx
                            y_local = -(v - self.cy) * z / self.fy
                            z_local = -z
                            pos_local = np.array([x_local, y_local, z_local])

                            self.target_world_pos = self.cam_pos + self.cam_mat.dot(pos_local)
                            
                            x, y, w, h = cv2.boundingRect(largest_contour)
                            cv2.rectangle(vis_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.drawMarker(vis_frame, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                            coord_str = f"WORLD XYZ: [{self.target_world_pos[0]:.3f}, {self.target_world_pos[1]:.3f}, {self.target_world_pos[2]:.3f}]"
                            cv2.putText(vis_frame, coord_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            
                            self.box_found = True

        cv2.imshow("Detection Diagnostics", vis_frame)
        cv2.waitKey(1)

# --- NEW BACKGROUND THREAD FUNCTION ---
def robot_control_sequence(grabber, sim):
    """Runs in a background thread so we can use time.sleep() without freezing the sim."""
    print("Waiting for cameras and target...")
    while not grabber.box_found:
        time.sleep(0.1)

    # 1. Get target in MuJoCo World Coordinates
    mujoco_target = grabber.target_world_pos
    
    # 2. Transform to Pyroki/URDF Coordinates (Inverse of the +90 deg Z rotation)
    ik_target = np.array([
        mujoco_target[1],   # X_ik = Y_mujoco
        -mujoco_target[0],  # Y_ik = -X_mujoco
        mujoco_target[2]    # Z_ik = Z_mujoco
    ])
    
    target_rpy = np.array([0.0, 0.0, 0.0]) 
    
    print(f"MuJoCo Detected Target: {mujoco_target}")
    print(f"Pyroki IK Target: {ik_target}")
    print("Executing procedural grab sequence...")

    # Note: We now use ik_target for all positioning commands
    print("-> Hovering")
    sim.apply_ik({"pos": ik_target + np.array([0.0, 0, 0.4]), "rpy": target_rpy, "gripper": grabber.gripper_open})
    time.sleep(3.0) 

    print("-> Descending")
    sim.apply_ik({"pos": ik_target + np.array([0.0, 0, 0.12]), "rpy": target_rpy, "gripper": grabber.gripper_open})
    time.sleep(2.0)

    print("-> Grabbing")
    sim.apply_ik({"pos": ik_target + np.array([0.0, 0, 0.12]), "rpy": target_rpy, "gripper": grabber.gripper_closed})
    time.sleep(1.0)

    print("-> Lifting")
    sim.apply_ik({"pos": ik_target + np.array([0.0, 0, 0.4]), "rpy": target_rpy, "gripper": grabber.gripper_closed})
    
    print("Sequence Complete!")


if __name__ == "__main__":
    PACKAGE_PATH = "./robotstudio_so101/"
    MJCF_FILE = "so101_camera_mount.xml"
    
    grabber = GreenBoxGrabber()
    
    sim = SO101Simulation(
        xml_path=PACKAGE_PATH + MJCF_FILE,
        urdf_name="so_arm101_description",    
        ik_target_link="gripper",             
        use_ik_web=False,                     
        enable_rgb=True,
        enable_depth=True,
        show_cv2=False, # Keep false to prevent so101_sim from launching double-windows
        enable_rerun=True,    
        rerun_log_meshes=False,  
        rerun_log_tf=True,      
        rerun_depth_mode="none",
        rerun_log_rgb=False,          
        rgb_callback=grabber.on_rgb_frame,
        depth_callback=grabber.on_depth_frame,
    )
    
    # 1. Start the CONTROL sequence in a background thread
    control_thread = Thread(target=robot_control_sequence, args=(grabber, sim))
    control_thread.daemon = True 
    control_thread.start()

    # Para cambiar las coordenadas de la caja, edita el archivo so101_camera_mount.xml y modifica la línea del cuerpo de la caja:
    #    <body name="box" pos="0.2 -0.2 0.03">
    #    <body name="box" pos="0.2 0.2 0.03">
    #    <body name="box" pos="0.31 0.05 0.03">

    # 2. Run the SIMULATION (and rendering) in the main thread
    sim.run()
    