import time
import cv2
import numpy as np
import torch
from PIL import Image
from threading import Thread

# Simulation imports
import mujoco
from so101_sim import SO101Simulation

# LeRobot imports
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

class SmolVLAController:
    def __init__(self, model_id="lerobot/smolvla_base", task_prompt="pick up the green box"):
        self.latest_bgr = None
        self.task_prompt = task_prompt
        
        # 1. Initialize Device & Model
        print(f"Loading VLA policy: {model_id}...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy = SmolVLAPolicy.from_pretrained(model_id).to(self.device).eval()
        
        # 2. Setup Processors
        self.preprocess, self.postprocess = make_pre_post_processors(
            self.policy.config,
            model_id,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )
        print("Model loaded successfully.")
        
        # Add to the end of your __init__ method:
        self.latest_joint_commands = None

    # Add this new method to the class:
    def control_callback(self, sim_time):
        """Simulator calls this every physics step to get the latest joint targets."""
        if self.latest_joint_commands is not None:
            return self.latest_joint_commands
        return {} # Return empty dict if no action is ready yet

    def on_rgb_frame(self, bgr_image):
        """Callback triggered by the simulation rendering loop."""
        self.latest_bgr = bgr_image.copy()
        
        # Optional: Show what the robot sees
        cv2.imshow("Robot View", self.latest_bgr)
        cv2.waitKey(1)

    def get_vla_frame(self):
        """Formats the latest OpenCV image for the LeRobot policy."""
        if self.latest_bgr is None:
            return None
        
        # Convert BGR (OpenCV) to RGB (Standard)
        rgb_image = cv2.cvtColor(self.latest_bgr, cv2.COLOR_BGR2RGB)
        
        # Resize to the 256x256 resolution expected by smolvla_base
        rgb_image = cv2.resize(rgb_image, (256, 256))
        
        # Convert to Normalized Tensor (b, c, h, w)
        img_tensor = torch.from_numpy(rgb_image).permute(2, 0, 1).contiguous()
        img_tensor = img_tensor.float() / 255.0
        img_tensor = img_tensor.unsqueeze(0)
        
        # CRITICAL FIX: Add the robot's current state (Proprioception)
        # smolvla_base expects a 6-dimensional state vector [1, 6]
        # We are using a dummy state of zeros just to get the model to run.
        state_tensor = torch.zeros((1, 6), dtype=torch.float32)
        
        frame = {
            "observation.images.camera1": img_tensor,
            "observation.state": state_tensor,
            "task": self.task_prompt
        }
        return frame


def vla_control_loop(controller, sim):
    """Runs continuously in a background thread to predict and apply actions."""
    print("Waiting for camera feed...")
    while controller.latest_bgr is None:
        time.sleep(0.1)
        
    print(f"Starting VLA Inference Loop. Task: '{controller.task_prompt}'")
    
    # Control loop frequency (e.g., 10 Hz)
    rate_hz = 10.0
    sleep_time = 1.0 / rate_hz

    while True:
        start_time = time.time()
        
        # 1. Grab and format the current frame
        frame = controller.get_vla_frame()
        if frame is None:
            continue

        # 2. Run Inference
        batch = controller.preprocess(frame)
        with torch.inference_mode():
            pred_action = controller.policy.select_action(batch)
            pred_action = controller.postprocess(pred_action)

        print(f"Raw VLA Output: {pred_action.cpu().numpy().squeeze()}")
        
        # 3. Translate Actions (CRITICAL STEP)
        # -------------------------------------------------------------------
        # VLA models output tensors. You must map these values to your robot.
        # Assuming a standard 7D output: [X, Y, Z, Roll, Pitch, Yaw, Gripper]
        # You will need to print pred_action and scale/slice it to match your IK.
        # -------------------------------------------------------------------
        
        action_np = pred_action.cpu().numpy().squeeze()
        
        try:
            # Map the 6 raw outputs directly to your robot's joints
            controller.latest_joint_commands = {
                'shoulder_pan': action_np[5],
                'shoulder_lift': action_np[4],
                'elbow_flex': action_np[3],
                'wrist_flex': action_np[2],
                'wrist_roll': action_np[1],
                'gripper': action_np[0]  # Might need scaling depending on your gripper limits
            }
            
            # Print to verify
            print(f"Joints updated: {controller.latest_joint_commands}")
            
        except IndexError:
            print("Action dimension mismatch! Check pred_action shape.")
            print(f"Raw action: {action_np}")
            break
            
        # Maintain loop frequency
        elapsed = time.time() - start_time
        if elapsed < sleep_time:
            time.sleep(sleep_time - elapsed)

if __name__ == "__main__":
    PACKAGE_PATH = "./robotstudio_so101/"
    MJCF_FILE = "so101_camera_mount.xml"
    
    # Initialize the VLA controller
    vla_controller = SmolVLAController(
        model_id="lerobot/smolvla_base", 
        task_prompt="pick up the green box"
    )
    
    # Initialize Simulation
    sim = SO101Simulation(
        xml_path=PACKAGE_PATH + MJCF_FILE,
        enable_rgb=True,
        enable_depth=False, # Disabled unless your specific VLA model uses depth
        show_cv2=False, 
        enable_rerun=False,    
        rerun_log_meshes=False,  
        rerun_log_tf=False,      
        rerun_depth_mode="none",
        rerun_log_rgb=False,          
        rgb_callback=vla_controller.on_rgb_frame,
        control_callback=vla_controller.control_callback
    )
    
    # Start the Control sequence in a background thread
    control_thread = Thread(target=vla_control_loop, args=(vla_controller, sim))
    control_thread.daemon = True 
    control_thread.start()

    # Run the Simulation (and rendering) in the main thread
    sim.run()