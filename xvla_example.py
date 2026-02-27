import time
import cv2
import numpy as np
import torch
from threading import Thread

# Simulation imports
import mujoco
from so101_sim import SO101Simulation

# LeRobot imports
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.xvla.modeling_xvla import XVLAPolicy

class XVLAController:
    def __init__(self, model_id="lerobot/xvla-base", task_prompt="pick up the green box"):
        self.latest_bgr = None
        self.task_prompt = task_prompt
        
        # 1. Initialize Device & Model
        print(f"Loading X-VLA policy: {model_id}...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Swapped to XVLAPolicy
        self.policy = XVLAPolicy.from_pretrained(model_id).to(self.device).eval()
        
        # 2. Setup Processors
        self.preprocess, self.postprocess = make_pre_post_processors(
            self.policy.config,
            model_id,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )
        print("Model loaded successfully.")
        
        self.current_joints = np.zeros(6, dtype=np.float32)
        self.latest_joint_commands = None

    def control_callback(self, sim_time):
        """Simulator calls this every physics step to get the latest joint targets."""
        if self.latest_joint_commands is not None:
            return self.latest_joint_commands
        return {} 

    def on_rgb_frame(self, bgr_image):
        """Callback triggered by the simulation rendering loop."""
        self.latest_bgr = bgr_image.copy()

    def my_joint_callback(self,joint_data):
        #print(f"Robot Joints: {joint_data}")
        # E.g. print(f"Base angle: {joint_data['joint1']}")
        self.current_joints = np.array([
            joint_data['shoulder_pan'],
            joint_data['shoulder_lift'],
            joint_data['elbow_flex'],
            joint_data['wrist_flex'],
            joint_data['wrist_roll'],
            joint_data['gripper']
        ], dtype=np.float32)

    def get_vla_frame(self, sim):
        """Formats the latest OpenCV image and joint states for the LeRobot policy."""
        if self.latest_bgr is None:
            return None
        
        # Vision
        rgb_image = cv2.cvtColor(self.latest_bgr, cv2.COLOR_BGR2RGB)
        rgb_image = cv2.resize(rgb_image, (256, 256)) 
        
        img_tensor = torch.from_numpy(rgb_image).permute(2, 0, 1).contiguous()
        img_tensor = img_tensor.float() / 255.0
        img_tensor = img_tensor.unsqueeze(0)
        
        state_tensor = torch.from_numpy(self.current_joints).unsqueeze(0)
        
        # CRITICAL FIX: Changed 'camera1' to 'image'
        frame = {
            "observation.images.image": img_tensor,
            "observation.state": state_tensor,
            "task": self.task_prompt
        }
        return frame


def vla_control_loop(controller, sim):
    """Runs continuously in a background thread to predict and apply actions."""
    print("Waiting for camera feed...")
    while controller.latest_bgr is None:
        time.sleep(0.1)
        
    print(f"Starting X-VLA Inference Loop. Task: '{controller.task_prompt}'")
    
    rate_hz = 10.0
    sleep_time = 1.0 / rate_hz

    while True:
        start_time = time.time()
        
        # 1. Grab frame WITH sim reference for joint states
        frame = controller.get_vla_frame(sim)
        if frame is None:
            continue

        # 2. Run Inference
        batch = controller.preprocess(frame)
        with torch.inference_mode():
            pred_action = controller.policy.select_action(batch)
            pred_action = controller.postprocess(pred_action)

        # print(f"Raw VLA Output: {pred_action.cpu().numpy().squeeze()}")
        
        # 3. Translate Actions
        action_np = pred_action.cpu().numpy().squeeze()
        
        try:
            # Map the raw outputs directly to your robot's joints
            controller.latest_joint_commands = {
                'shoulder_pan': action_np[5],
                'shoulder_lift': action_np[4],
                'elbow_flex': action_np[3],
                'wrist_flex': action_np[2],
                'wrist_roll': action_np[1],
                'gripper': action_np[0] 
            }
            
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
    
    # Initialize the X-VLA controller
    vla_controller = XVLAController(
        model_id="lerobot/xvla-base", 
        task_prompt="pick up the green box"
    )
    
    # Initialize Simulation
    sim = SO101Simulation(
        xml_path=PACKAGE_PATH + MJCF_FILE,
        enable_rgb=True,
        enable_depth=False, 
        show_cv2=False, 
        enable_rerun=False,    
        rerun_log_meshes=False,  
        rerun_log_tf=False,      
        rerun_depth_mode="none",
        rerun_log_rgb=False,          
        rgb_callback=vla_controller.on_rgb_frame,
        control_callback=vla_controller.control_callback # Now the sim will actually read the commands
    )
    
    # Start the Control sequence in a background thread
    control_thread = Thread(target=vla_control_loop, args=(vla_controller, sim))
    control_thread.daemon = True 
    control_thread.start()

    # Run the Simulation (and rendering) in the main thread
    sim.run()