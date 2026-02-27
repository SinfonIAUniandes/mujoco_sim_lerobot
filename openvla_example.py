import time
import cv2
import numpy as np
import torch
from PIL import Image
from threading import Thread

# Hugging Face imports for OpenVLA
from transformers import AutoModelForVision2Seq, AutoProcessor, BitsAndBytesConfig

# Simulation imports
from so101_sim import SO101Simulation

class OpenVLAController:
    def __init__(self, model_id="openvla/openvla-7b", task_prompt="pick up the green box"):
        self.latest_bgr = None
        self.task_prompt = task_prompt
        
        # 1. Initialize Device & Model
        print(f"Loading OpenVLA policy: {model_id}...")
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        
        # 2. Configure 4-Bit Quantization
        quantization_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_compute_dtype=torch.bfloat16,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_use_double_quant=True
        )

        # 3. Load Model with Quantization and Hardcoded Device Map
        self.processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
        self.vla = AutoModelForVision2Seq.from_pretrained(
            model_id, 
            quantization_config=quantization_config,
            device_map={"": 0},  # <--- THIS IS THE FIX
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True, 
            trust_remote_code=True
        )
        
        print("Model loaded successfully in 4-bit mode.")

    def on_rgb_frame(self, bgr_image):
        """Callback triggered by the simulation rendering loop."""
        self.latest_bgr = bgr_image.copy()

    def get_vla_frame(self):
        """Formats the latest OpenCV image for the OpenVLA processor."""
        if self.latest_bgr is None:
            return None
        
        # OpenVLA processor handles resizing and tensor conversion natively,
        # so we just need to pass it a standard RGB PIL Image.
        rgb_image = cv2.cvtColor(self.latest_bgr, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb_image)


def vla_control_loop(controller, sim):
    """Runs continuously in a background thread to predict and apply actions."""
    print("Waiting for camera feed...")
    while controller.latest_bgr is None:
        time.sleep(0.1)
        
    print(f"Starting OpenVLA Inference Loop. Task: '{controller.task_prompt}'")
    
    # OpenVLA documentation recommends 5Hz to 10Hz control frequency
    rate_hz = 5.0 
    sleep_time = 1.0 / rate_hz

    # OpenVLA expects a very specific prompt structure
    prompt = f"In: What action should the robot take to {controller.task_prompt}?\nOut:"

    while True:
        start_time = time.time()
        
        # 1. Grab image
        img_pil = controller.get_vla_frame()
        if img_pil is None:
            continue

        # 2. Format inputs using the Hugging Face processor
        inputs = controller.processor(prompt, img_pil).to(controller.device, dtype=torch.bfloat16)
        
        # 3. Predict Action
        with torch.inference_mode():
            # unnorm_key="bridge_orig" outputs standard 7-DoF absolute/delta cartesian actions
            action = controller.vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)

        # print(f"Raw OpenVLA Output: {action}")
        
        try:
            # 4. Translate 7-DoF Cartesian output back to the IK Solver
            ik_target = np.array([action[0], action[1], action[2]])
            target_rpy = np.array([action[3], action[4], action[5]])
            gripper_cmd = action[6] 
            
            sim.apply_ik({
                "pos": ik_target, 
                "rpy": target_rpy, 
                "gripper": gripper_cmd
            })
            
        except IndexError:
            print("Action dimension mismatch! Check OpenVLA output shape.")
            print(f"Raw action: {action}")
            break
            
        # Maintain loop frequency
        elapsed = time.time() - start_time
        if elapsed < sleep_time:
            time.sleep(sleep_time - elapsed)

if __name__ == "__main__":
    PACKAGE_PATH = "./robotstudio_so101/"
    MJCF_FILE = "so101_camera_mount.xml"
    
    # Initialize the OpenVLA controller
    vla_controller = OpenVLAController(
        model_id="openvla/openvla-7b", 
        task_prompt="pick up the green box"
    )
    
    # Initialize Simulation (Reverted to IK control)
    sim = SO101Simulation(
        xml_path=PACKAGE_PATH + MJCF_FILE,
        urdf_name="so_arm101_description",    
        ik_target_link="gripper", # <--- WE ARE BACK TO IK!            
        use_ik_web=False,
        enable_rgb=True,
        enable_depth=False, 
        show_cv2=False, 
        enable_rerun=False,    
        rerun_log_meshes=False,  
        rerun_log_tf=False,      
        rerun_depth_mode="none",
        rerun_log_rgb=False,          
        rgb_callback=vla_controller.on_rgb_frame
    )
    
    # Start the Control sequence in a background thread
    control_thread = Thread(target=vla_control_loop, args=(vla_controller, sim))
    control_thread.daemon = True 
    control_thread.start()

    # Run the Simulation (and rendering) in the main thread
    sim.run()