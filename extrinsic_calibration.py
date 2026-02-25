import mujoco
import numpy as np
import json

PACKAGE_PATH = "./robotstudio_so101/"
MJCF_FILE = "so101_camera_mount.xml"
CAMERA_NAME = "realsense_d435i"

def calibrate_workspace():
    print("Starting extrinsic calibration...")
    
    # Load the model
    model = mujoco.MjModel.from_xml_path(PACKAGE_PATH + MJCF_FILE)
    data = mujoco.MjData(model)
    
    # Replicate the initialization logic from so101_sim.py to ensure the camera 
    # is physically snapped to the mount just like it is during runtime.
    cam_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "d435i")
    mount_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "air_camera_mount")
    
    if cam_body_id != -1 and mount_body_id != -1:
        model.body_pos[cam_body_id] = model.body_pos[mount_body_id]
        model.body_quat[cam_body_id] = model.body_quat[mount_body_id]
        
    # Calculate the physics kinematics to update all global positions
    mujoco.mj_forward(model, data)
    
    # Extract the global position and rotation matrix of the camera lens
    actual_cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
    cam_pos = data.cam_xpos[actual_cam_id].tolist()
    cam_mat = data.cam_xmat[actual_cam_id].reshape(3, 3).tolist()
    
    # Save the calibration data
    calib_data = {
        "cam_pos": cam_pos,
        "cam_mat": cam_mat
    }
    
    with open("camera_calib.json", "w") as f:
        json.dump(calib_data, f, indent=4)
        
    print("Calibration successful!")
    print(f"Camera Translation (XYZ): {cam_pos}")
    print("Saved to 'camera_calib.json'")

if __name__ == "__main__":
    calibrate_workspace()