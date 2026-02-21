import mujoco
import mujoco.viewer
import time
import numpy as np
import cv2  # Importar OpenCV

PACKAGE_PATH: str = "./robotstudio_so101/"

MJCF_FILE: str = "so101_camera_mount.xml" #can be so101.xml, scene.xml or scene_box.xml

model = mujoco.MjModel.from_xml_path(PACKAGE_PATH+MJCF_FILE)
data = mujoco.MjData(model)

# Get the internal IDs for the camera and the mount
cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "d435i")
mount_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "air_camera_mount")

# Snap the camera's position and orientation to match the mount perfectly
model.body_pos[cam_id] = model.body_pos[mount_id]
model.body_quat[cam_id] = model.body_quat[mount_id]

data = mujoco.MjData(model)
mujoco.mj_forward(model, data) 

# Importante: llamar a mj_forward para propagar el cambio de posición a la cinemática
mujoco.mj_forward(model, data)
# --- FIN DEL CAMBIO ---

# --- INICIO SETUP RENDERER ---
# Configuración del renderizador offscreen
# Asegúrate de que este nombre coincida con el nombre de la cámara en el XML d435i_mountable.xml
camera_name = "realsense_d435i" 
width, height = 640, 480
renderer = mujoco.Renderer(model, height=height, width=width)

# Control de FPS para la visualización de OpenCV
render_fps = 30
render_interval = 1.0 / render_fps
last_render_time = time.time()
# --- FIN SETUP RENDERER ---

# Track when the simulation started (Add this right before the viewer launch)
sim_start_time = time.time()

# 2. Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        # 1. Calculate how much real-world time has actually passed
        real_elapsed_time = time.time() - sim_start_time

        # 2. Rapidly step the physics forward until it catches up to real-world time
        while data.time < real_elapsed_time:
            mujoco.mj_step(model, data)
            
            # Keep your camera snapping math INSIDE the physics loop
            model.body_pos[cam_id] = model.body_pos[mount_id]
            model.body_quat[cam_id] = model.body_quat[mount_id]
            mujoco.mj_forward(model, data) 

        # 3. Sync the viewer ONLY once physics is up to date
        viewer.sync()

        # --- CODIGO DE VISUALIZACION START ---
        current_time = time.time()
        if current_time - last_render_time >= render_interval:
            # 1. Update and Render RGB
            renderer.update_scene(data, camera=camera_name)
            renderer.disable_depth_rendering()
            rgb_image = renderer.render()
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            
            # 2. Update and Render Depth
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera_name) 
            depth_image = renderer.render() 
            
            # 3. Procesamiento de Profundidad 
            max_depth = 3.0 
            depth_visual = depth_image.copy()
            
            bg_mask = (depth_visual == 0.0) | np.isinf(depth_visual) | np.isnan(depth_visual)
            depth_visual[bg_mask] = max_depth
            
            depth_normalized = np.clip(depth_visual, 0, max_depth) / max_depth
            depth_8bit = (depth_normalized * 255).astype(np.uint8)
            depth_8bit = 255 - depth_8bit
            
            depth_colormap = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

            # 4. Mostrar imágenes
            cv2.imshow("RGB Camera", bgr_image)
            cv2.imshow("Depth Camera", depth_colormap)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            last_render_time = current_time
        # --- CODIGO DE VISUALIZACION END ---

cv2.destroyAllWindows()