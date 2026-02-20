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

# 2. Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer to stop the script
    while viewer.is_running():
        step_start = time.time()

        # Step the physics
        mujoco.mj_step(model, data)

        # Sync the viewer with the new physics state
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
            # Forzar actualización de escena para aplicar flags de profundidad
            renderer.update_scene(data, camera=camera_name) 
            depth_image = renderer.render() 
            
            # 3. Procesamiento de Profundidad (Normalización robusta)
            max_depth = 3.0 # Rango máximo en metros para visualización
            depth_visual = depth_image.copy()
            
            # Arreglar fondo: MuJoCo pone espacios vacíos como 0.0, inf o nan
            bg_mask = (depth_visual == 0.0) | np.isinf(depth_visual) | np.isnan(depth_visual)
            depth_visual[bg_mask] = max_depth
            
            # Recortar y normalizar
            depth_normalized = np.clip(depth_visual, 0, max_depth) / max_depth
            depth_8bit = (depth_normalized * 255).astype(np.uint8)
            
            # Invertir mapa: Cerca = brillante, Lejos = oscuro
            depth_8bit = 255 - depth_8bit
            
            # Aplicar mapa de color (JET)
            depth_colormap = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

            # 4. Mostrar imágenes
            cv2.imshow("RGB Camera", bgr_image)
            cv2.imshow("Depth Camera", depth_colormap)
            
            # Salir con 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            last_render_time = current_time
        # --- CODIGO DE VISUALIZACION END ---

        # Rough timing to keep it real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

cv2.destroyAllWindows()