import mujoco
import mujoco.viewer
import time
import numpy as np
import rerun as rr  # Importar Rerun
import uuid

PACKAGE_PATH: str = "./robotstudio_so101/"
MJCF_FILE: str = "so101_camera_mount.xml"

model = mujoco.MjModel.from_xml_path(PACKAGE_PATH + MJCF_FILE)
data = mujoco.MjData(model)

# Get the internal IDs for the camera and the mount
cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "d435i")
mount_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "air_camera_mount")

# Snap the camera's position and orientation to match the mount perfectly
model.body_pos[cam_id] = model.body_pos[mount_id]
model.body_quat[cam_id] = model.body_quat[mount_id]

mujoco.mj_forward(model, data)

# --- INICIO SETUP RENDERER ---
camera_name = "realsense_d435i"
width, height = 640, 480
renderer = mujoco.Renderer(model, height=height, width=width)

render_fps = 30
render_interval = 1.0 / render_fps
last_render_time = time.time()
# --- FIN SETUP RENDERER ---

# Initialize Rerun and spawn the viewer alongside the script
rand_id = str(uuid.uuid4())
rr.init(rand_id, recording_id=rand_id, spawn=True)

rr.log(
    "world/realsense_d435i",
    rr.Pinhole(
        resolution=[width, height],
        focal_length=400,  # Approximate focal length
    ),
    rr.ViewCoordinates.RUB,  # Define the coordinate system for the camera.
    static=True,
)

# =========================================================================
# --- INICIO DE CARGA ESTÁTICA DE MALLAS 3D (VISUALES) ---
# =========================================================================
for geom_id in range(model.ngeom):
    body_id = model.geom_bodyid[geom_id]
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)

    # Ignoramos la caja de colisión genérica y los cuerpos sin nombre
    if not body_name or body_name in {"box"}:
        continue

    # Filtramos para visualizar solo geometrías visuales
    if (
        model.geom_group[geom_id] > 2
        or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH
    ):
        continue

    geom_name = (
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
    )
    entity_path = f"world/tf/{body_name}/{geom_name}"

    # 1. Posición y rotación LOCAL de la geometría
    local_pos = model.geom_pos[geom_id]
    local_quat_wxyz = model.geom_quat[geom_id]
    local_quat_xyzw = [
        local_quat_wxyz[1],
        local_quat_wxyz[2],
        local_quat_wxyz[3],
        local_quat_wxyz[0],
    ]

    rr.log(
        entity_path,
        rr.Transform3D(
            translation=local_pos, rotation=rr.Quaternion(xyzw=local_quat_xyzw)
        ),
        static=True,
    )

    # 2. Extracción de vértices y caras
    mesh_id = model.geom_dataid[geom_id]
    if mesh_id == -1:
        continue

    vert_adr = model.mesh_vertadr[mesh_id]
    vert_num = model.mesh_vertnum[mesh_id]
    vertices = model.mesh_vert[vert_adr : vert_adr + vert_num]

    face_adr = model.mesh_faceadr[mesh_id]
    face_num = model.mesh_facenum[mesh_id]
    faces = model.mesh_face[face_adr : face_adr + face_num]

    # 3. EXTRAER EL COLOR CORRECTAMENTE (Material vs Geom)
    mat_id = model.geom_matid[geom_id]
    if mat_id != -1:
        # Extrae el RGBA del material referenciado
        rgba = model.mat_rgba[mat_id]
    else:
        # Extrae el RGBA directo de la geometría
        rgba = model.geom_rgba[geom_id]

    # Convertir a formato 0-255
    color = np.array([rgba[0], rgba[1], rgba[2], rgba[3]]) * 255
    color = color.astype(np.uint8)

    # Multiplicar el color por el número de vértices para que Rerun pinte toda la malla
    vertex_colors = np.tile(color, (vert_num, 1))

    # 4. Registrar la malla en Rerun con sus colores
    rr.log(
        f"{entity_path}/mesh",
        rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=faces,
            vertex_colors=vertex_colors,
        ),
        static=True,
    )
# =========================================================================
# --- FIN DE CARGA ESTÁTICA ---
# =========================================================================

sim_start_time = time.time()

# Launch the passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        real_elapsed_time = time.time() - sim_start_time

        while data.time < real_elapsed_time:
            mujoco.mj_step(model, data)

            # Keep your camera snapping math INSIDE the physics loop
            model.body_pos[cam_id] = model.body_pos[mount_id]
            model.body_quat[cam_id] = model.body_quat[mount_id]
            mujoco.mj_forward(model, data)

        viewer.sync()

        # --- CODIGO DE VISUALIZACION RERUN START ---
        current_time = time.time()
        if current_time - last_render_time >= render_interval:
            last_render_time = (
                current_time  # Importante: Actualizar el tiempo para mantener los FPS
            )

            # Bodies we want the system to be blind to
            IGNORED_BODIES = {"box"}

            # 1. Log the TF Tree (Global poses) and kinematic skeleton
            for i in range(1, model.nbody):
                body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)

                if not body_name or body_name in IGNORED_BODIES:
                    continue

                pos = data.xpos[i]
                quat_wxyz = data.xquat[i]
                quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]

                # Log the coordinate frame (the X/Y/Z axes) for each valid part
                # AL ACTUALIZAR ESTO, LAS MALLAS HIJAS SE MOVERÁN AUTOMÁTICAMENTE
                rr.log(
                    f"world/tf/{body_name}",
                    rr.Transform3D(
                        translation=pos, rotation=rr.Quaternion(xyzw=quat_xyzw)
                    ),
                    rr.TransformAxes3D(0.05),
                )

                # Draw an arrow from the parent body to this body to visualize the tree structure
                parent_id = model.body_parentid[i]

                if parent_id != 0:
                    parent_name = mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_BODY, parent_id
                    )

                    if parent_name and parent_name not in IGNORED_BODIES:
                        parent_pos = data.xpos[parent_id]
                        vector_to_child = pos - parent_pos

                        rr.log(
                            f"world/tf_skeleton/{parent_name}_to_{body_name}",
                            rr.Arrows3D(
                                origins=[parent_pos],
                                vectors=[vector_to_child],
                                colors=[[150, 150, 150]],
                                radii=0.002,
                            ),
                        )

            # 1.5. Draw arrow from robot base to camera
            base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            if base_id != -1 and cam_id != -1:
                base_pos = data.xpos[base_id]
                cam_body_pos = data.xpos[cam_id]

                rr.log(
                    "world/visuals/base_to_camera",
                    rr.Arrows3D(
                        origins=[base_pos],
                        vectors=[cam_body_pos - base_pos],
                        colors=[[255, 0, 0]],
                        radii=0.005,
                    ),
                )

            # Add the 3d point cloud from the depth camera to the viewer
            renderer.update_scene(data, camera="realsense_d435i")
            renderer.disable_depth_rendering()
            rgb_image = renderer.render()
            renderer.enable_depth_rendering()
            depth_image = renderer.render()

            # Log the RGB and Depth images
            rr.log("world/realsense_d435i/rgb", rr.Image(rgb_image))
            rr.log("world/realsense_d435i/depth", rr.DepthImage(depth_image, meter=1.0))

            # --- UPDATE CAMERA TRANSFORM ---
            actual_cam_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_CAMERA, "realsense_d435i"
            )

            if actual_cam_id != -1:
                cam_pos = data.cam_xpos[actual_cam_id]
                cam_mat = data.cam_xmat[actual_cam_id]
                cam_quat_wxyz = np.zeros(4)
                mujoco.mju_mat2Quat(cam_quat_wxyz, cam_mat)

                cam_quat_xyzw = [
                    cam_quat_wxyz[1],
                    cam_quat_wxyz[2],
                    cam_quat_wxyz[3],
                    cam_quat_wxyz[0],
                ]

                rr.log(
                    "world/realsense_d435i",
                    rr.Transform3D(
                        translation=cam_pos, rotation=rr.Quaternion(xyzw=cam_quat_xyzw)
                    ),
                )
        # --- CODIGO DE VISUALIZACION RERUN END ---
