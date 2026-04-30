# Simulación de brazos LeRobot con MuJoCo y PyRoki
![arm_gif](https://github.com/user-attachments/assets/aaa8351c-93e6-4dfd-b276-cd3e3cd380da)


<img width="705" height="702" alt="image" src="https://github.com/user-attachments/assets/11dc2414-a661-49db-b98d-7365331451df" />

<br/>
<img width="1237" height="702" alt="image" src="https://github.com/user-attachments/assets/b62c632a-c73f-4922-b2de-c0a343371965" />

<br/>
<img width="1241" height="702" alt="image" src="https://github.com/user-attachments/assets/3aab2468-0e62-402f-93d6-707f4b0a59ca" />



Este repositorio contiene scripts y utilidades para simular y controlar el brazo SO-ARM101 usando MuJoCo (simulación física) y PyRoki (IK/planning).

Resumen rápido de lo que hay
- so101_sim.py — Módulo principal que expone la clase SO101Simulation (API de simulación, cámaras, Rerun, IK).
- run_sim.py — Ejemplo minimal para lanzar la simulación y parámetros comunes.
- go_grab.py — Pipeline completo: visión (detección de caja verde), conversión a coordenadas, y secuencia de agarre usando apply_ik.
- keyboard_input.py — Interfaz por consola para enviar objetivos IK manualmente.
- extrinsic_calibration.py — Guarda `camera_calib.json` con la pose y matriz de la cámara (usado por go_grab).
- camera_calib.json — Archivo de calibración generado por extrinsic_calibration.py.
- robotstudio_so101/ — Modelos, meshes y XMLs MuJoCo del robot y la cámara.
- pyroki_snippets/ — Helpers usados para resolver IK desde PyRoki.

Requisitos
- Python 3.x
- MuJoCo Python bindings
- numpy, opencv-python
- (Opcional) rerun-sdk para visualización en Rerun
- (Opcional) pyroki, robot_descriptions, viser para IK y web visualizer

Instalación básica
1. Crear y activar un entorno virtual:
   python3 -m venv venv && source venv/bin/activate
2. Instalar dependencias:
   pip install -r requirements.txt
3. (Opcional) instalar paquetes extra para IK/web:
   pip install pyroki robot_descriptions viser rerun-sdk

Flujo / API rápido (so101_sim.S0101Simulation)
- Importar:
  from so101_sim import SO101Simulation

- Constructor importante (parámetros más usados):
  - xml_path: ruta al MJCF (ej. "./robotstudio_so101/so101_camera_mount.xml")
  - urdf_name: nombre del paquete URDF (para PyRoki)
  - ik_target_link: link objetivo para IK ("gripper")
  - use_ik_web: habilita interfaz web de Viser (si disponible)
  - enable_rerun: habilita logging a Rerun (si instalado)
  - enable_rgb / enable_depth / show_cv2: streaming de cámara y ventanas OpenCV
  - rerun_depth_mode: "none" | "depth" | "pointcloud"
  - Callbacks:
    - rgb_callback(bgr_image)         # recibe BGR frame (OpenCV order)
    - depth_callback(raw_depth, colored_depth)
    - joint_callback(joint_dict)      # diccionario {joint_name: qpos}
    - control_callback(sim_time)      # debe devolver dict de comandos actuadores
    - ik_callback(sim_time)           # debe devolver {"pos": np.array, "rpy"/"quat":..., "gripper": val}

- Métodos públicos relevantes:
  - run() : inicia la ventana de MuJoCo y el bucle de simulación.
  - apply_commands(dict) : aplica directamente values a actuadores por nombre.
  - apply_ik(dict) : solicita IK (PyRoki) y aplica la solución a los actuadores mapeados.

Ejemplo mínimo (usar run_sim.py como referencia):
- Crear instancia con callbacks y opciones, luego llamar sim.run().

Contacto
- Proyecto mantenido por SinfonIA. Para problemas o adaptaciones, editar y ejecutar los scripts en la raíz.
