# Simulación de brazos LeRobot con MuJoCo y PyRoki
![arm_gif](https://github.com/user-attachments/assets/aaa8351c-93e6-4dfd-b276-cd3e3cd380da)

Este repositorio contiene ejemplos y scripts para visualizar, simular y controlar brazos robóticos de LeRobot (como el `so_arm101`) utilizando **MuJoCo** para la simulación física y **PyRoki** para la resolución de cinemática inversa (IK) y planificación de movimiento.

## ¿Para qué sirve?

El objetivo de este proyecto es proveer un entorno de pruebas ligero y rápido para:
- Visualizar el modelo del robot en MuJoCo.
- Probar algoritmos de control básicos.
- Resolver cinemática inversa (IK) en tiempo real usando PyRoki.
- Visualizar objetivos y movimientos del robot usando `viser`.

## Requisitos

- Python
- Se recomienda encarecidamente usar un **entorno virtual** para aislar las dependencias.

## Instalación

1. **Clonar el repositorio:**

   ```bash
   git clone https://github.com/SinfonIAUniandes/mujoco_sim_lerobot.git
   cd mujoco_sim_lerobot
   ```

2. **Crear y activar un entorno virtual (recomendado):**

   En Linux/macOS:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```
   
   En Windows:
   ```powershell
   python -m venv venv
   .\venv\Scripts\activate
   ```

3. **Instalar dependencias:**

   El archivo `requirements.txt` contiene las dependencias principales. Además, asegúrate de instalar `viser`, `robot_descriptions` y `numpy` para ejecutar todos los ejemplos:

   ```bash
   pip install -r requirements.txt
   ```

   *Nota: `pyroki` se instala directamente desde su repositorio git como se indica en `requirements.txt`.*

## Estructura del Proyecto

- `pyroki_snippets/`: Módulos auxiliares para funciones de PyRoki.
- `examples/`: Contiene scripts de ejemplo para visualizar el robot, resolver IK y planificar movimientos.
- `requirements.txt`: Lista de dependencias base.

## Scripts Principales

A continuación se describen los scripts disponibles en la raíz del proyecto y cómo ejecutarlos:

### `mujoco_visualize_so101.py`
Visualizador básico del brazo robótico SO-ARM101 en MuJoCo. Útil para verificar que el modelo XML carga correctamente sin sensores ni lógica de control compleja.

```bash
python mujoco_visualize_so101.py
```

### `mujoco_visualize_rsd435i.py`
Visualiza únicamente la cámara RealSense D435i dentro de una escena de MuJoCo.

```bash
python mujoco_visualize_rsd435i.py
```

### `mujoco_depth_cam.py`
Carga una escena con una caja (`scene_box.xml`) y renderiza la vista de profundidad y RGB de la cámara. Muestra las ventanas de OpenCV con la visualización en tiempo real.

```bash
python mujoco_depth_cam.py
```

### `mujoco_depth_arm.py`
Simulación del brazo robótico equipado con la cámara de profundidad. Renderiza las vistas RGB y de profundidad desde la perspectiva del brazo mientras corre la simulación física pasiva.

```bash
python mujoco_depth_arm.py
```

### `so101_ik_camera_mount.py`
Script de prueba para ajustar la posición de la cámara (teletransporte) y visualizar el resultado. Permite mover la cámara a coordenadas específicas antes de iniciar la simulación para verificar encuadres.

```bash
python so101_ik_camera_mount.py
```

### `depth_so101_ik.py`
Ejecuta una secuencia predefinida de movimientos (Pick & Place) utilizando Cinemática Inversa (IK) con **PyRoki**. Muestra simultáneamente la visualización RGB/Depth de la cámara montada en el brazo mientras este ejecuta la tarea.

```bash
python depth_so101_ik.py
```

### `depth_so101_ik_web.py`
Integra **Viser** (interfaz web) con la simulación de cámara.
- Abre un servidor web local (en `http://localhost:8080`) donde puedes mover un objetivo interactivo ("Dragger") con el mouse.
- El brazo seguirá el objetivo usando IK en tiempo real.
- Simultáneamente, verás las ventanas de OpenCV con la vista de la cámara montada en el brazo.

```bash
python depth_so101_ik_web.py
```
