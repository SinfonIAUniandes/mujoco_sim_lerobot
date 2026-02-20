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
