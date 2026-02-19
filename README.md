# Simulación de brazos LeRobot con MuJoCo y PyRoki

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

## Scripts Disponibles

A continuación, se describen los scripts principales y cómo ejecutarlos:

### `quick_mujoco.py`
Un script minimalista para probar que la instalación de MuJoCo funciona correctamente con un modelo simple creado desde un string XML.

```bash
python quick_mujoco.py
```

### `mujoco_viewer_lerobot.py`
Un visualizador pasivo simple para cargar el robot en MuJoCo y verificar que el modelo XML es correcto.

```bash
python mujoco_viewer_lerobot.py
```

### `control_lerobot.py`
Carga el modelo del brazo en MuJoCo y aplica un control simple (onda senoidal) a las articulaciones para probar que los actuadores funcionan correctamente.

```bash
python control_lerobot.py
```

### `so101_ik.py`
Ejemplo de Cinemática Inversa (IK) utilizando **PyRoki** y **Viser**. Permite mover un objetivo interactivamente en el navegador y ver cómo el robot (visualizado en Viser) sigue el objetivo.

```bash
python so101_ik.py
```

### `simplify_mix_mujoco_pyroki.py`
Combina lo mejor de ambos mundos:
- **PyRoki** resuelve la cinemática inversa para alcanzar un objetivo definido en **Viser**.
- La solución se envía a **MuJoCo** para simular la física del robot moviéndose a esa posición.
Ideal para probar control cinemático con física realista.

```bash
python simplify_mix_mujoco_pyroki.py
```

## Estructura del Proyecto

- `pyroki_snippets/`: Módulos auxiliares para funciones de PyRoki.
- `requirements.txt`: Lista de dependencias base.
