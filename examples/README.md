# Ejemplos sencillos de Simulación con MuJoCo para el Robot SO-ARM101

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