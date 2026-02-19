import mujoco
import mujoco.viewer
import numpy as np
import time

# Minimal XML with one movable joint (hinge)
xml = """
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    <body pos="0 0 1">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="capsule" size=".02" fromto="0 0 0 0 0 -0.5" rgba="1 0 0 1"/>
      <body pos="0 0 -0.5">
        <joint type="hinge" axis="1 0 0"/>
        <geom type="capsule" size=".02" fromto="0 0 0 0 0 -0.5" rgba="0 1 0 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print(f"Model loaded. Number of DOFs (nq): {model.nq}")  # Should print 2

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running():
        # Move the first joint
        if model.nq > 0:
            data.qpos[0] = np.sin(time.time())
            
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)