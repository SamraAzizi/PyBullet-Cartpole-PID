import pybullet as p
import pybullet_data
import time

# --- 1. Connect to the simulator (open the GUI window) ---
p.connect(p.GUI)

# --- 2. Tell PyBullet where its built-in models live ---
p.setAdditionalSearchPath(pybullet_data.getDataPath())
