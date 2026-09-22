import pybullet as p
import pybullet_data
import time

# --- 1. Connect to the simulator (open the GUI window) ---
p.connect(p.GUI)

# --- 2. Tell PyBullet where its built-in models live ---
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# --- 3. Set gravity (downward) ---
p.setGravity(0, 0, -9.81)

# --- 4. Set the simulation timestep ---
# 1/240 means the simulator advances 240 times per second of "sim time"
p.setTimeStep(1/240)

# --- 5. Load the ground plane ---
plane = p.loadURDF("plane.urdf")

# --- 6. Load the cartpole at the origin ---
cartpole = p.loadURDF("cartpole.urdf", [0, 0, 0])
