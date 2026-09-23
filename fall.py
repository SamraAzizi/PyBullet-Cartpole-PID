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
# --- 7. Print out the joints so we know what's what ---
print("Joint info for the cartpole:")
for j in range(p.getNumJoints(cartpole)):
    info = p.getJointInfo(cartpole, j)
    print("  joint index:", j, "| name:", info[1].decode(), "| type:", info[2])

# --- 8. Let it fall for 3 seconds of simulated time ---
for i in range(720):                 # 720 steps × 1/240 s = 3 seconds
    p.stepSimulation()
    time.sleep(1/240)                # slow it down so our eyes can see it

# --- 9. Keep the window open for a moment, then close ---
time.sleep(1)
p.disconnect()