import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240)

plane = p.loadURDF("plane.urdf")
cartpole = p.loadURDF("cartpole.urdf", [0, 0, 0])

# Joint indices from Step 1
CART_JOINT = 0
POLE_JOINT = 1

# --- Our reader function ---
def get_state():
    cart_pos, cart_vel = p.getJointState(cartpole, CART_JOINT)[:2]
    pole_angle, pole_vel = p.getJointState(cartpole, POLE_JOINT)[:2]
    return cart_pos, cart_vel, pole_angle, pole_vel

# --- Let it fall, printing the state every 60 steps (= 4 times per second) ---
for i in range(720):
    p.stepSimulation()

    if i % 60 == 0:
        cart_pos, cart_vel, pole_angle, pole_vel = get_state()
        print(f"t={i/240:.2f}s | "
              f"cart_pos={cart_pos:+.3f} | cart_vel={cart_vel:+.3f} | "
              f"pole_angle={pole_angle:+.3f} rad | pole_vel={pole_vel:+.3f}")

    time.sleep(1/240)

p.disconnect()