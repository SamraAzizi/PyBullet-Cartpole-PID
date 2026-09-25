import pybullet as p
import pybullet_data
import time

# ---------- Setup ----------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240)

plane = p.loadURDF("plane.urdf")
cartpole = p.loadURDF("cartpole.urdf", [0, 0, 0])

CART_JOINT = 0
POLE_JOINT = 1

# ---------- PID parameters (start with these) ----------
Kp = 1.0
Kd = 0.1
Ki = 0.0

MAX_SPEED = 5.0          # cart can't go faster than this (m/s)

# ---------- PID bookkeeping ----------
prev_error = 0.0
integral = 0.0

# ---------- Reader ----------
def get_state():
    cart_pos, cart_vel = p.getJointState(cartpole, CART_JOINT)[:2]
    pole_angle, pole_vel = p.getJointState(cartpole, POLE_JOINT)[:2]
    return cart_pos, cart_vel, pole_angle, pole_vel
