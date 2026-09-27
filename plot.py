import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt

# ---------- Setup ----------
p.connect(p.DIRECT)          # <-- no window! run silently, faster
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240)

plane = p.loadURDF("plane.urdf")
cartpole = p.loadURDF("cartpole.urdf", [0, 0, 0])

CART_JOINT = 0
POLE_JOINT = 1
# ---------- PID parameters ----------
Kp = 20.0
Kd = 2.0
Ki = 0.0

MAX_SPEED = 5.0

prev_error = 0.0
integral = 0.0
# ---------- Recorders (empty lists, we'll fill them) ----------
times       = []
pole_angles = []
cart_positions = []

# ---------- Reader ----------
def get_state():
    cart_pos, cart_vel = p.getJointState(cartpole, CART_JOINT)[:2]
    pole_angle, pole_vel = p.getJointState(cartpole, POLE_JOINT)[:2]
    return cart_pos, cart_vel, pole_angle, pole_vel
