import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np


# ---------- Setup ----------
p.connect(p.DIRECT)
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
