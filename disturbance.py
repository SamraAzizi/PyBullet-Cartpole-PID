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


# ---------- Recorders ----------
times = []
pole_angles = []
cart_positions = []

# ---------- Reader ----------
def get_state():
    cart_pos, cart_vel = p.getJointState(cartpole, CART_JOINT)[:2]
    pole_angle, pole_vel = p.getJointState(cartpole, POLE_JOINT)[:2]
    return cart_pos, cart_vel, pole_angle, pole_vel

# ---------- Main loop ----------
STEPS = 2880  # 12 seconds
for i in range(STEPS):
    p.stepSimulation()

    # Apply the push during the window
    if PUSH_START_STEP <= i < PUSH_END_STEP:
        p.applyExternalForce(
            objectUniqueId=cartpole,
            linkIndex=POLE_JOINT,
            forceObj=[PUSH_FORCE_N, 0, 0],   # push along +x
            posObj=[0, 0, 0],
            flags=p.WORLD_FRAME
        )

    cart_pos, cart_vel, pole_angle, pole_vel = get_state()

    # PID
    error = 0.0 - pole_angle
    integral += error
    derivative = error - prev_error
    prev_error = error

    target_speed = Kp * error + Ki * integral + Kd * derivative
    target_speed = max(min(target_speed, MAX_SPEED), -MAX_SPEED)

    p.setJointMotorControl2(
        bodyIndex=cartpole,
        jointIndex=CART_JOINT,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=target_speed,
        force=1000
    )

    # Record
    times.append(i / 240.0)
    pole_angles.append(pole_angle)
    cart_positions.append(cart_pos)

p.discon