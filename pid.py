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

# ---------- Main loop ----------
for i in range(2400):            # 10 seconds
    p.stepSimulation()

    cart_pos, cart_vel, pole_angle, pole_vel = get_state()

    # 1. Error: we want pole_angle == 0
    error = 0.0 - pole_angle

    # 2. PID math
    integral    += error
    derivative   = error - prev_error
    prev_error   = error

    target_speed = Kp * error + Ki * integral + Kd * derivative

    # 3. Safety: don't let the cart move absurdly fast
    target_speed = max(min(target_speed, MAX_SPEED), -MAX_SPEED)

    # 4. Apply it
    p.setJointMotorControl2(
        bodyIndex=cartpole,
        jointIndex=CART_JOINT,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=target_speed,
        force=1000
    )

    # 5. Show progress every half second
    if i % 120 == 0:
        print(f"t={i/240:.2f}s | pole_angle={pole_angle:+.3f} | "
              f"target_speed={target_speed:+.3f}")

    time.sleep(1/240)

p.disconnect()