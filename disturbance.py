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

# ---------- Disturbance config ----------
PUSH_START_S = 4.0            # when the push begins
PUSH_DURATION_S = 0.1         # how long the push lasts
PUSH_FORCE_N = 80.0           # how strong the push is (Newtons)
PUSH_START_STEP = int(PUSH_START_S * 240)
PUSH_END_STEP   = int((PUSH_START_S + PUSH_DURATION_S) * 240)

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

p.disconnect()

# ---------- Plot ----------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(times, pole_angles, color="crimson")
ax1.axhline(0, color="gray", linewidth=0.5, linestyle="--")
ax1.axvspan(PUSH_START_S, PUSH_START_S + PUSH_DURATION_S,
            color="orange", alpha=0.3, label="disturbance")
ax1.set_ylabel("Pole angle (rad)")
ax1.set_title("Disturbance rejection: pole angle vs time")
ax1.legend()
ax1.grid(True)

ax2.plot(times, cart_positions, color="steelblue")
ax2.axvspan(PUSH_START_S, PUSH_START_S + PUSH_DURATION_S,
            color="orange", alpha=0.3)
ax2.set_ylabel("Cart position (m)")
ax2.set_xlabel("Time (s)")
ax2.grid(True)

plt.tight_layout()
plt.show()

# ---------- Recovery time calculation ----------
tolerance = 0.05
angles = np.array(pole_angles)
t_array = np.array(times)

# Look only AFTER the push ends
after_mask = t_array >= (PUSH_START_S + PUSH_DURATION_S)
angles_after = angles[after_mask]
times_after = t_array[after_mask]

outside = np.abs(angles_after) > tolerance
if outside.any():
    last_outside_idx = np.where(outside)[0][-1]
    recovery_time = times_after[last_outside_idx] - (PUSH_START_S + PUSH_DURATION_S)
    print(f"Recovery time after push: {recovery_time:.2f} s")
else:
    print("Pole never left the tolerance band after the push.")