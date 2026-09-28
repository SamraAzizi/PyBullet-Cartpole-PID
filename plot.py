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

# ---------- Main loop ----------
STEPS = 2400                 # 10 seconds
for i in range(STEPS):
    p.stepSimulation()

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

    # ---------- Record ----------
    times.append(i / 240.0)
    pole_angles.append(pole_angle)
    cart_positions.append(cart_pos)

p.disconnect()

# ---------- Plot ----------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(times, pole_angles, color="crimson")
ax1.axhline(0, color="gray", linewidth=0.5, linestyle="--")
ax1.set_ylabel("Pole angle (rad)")
ax1.set_title("Tracking error over time")
ax1.grid(True)

ax2.plot(times, cart_positions, color="steelblue")
ax2.axhline(0, color="gray", linewidth=0.5, linestyle="--")
ax2.set_ylabel("Cart position (m)")
ax2.set_xlabel("Time (s)")
ax2.grid(True)

plt.tight_layout()
plt.show()