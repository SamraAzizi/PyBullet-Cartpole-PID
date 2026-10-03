# S5 — Cartpole Balance Control in PyBullet

A beginner-friendly mechatronics project: balance a simulated cartpole using a
PID controller, quantify the performance with plots, and demonstrate
disturbance rejection.

No hardware required. Runs entirely on your laptop.

---

## What is a cartpole?

A cartpole is a wheeled cart with a pole hinged on top:

```
        |
        |   <- pole (free to tilt)
        |
    [=======]   <- cart (free to slide left/right)
    ─────────────   <- track
```

The cart can only slide left or right. It cannot grab the pole. The task is to
push the cart at every instant so the pole stays balanced upright.

This is a classic control problem and appears in rockets, Segways, and robot
arms.

---

## Project goal

- Simulate a cartpole in PyBullet.
- Read its state every timestep.
- Control it with a PID controller.
- Tune it until the pole balances.
- Quantify performance with plots.
- Add a disturbance and show recovery.

---

## Demo

Balancing result:

![Balancing plot](plots/balancing.png)

Disturbance rejection:

![Disturbance plot](plots/disturbance.png)

---

## Requirements

- Python 3.8 or newer
- The packages listed in `requirements.txt`

Install them with:

```bash
pip install -r requirements.txt
```

---

## How to run

Each script is standalone. Run them in order for a guided tour:

```bash
python fall.py         # Load cartpole, let it fall (sanity check)
python read_state.py   # Read state every step, print it
python pid.py          # First working PID balancer
python plot.py         # Record and plot the results
python disturbance.py  # Push the pole, verify recovery
```

Close the PyBullet window (or press `Ctrl+C`) to stop the GUI scripts.

---

## Files

| File | Purpose |
|---|---|
| `fall.py` | Loads cartpole, applies gravity, lets it fall. |
| `read_state.py` | Reads joint positions and velocities every step. |
| `pid.py` | First working PID controller (interactive GUI). |
| `plot.py` | Runs headless, records data, and plots tracking error. |
| `disturbance.py` | Adds a lateral push and measures recovery time. |
| `requirements.txt` | Python package dependencies. |
| `README.md` | This file. |

---

## The controller

The PID controller converts the pole's tilt into a commanded cart velocity:

```python
error        = 0 - pole_angle
integral    += error
derivative   = error - prev_error
target_speed = Kp * error + Ki * integral + Kd * derivative
```

Each term:

| Term | Meaning | Role |
|------|---------|------|
| `Kp` | Proportional | Push harder the more tilted the pole is. |
| `Ki` | Integral | Correct for accumulated error over time. |
| `Kd` | Derivative | Damp oscillation; acts like a brake. |

The output is clamped to `±5 m/s` and applied via
`p.setJointMotorControl2(..., controlMode=p.VELOCITY_CONTROL, ...)`.

---

## Tuned parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `Kp` | 20.0 | Strong enough to catch a falling pole, weak enough to stay smooth. |
| `Kd` | 2.0 | Damps the wobble introduced by `Kp = 20`. |
| `Ki` | 0.0 | Not needed — no persistent offset in this system. |
| `MAX_SPEED` | 5.0 m/s | Safety clamp to prevent sim instability. |
| Motor force | 1000 N | Strong enough to hit commanded velocities. |
| Sim timestep | 1/240 s | Standard for stable physics. |
| Disturbance | 80 N for 0.1 s at t = 4 s | Strong enough to visibly tilt the pole. |

**How these were chosen:** start with `Kp` only (1 → 5 → 10 → 20 → 50).
At `Kp = 50` the pole oscillates; at `Kp = 20` it balances. Then raise `Kd`
from 0 → 0.1 → 0.5 → 2.0 until the wobble disappears. `Ki` stayed at 0
because adding it made the cart drift and did not improve pole stability.

---

## Results

| Metric | Value |
|--------|-------|
| Pole angle range while balancing | ±0.05 rad (±3°) |
| Settling time into ±0.05 rad | < 1 s |
| Disturbance magnitude | 80 N × 0.1 s |
| Recovery time after disturbance | ~0.6 s |

**Free-fall baseline:** with no controller, the pole collapses within
~0.5 s — confirming the simulator models realistic dynamics.

**Balancing:** with the tuned PID, the pole angle stays within ±0.05 rad for
the full 10 s run while the cart makes small back-and-forth corrections.

**Disturbance rejection:** at t = 4 s, an 80 N push is applied for 0.1 s.
The pole angle spikes to ~±0.3 rad and returns inside ±0.05 rad within
~0.6 s.

---

## What I learned

- **A control loop is simple at its core:** read state → compute error →
  apply correction → repeat.
- **PID is a recipe, not a theory.** The three numbers must be tuned by
  trial and error; the formula itself is three lines.
- **Damping matters.** Without `Kd`, a large `Kp` makes the pole oscillate
  wildly. The derivative term is what makes the system feel smooth.
- **Not every term is needed.** `Ki` was unnecessary here. Knowing when to
  leave a term out is part of engineering.
- **Measurement is the point.** Watching the pole balance is satisfying;
  plotting the error over time is what turns a demo into an engineering
  result.

---
