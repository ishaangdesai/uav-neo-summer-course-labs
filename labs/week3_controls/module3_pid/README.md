# Week 3 · Module 3 — PID Control

Full PID control, and a capstone that combines Week 2 vision with Week 3 control.

## What you'll learn

- The PID law (P + I + D) and anti-windup
- Why the integral term removes steady-state error
- Estimating distance by integrating velocity (dead reckoning)
- Visual-servoing: closing a PID loop on a camera pixel error

## Key terms

- **PID control** — a controller that sums three terms: `output = Kp·error + Ki·(integral of error) + Kd·(rate of change of error)`.
- **Integral term (I)** — adds up error over time. A small constant error builds the integral until the controller pushes hard enough to erase it — this is what removes the steady-state error P alone leaves.
- **Derivative term (D)** — responds to how fast the error is changing; it acts as a brake to reduce overshoot and oscillation.
- **Anti-windup** — clamping the accumulated integral so it can't grow huge while the drone is far from target (which would cause a big overshoot later). Here `INT_CLAMP` bounds it.
- **Dead reckoning** — estimating position by integrating velocity over time (`position += velocity · dt`) when you have no direct position sensor.
- **Visual servoing** — closing a control loop on a camera pixel error (here, yaw until the gate's column equals the image center).
- **Normalized error** — a pixel error divided by half the image width, so it lands in roughly −1…+1 regardless of resolution.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week3_controls/module3_pid/main.py            # all steps, your code
drone sim course/week3_controls/module3_pid/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_pid_altitude.py`** — hold a target height with a full PID controller
2. **`step2_position_hold.py`** — fly a set distance forward (PID on integrated position)
3. **`step3_visual_servo.py`** — yaw with a PID loop to lock onto a glowing gate

## What to expect

Runs the three steps in order: hold 5 m, fly forward, then turn to center and lock onto a gate, then land.

## You're done when

- Step 1: the drone settles to `TARGET_HEIGHT` with **no** lasting gap (tighter than the P-only Module 2) and holds for `HOLD_TIME`.
- Step 2: the drone flies forward about `TARGET_DIST` meters and brakes to a stop instead of overshooting.
- Step 3: the drone turns until a gate is centered (within `CENTER_TOL`), holds, then lands.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| `NameError: name 'image' is not defined` (Step 3) | Capture the frame: `image = drone.camera.get_color_image()`. |
| Altitude oscillates and grows | Integral windup — make sure you clamp `_err_int` to `±INT_CLAMP`, and add some `Kd`. |
| Step 2 overshoots the distance | Use velocity as the derivative term (`err_dot = -velocity[2]`) so it brakes early; raise `KD`. |
| Step 3 jumps between gates and never locks | Track one gate: store `_target_col` and use `gate_nearest_to`, not `gate_nearest_center`, after the first frame. |
| Step never finishes | Check your "settled" timer logic — it must require staying within tolerance for the full hold time. |

## Going further (optional)

- Tune Step 1 for the fastest settle with **no** overshoot. Record the `Kp, Ki, Kd` you land on.
- Step 2 dead-reckons distance from velocity, which drifts. How far off is it after 4 m? Could the downward camera correct the drift?
- Combine Steps 2 and 3: visual-servo the yaw *while* flying forward, so the drone both aims at and approaches the gate.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
