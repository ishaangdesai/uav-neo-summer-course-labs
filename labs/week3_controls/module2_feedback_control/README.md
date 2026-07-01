# Week 3 · Module 2 — Proportional Control (Altitude Hold)

Your first closed-loop controller: proportional control. Warm up on a 1-D math model, then hold the real drone at a target height.

## What you'll learn

- The proportional law: command = Kp · error
- Why a P-only controller leaves a small steady-state error
- Driving the drone's throttle from an altitude error

## Key terms

- **Closed-loop control** — measure the actual state, compare it to the target, and correct. (Open-loop would just command and hope.)
- **Error** — `target − measured`. Here, `target_height − current_height`.
- **Setpoint** — the value you want to reach (the target height).
- **Proportional control (P)** — command proportional to the error: `command = Kp · error`. Far from target → big push; close → gentle.
- **Gain (Kp)** — the tuning knob multiplying the error. Too small is sluggish; too large overshoots or oscillates.
- **Steady-state error** — the small leftover gap a P-only controller settles into: as error shrinks, the command shrinks, until it's too weak to close the last bit (made worse here by the throttle deadband). The integral term in Module 3 fixes this.
- **Step response** — how the system reacts when the target suddenly jumps to a new value.

## How to run

```bash
python3 tasks/p_control.py        # your work (prints PASS/FAIL)
python3 solutions/p_control.py    # reference
```

Then on the simulator:
```bash
drone open_sim                          # launch the sim once
drone sim course/week3_controls/module2_feedback_control/main.py            # all steps, your code
drone sim course/week3_controls/module2_feedback_control/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_altitude_hold.py`** — proportional throttle to hold a target height above ground
2. **`step2_altitude_steps.py`** — chase a sequence of target heights (a step response)

## What to expect

`p_control.py` self-checks with `python3`. The sim steps arm, climb, and hold each target height (settling a little short — that's the P-control lesson).

## You're done when

- `p_control.py` prints all `[PASS]`.
- Step 1: the drone climbs to roughly `TARGET_HEIGHT` (within `TOL`), holds for `HOLD_TIME`, and lands. Settling slightly below the target is expected and correct.
- Step 2: the drone visibly steps through each height in `SETPOINTS` in order, holding each before moving on.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| Drone shoots up and oscillates | `KP` too high, or you forgot to clamp the throttle to `THROTTLE_LIMIT`. |
| Drone barely moves | `KP` too low, or the error is below the throttle deadband (~0.05). |
| Step never completes | You aren't accumulating `_hold` while within `TOL`, or never reset it when you leave `TOL`. |
| Step 2 skips heights | Reset `_hold = 0.0` when you advance `_index`. |

## Going further (optional)

- Measure the steady-state error: print `TARGET_HEIGHT − height` once settled. How does it change if you double `KP`?
- Add a feed-forward term: a constant throttle that roughly cancels gravity's pull, so P only handles the remainder. Does the steady-state error shrink?
- Plot (or log) height vs. time for a step and identify the rise time and any overshoot.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
