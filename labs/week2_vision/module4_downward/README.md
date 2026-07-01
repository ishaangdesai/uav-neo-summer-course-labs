# Week 2 · Module 4 — Downward Camera (Gate Detection)

Find a gate beneath the drone with contour analysis and hover directly over it.

## What you'll learn

- Building a mask of the glowing gate edges
- `cv2.findContours` and picking the largest
- Contour centroid + area
- Visual-servoing with pitch/roll to center on a target

## Key terms

- **Contour** — the outline of a connected white region in a mask, stored as a list of boundary points. `cv2.findContours` returns one per blob.
- **Contour area** — the number of pixels the contour encloses; used to ignore small noise and pick the biggest object.
- **Centroid** — the center point of a contour, `(row, col)`. The `neo_lab`/`uav_utils` helper returns it for you.
- **Visual servoing** — controlling motion directly from what the camera sees: turn the pixel error (how far the target is from center) into a movement command.
- **Pitch** — tilting forward/back, which moves the drone forward/back. Used here together with roll to slide over the gate.
- **Pixel error** — `(target pixel) − (image center)`. Drive it to zero to center the drone over the gate.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week2_vision/module4_downward/main.py            # all steps, your code
drone sim course/week2_vision/module4_downward/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_find_contours.py`** — count the glowing-edge contours below the drone
2. **`step2_largest_object.py`** — locate the largest gate and report its center & area
3. **`step3_track_object.py`** — fly pitch/roll to center the drone over the gate

## What to expect

The drone arms, climbs, finds the gate frame below it, then nudges itself until the gate is centered in the downward image, then lands.

## You're done when

- Step 1 prints a contour count of at least 1 when a gate is below.
- Step 2 prints a center `(row, col)` and an area in the hundreds-to-thousands.
- Step 3 moves until the gate center sits within `CENTER_TOL` pixels of the image center `(240, 320)`, holds for `HOLD_TIME`, then lands.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| `NameError: name 'image' is not defined` | Capture the frame: `image = drone.camera.get_downward_image()`. |
| 0 contours found | The gate isn't under the drone yet, or `V_MIN` is too high. Lower `V_MIN` or raise the launch height. |
| Drone drifts *away* from the gate | Your pitch or roll sign is inverted — flip it. The hint warns the correct sign depends on camera mounting. |
| Never settles (oscillates over the gate) | Lower `MAX_TILT`, or widen `CENTER_TOL` slightly. |

## Going further (optional)

- Use the gate **area** to also hold a target altitude: if the gate looks too small, descend; too big, climb.
- Replace `largest_bright_contour` with the square-shape filter (`neo_lab.largest_gate`) and see whether it rejects the long boundary lines.
- Add a simple derivative term (use `get_linear_velocity`) so the drone brakes as it nears center instead of overshooting.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
