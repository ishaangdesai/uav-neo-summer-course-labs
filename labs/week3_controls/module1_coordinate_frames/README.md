# Week 3 · Module 1 — Coordinate Frames & Dynamics

The rotation math behind a drone: orientation representations and frame transforms. Pure math — no simulator needed.

## What you'll learn

- Euler angles → rotation matrix (aerospace ZYX)
- Rotation matrix → quaternion
- ENU ↔ NED frame conversion
- Sizing rotor thrust for hover

## Key terms

- **Coordinate frame** — a set of x/y/z axes you measure positions and directions against. The same point has different numbers in different frames.
- **Euler angles (roll, pitch, yaw)** — three angles that describe orientation by rotating about three axes in order. Intuitive, but they have edge cases (gimbal lock).
- **Rotation matrix (R)** — a 3×3 matrix that rotates a vector from one frame to another. `R @ v` re-expresses vector `v` in the rotated frame.
- **Quaternion** — a four-number representation of orientation (x, y, z, w) with no gimbal lock. Flight controllers use it internally.
- **ENU** — East-North-Up axis convention (x=East, y=North, z=Up). Common in robotics/graphics.
- **NED** — North-East-Down convention (x=North, y=East, z=Down). Common in aerospace. Converting between ENU and NED swaps and negates axes.
- **Hover thrust** — the total upward force the rotors must produce to cancel gravity: `m · g`. Each of four rotors supplies a quarter of it.

## How to run

```bash
python3 tasks/coordinate_frames.py        # your work (prints PASS/FAIL)
python3 solutions/coordinate_frames.py    # reference
```

## You're done when

The script prints all `[PASS]` and a full score on the final line. A `[FAIL]` prints the array your function returned so you can compare it element-by-element against the expected matrix.

## If it doesn't work

| Symptom | Likely cause |
|---------|--------------|
| Rotation matrix is close but transposed | You multiplied the per-axis rotations in the wrong order. Aerospace ZYX applies yaw, then pitch, then roll. |
| Off by a sign | Check the sign convention of each elementary rotation (sin terms). |
| Quaternion fails | Watch the `w` term and the `0.5 * sqrt(1 + trace)` denominator; guard against dividing by zero. |
| ENU/NED fails | The conversion swaps Y/Z and negates the down axis — write out which axis maps to which before coding. |

## Going further (optional)

- Compose two rotations by multiplying their matrices. Does order matter? Show an example where `R1 @ R2 != R2 @ R1`.
- Convert your quaternion back to Euler angles and confirm you recover the inputs.
- The hover-thrust function assumes level flight. How does the required thrust change when the drone tilts to angle θ? (Hint: the vertical component of thrust must still equal `m·g`.)

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
