# Week 2 · Module 3 — Linear Regression (Edge Following)

Fit a straight line to detected pixels with least squares, then use it to steer. The gates glow, so we follow a bright gate edge from the downward camera.

## What you'll learn

- Thresholding bright pixels and `np.argwhere`
- Least-squares line fit (`np.polyfit`)
- Turning a pixel offset into a steering (roll) command

## Key terms

- **Linear regression** — finding the straight line that best fits a cloud of points.
- **Least squares** — the rule that picks "best": the line that minimizes the sum of squared vertical distances from the points. `np.polyfit(xs, ys, 1)` does it for you and returns slope `m` and intercept `b`.
- **`np.argwhere(mask)`** — returns the `(row, col)` coordinates of every nonzero pixel in a mask. Rows are the y-axis, columns are the x-axis.
- **Pixel offset** — how far the detected edge is from the center column of the image, in pixels. Positive means the edge is to the right.
- **Roll** — tilting left/right, which makes the drone strafe sideways. Used here to slide back over the edge.
- **Proportional steering** — making the correction proportional to the offset: far off → strong correction, nearly centered → gentle.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week2_vision/module3_linear_regression/main.py            # all steps, your code
drone sim course/week2_vision/module3_linear_regression/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_detect_line.py`** — find and count the bright edge pixels
2. **`step2_fit_line.py`** — fit y = m·x + b to those pixels
3. **`step3_follow_line.py`** — fly forward while rolling to keep the edge centered

## What to expect

Steps 1-2 hover and report; Step 3 flies forward, steering to stay over a bright edge for a fixed time.

## You're done when

- Step 1 prints a bright-pixel count in the thousands while a gate edge is in view.
- Step 2 prints a slope `m` and intercept `b` (any finite numbers — the exact values depend on what the camera sees).
- Step 3 flies forward for `FOLLOW_TIME` seconds, visibly correcting left/right to stay over the edge, then lands.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| `NameError: name 'image' is not defined` | Capture the frame: `image = drone.camera.get_downward_image()`. |
| `np.polyfit` raises or returns `nan` | You passed too few points, or columns/rows are swapped. Convert to float and use column as x, row as y. |
| Drone rolls the wrong way and loses the edge | Flip the sign of your roll command (camera/strafe direction may be inverted). |
| Step 3 steers wildly | Lower `MAX_ROLL`, or require a minimum pixel count before steering so noise doesn't dominate. |

## Going further (optional)

- A nearly-vertical edge makes `y = m·x + b` blow up (slope → ∞). Detect that case and fit `x = m·y + b` instead.
- Use the fitted line's *angle* (not just the offset) to also yaw the drone so it flies along the edge, not just over it.
- Reject outliers: refit after dropping points far from the first fit (a one-step RANSAC).

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
