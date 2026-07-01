# Week 2 · Module 2 — OpenCV (Thresholding & Morphology)

Core image-processing operations applied to the drone's live downward camera.

## What you'll learn

- Grayscale + binary thresholding (`cv2.threshold`)
- Cleaning a mask with erosion/dilation (morphological opening)
- Blurring and Sobel edge detection

## Key terms

- **Grayscale** — a one-channel image where each pixel is a single brightness value 0–255 instead of three color values.
- **Threshold** — turn a grayscale image into black-and-white by keeping pixels above a cutoff and zeroing the rest. The result is a **binary mask**.
- **Binary mask** — an image whose pixels are either 0 or 255, marking which pixels belong to the thing you care about.
- **Erosion** — shrinks white regions; removes small specks.
- **Dilation** — grows white regions; fills small holes.
- **Opening** — erosion *then* dilation. It deletes small noise while leaving the big shapes roughly their original size.
- **Kernel** — the small window (here `KERNEL_SIZE`×`KERNEL_SIZE`) a morphology or blur operation slides over the image.
- **Sobel filter** — measures how fast brightness changes in one direction; large values mark edges.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week2_vision/module2_opencv/main.py            # all steps, your code
drone sim course/week2_vision/module2_opencv/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_threshold.py`** — threshold the camera image and measure the bright fraction
2. **`step2_morphology.py`** — erode then dilate to remove speckle noise
3. **`step3_blur_edges.py`** — average-blur then a Sobel edge-magnitude image

## What to expect

The drone arms, climbs, and hovers while each step processes a frame and prints a measurement. It does not fly around.

## You're done when

Each step prints its measurement and the module advances:
- Step 1 prints a white-fraction between 0 and 1 (the bright gate edges are a small fraction of the floor).
- Step 2 prints a positive "pixels removed" count (opening deleted some speckle).
- Step 3 prints a mean edge magnitude greater than 0.
After Step 3 the drone lands.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| `NameError: name 'image' is not defined` | Grab a frame first: `image = drone.camera.get_downward_image()`. |
| White fraction is 0 or 1 | Your `THRESHOLD_VALUE` is too high or too low for the scene; print `gray.max()` to see the real range. |
| Step never finishes | You aren't incrementing `_timer` (`_timer += drone.get_delta_time()`), so it never reaches `HOVER_TIME`. |
| `cv2.Sobel` error | Use `cv2.CV_64F` as the output depth so negative gradients aren't clipped. |

## Going further (optional)

- Replace the fixed threshold with **Otsu's method** (`cv2.threshold(..., cv2.THRESH_OTSU)`) and compare the white fraction.
- Try `cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)` (dilation then erosion) and describe the difference from opening.
- Overlay the Sobel edges back onto the original by thresholding the magnitude — what threshold cleanly isolates the gate frame?

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
