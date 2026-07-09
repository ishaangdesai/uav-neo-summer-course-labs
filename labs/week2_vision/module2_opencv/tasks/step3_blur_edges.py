"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Blur & Edge Detection
Averaging blur then a Sobel edge-magnitude image.
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

# -- Course setup: makes the shared `neo_lab` helper importable.
#    You don't need to read or change this block. --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
from tasks.step1_threshold import THRESHOLD_VALUE
import neo_lab

# -- Constants --------------------------------------------------------------
KERNEL_SIZE = 3
HOVER_TIME  = 3.0

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done  = False
image = None
counter = 0

def reset():
    global _timer, _done
    _timer = 0.0
    _done  = False


def update(drone):
    global _timer, _done, image, counter
    if _done:
        return True
    drone.flight.stop()   # hover in place
    ##################################
    #### START PUT CODE HERE #########

    _timer += drone.get_delta_time()

    if counter%10 ==0:
        image = drone.camera.get_downward_image()
        print(_timer)
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary_mask = cv2.threshold(grayscale_image, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
        kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)
        blurred = cv2.blur(binary_mask, (KERNEL_SIZE, KERNEL_SIZE))
        sobel_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
        mean_magnitude = np.mean(magnitude)
        


        if _timer >= HOVER_TIME:
            _done = True
            print(f"Mean edge magnitude: {mean_magnitude}")
    counter+=1

    # GOAL: report the average edge strength in the downward image.
    #
    # Grayscale the image, blur it with a KERNEL_SIZE box filter so single-pixel noise
    # does not dominate, then use a Sobel filter (across and down) to measure how fast
    # brightness changes and combine the two directions into one edge magnitude. Print the
    # mean magnitude. Advance _timer and finish at HOVER_TIME. See the README (Key terms).

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Blur & Edge Detection")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
