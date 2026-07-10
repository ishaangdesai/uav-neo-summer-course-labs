"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Seek the Gate
Yaw to center the largest cyan gate, then fly forward toward it.
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
import neo_lab

# -- Constants --------------------------------------------------------------
MIN_AREA   = 400
COL_CENTER = 320
MAX_YAW    = 0.3        # yaw authority for centering
APPROACH_PITCH = 0.2    # forward speed once centered
CENTER_TOL = 20         # px error to count as centered
SEARCH_YAW = 0.05        # spin slowly when no gate is seen
TARGET_WIDTH = 170      # gate this wide (px) => close enough

# -- Module-level state -----------------------------------------------------
_done = False

def reset():
    global _done, _timer
    _timer = 0.0
    _done = False


def update(drone):
    global _done, _timer
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: spin until a cyan gate is found, yaw to center it, then fly forward until
    # it fills the view (bounding-box width >= TARGET_WIDTH).
    #


    image = drone.camera.get_color_image()  # get the forward camera color image
    largest_gate = neo_lab.largest_cyan_gate(image, MIN_AREA)  # find the largest cyan gate contour
    if largest_gate is None:
        drone.flight.send_pcmd(0.0, 0.0, SEARCH_YAW, 0.0)  # no gate -> spin slowly
        print("Searching for gate...")
        return False
    x, y, w, h = cv2.boundingRect(largest_gate)
    horizontal_offset = (x + w / 2) - COL_CENTER  # box center column vs. COL_CENTER
    vertical_offset = (y + h / 2) - 240
    print("horizontal offset ", horizontal_offset, "vertical offset ", vertical_offset, "bounding box width ", w)

    yaw = uav_utils.clamp(horizontal_offset / COL_CENTER, -MAX_YAW, MAX_YAW)  # yaw authority for centering

    thrust = uav_utils.clamp(-vertical_offset / COL_CENTER, -.3, .3)  # throttle to reach height 240px (center of image)
    print("thrust ", thrust)

    pitch = APPROACH_PITCH if abs(horizontal_offset) < CENTER_TOL else 0.0  # forward speed once centered
    drone.flight.send_pcmd(pitch, 0.0, yaw, thrust)  # send pitch/yaw command
    if w >= TARGET_WIDTH and abs(horizontal_offset) < CENTER_TOL and abs(vertical_offset) < CENTER_TOL:
        print("Reached gate: bounding box width ", w, "px >= TARGET_WIDTH ", TARGET_WIDTH, "px")
        _done = True  # finish when the gate fills the view

    # Tools: drone.camera.get_color_image(); neo_lab.largest_cyan_gate(image, MIN_AREA);
    #        cv2.boundingRect(contour) -> (x, y, w, h); uav_utils.clamp(...);
    #        drone.flight.send_pcmd(pitch, roll, yaw, throttle).
    #
    # No gate in view -> turn slowly (SEARCH_YAW) to find one. With a gate, the box
    # center column vs. COL_CENTER gives a yaw error; only add forward pitch once it is
    # roughly centered (within CENTER_TOL) so you turn toward it before chasing. The box
    # grows as you approach; stop when w reaches TARGET_WIDTH.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Seek the Gate")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            #_drone.flight.land()
            _drone.flight.stop()  # hover in place when done

    _drone.set_start_update(start, _update)
    _drone.go()
