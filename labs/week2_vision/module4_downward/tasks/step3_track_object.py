"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Center Over the Gate
Visual-servo the drone to hover directly above the gate frame.
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
V_MIN = 200
MIN_AREA = 300
MAX_TILT = 0.18      # pitch/roll authority
CENTER_TOL = 20      # pixels considered 'centered'
HOLD_TIME = 2.0      # seconds to stay centered before done
ROW_CENTER = 240
COL_CENTER = 320
kP = 0.005          # proportional gain for pitch/roll control

# -- Module-level state -----------------------------------------------------
_hold = 0.0
_done = False

def reset():
    global _hold, _done
    _hold = 0.0
    _done = False


def update(drone):
    global _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########


    image = drone.camera.get_downward_image()
    largest = neo_lab.largest_bright_contour(image, V_MIN, MIN_AREA)
    y, x = uav_utils.get_contour_center(largest) if largest is not None else (320, 240)
    print(x, y)
    forward_adjustment = y-ROW_CENTER
    lateral_adjustment = x-COL_CENTER
    print(lateral_adjustment, forward_adjustment)
    pitch = uav_utils.clamp(-kP * forward_adjustment, -MAX_TILT, MAX_TILT)
    roll = uav_utils.clamp(kP * lateral_adjustment, -MAX_TILT, MAX_TILT)
    print(f"Pitch: {pitch:.3f}, Roll: {roll:.3f}")
    if abs(forward_adjustment) < CENTER_TOL and abs(lateral_adjustment) < CENTER_TOL:
        print("done")
        _hold += drone.get_delta_time()
        drone.flight.send_pcmd(0, 0, 0, 0)

        if _hold > HOLD_TIME:
            drone.flight.stop()
            _done = True

    drone.flight.send_pcmd(pitch, roll, 0, 0)




    # GOAL: move with pitch/roll until the gate sits in the middle of the downward
    # camera, hold that for HOLD_TIME, then finish.
    #
    # Tools: drone.camera.get_downward_image(); neo_lab.largest_bright_contour(image,
    #        V_MIN, MIN_AREA) -> contour or None; uav_utils.get_contour_center(c) ->
    #        (row, col); uav_utils.clamp(...); drone.flight.send_pcmd(...).
    #        The image center is (ROW_CENTER, COL_CENTER).
    #
    # Drive roll from the column error and pitch from the row error, each scaled to
    # MAX_TILT. Which sign centers the drone depends on how the camera is mounted --
    # pick a sign, watch which way it runs, and flip it if it diverges. With no gate
    # in view, hold position and reset your centered timer.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Center Over the Gate")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
