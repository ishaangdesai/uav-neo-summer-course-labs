"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 2: Altitude Setpoint Sequence
Chase a sequence of target heights (a step response).
"""

import drone_core
import drone_utils as uav_utils

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
SETPOINTS = [15.0, 5.0, 20.0]   # meters above ground, in order
KP = 0.2
THROTTLE_LIMIT = 0.5
TOL = 0.4
HOLD_TIME = 2.0

# -- Module-level state -----------------------------------------------------
_index = 0
_hold = 0.0
_done = False

def reset():
    global _index, _hold, _done
    _index = 0
    _hold = 0.0
    _done = False


def update(drone):
    global _index, _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########
    TARGET_HEIGHT = SETPOINTS[_index]

    current_height = neo_lab.height(drone)
    height_error = TARGET_HEIGHT - current_height
    throttle = uav_utils.clamp(height_error*KP, -THROTTLE_LIMIT, THROTTLE_LIMIT)
    print(TARGET_HEIGHT)
    print(height_error)
    drone.flight.send_pcmd(0.0, 0.0, 0.0, throttle)
    if abs(height_error) < TOL:
        _hold += drone.get_delta_time()
    if _hold >= HOLD_TIME:
        _index += 1
        _hold = 0.0
    if _index >= len(SETPOINTS):
            _done = True
    


    # GOAL: hold each height in SETPOINTS in turn, moving to the next once you have
    # stayed within TOL of the current one for HOLD_TIME. Finish after the last.
    #
    # This is your Step 1 proportional controller with one change: the target is
    # SETPOINTS[_index] instead of a fixed value, and you advance _index after holding
    # each one. Stop and set _done once _index runs past the end of the list.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Altitude Setpoint Sequence")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
