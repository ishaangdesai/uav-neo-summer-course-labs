"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 1: PID Altitude Hold
Hold a target height with a full PID controller (P + I + D).
Heights are measured above the ground sampled at launch.
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
TARGET_HEIGHT = 15.0
KP = 0.1
KI = 0.03
KD = 0.03
INT_CLAMP = 3.0      # anti-windup limit on the integral
THROTTLE_LIMIT = 0.5
TOL = 0.3
HOLD_TIME = 3.0

# -- Module-level state -----------------------------------------------------
_err_int = 0.0
_prev_err = 0.0
_hold = 0.0
_done = False

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Return the PID controller output from the three gain terms (see README, Key terms)."""
    ##################################
    #### START PUT CODE HERE #########

    output = err*kp + err_int*ki + err_dot*kd
    ###### END PUT CODE HERE #########
    ##################################
    return output

def reset():
    global _err_int, _prev_err, _hold, _done
    _err_int = 0.0
    _prev_err = 0.0
    _hold = 0.0
    _done = False


def update(drone):
    global _err_int, _prev_err, _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########
    _err = TARGET_HEIGHT - neo_lab.height(drone)
    _err_int += _err*drone.get_delta_time()
    # print("added error", _err*drone.get_delta_time())
    _err_int = uav_utils.clamp(_err_int, -INT_CLAMP, INT_CLAMP)
    # print(_err-_prev_err)
    _err_dot = (_err - _prev_err)/drone.get_delta_time()
    _prev_err = _err
    # print(_err)
    throttle = uav_utils.clamp(pid_control(_err, _err_int, _err_dot, KP, KI, KD), -THROTTLE_LIMIT, THROTTLE_LIMIT)
    # print("throttle: ", throttle)
    # print("integral: ", _err_int)
    # print("derivative: ", _err_dot)

    drone.flight.send_pcmd(0.0, 0.0, 0.0, throttle)
    if abs(_err) < TOL:
        _hold += drone.get_delta_time()
        _err_int = 0.0
        if _hold >= HOLD_TIME:
            _done = True
    else:
        _hold = 0.0

    # Implement pid_control() above, then use it to drive the drone to TARGET_HEIGHT.
    # Track the integral of the height error (with anti-windup at INT_CLAMP) and its
    # derivative yourself. Throttle is a vertical-velocity command; clamp it to
    # +/-THROTTLE_LIMIT. Finish (set _done) once the height stays within TOL for
    # HOLD_TIME. See the README (Key terms) for the PID law and anti-windup.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 1: PID Altitude Hold")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
