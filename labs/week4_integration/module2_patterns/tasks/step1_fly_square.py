"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 2 — Step 1: Fly a Square
A flight path is just a list of waypoints visited in order. Here the list forms a
square. Reuse the waypoint controller from Module 1, one corner at a time, advancing
to the next corner when you reach the current one.
"""

import drone_core
import drone_utils as uav_utils
import math

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
SIDE = 3.0
# Corners as (right, forward) meters from the start, traced as a square.
WAYPOINTS = [(0.0, SIDE), (SIDE, SIDE), (SIDE, 0.0), (0.0, 0.0)]
TARGET_HEIGHT = 3.0
KP_POS = 0.3
KD_POS = 0.02
ALT_KP = 0.12
ROLL_LIMIT = 0.25
PITCH_LIMIT = 0.25
THROTTLE_LIMIT = 0.5
WP_TOL = 0.6           # meters from a corner counted as reached

# -- Module-level state -----------------------------------------------------
_x = 0.0
_z = 0.0
_wp = 0
_done = False

def reset():
    global _x, _z, _wp, _done
    _x = 0.0
    _z = 0.0
    _wp = 0
    _done = False


def update(drone):
    global _x, _z, _wp, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########
    TARGET_FWD = WAYPOINTS[_wp][1]
    TARGET_RIGHT = WAYPOINTS[_wp][0]

    
    velocity = drone.physics.get_linear_velocity()
    dt = drone.get_delta_time()
    _z += velocity[2] * dt                     # z axis points forward
    _x += velocity[0] * dt                     # x axis points right
    z_error = TARGET_FWD - _z
    x_error = TARGET_RIGHT - _x
    print(_wp)
    z_err_dot = -velocity[2]        # d(error)/dt = -forward velocity (clean derivative term)
    x_err_dot = -velocity[0]

    
    pitch = uav_utils.clamp(KP_POS*z_error+z_err_dot*KD_POS,
                            -PITCH_LIMIT, PITCH_LIMIT)
    roll = uav_utils.clamp(KP_POS*x_error+x_err_dot*KD_POS,
                           -PITCH_LIMIT, PITCH_LIMIT)


    throttle = uav_utils.clamp(ALT_KP * (TARGET_HEIGHT - neo_lab.height(drone)),
                               -THROTTLE_LIMIT, THROTTLE_LIMIT)
    drone.flight.send_pcmd(pitch, roll, 0, throttle)
    if math.fabs(x_error)<WP_TOL and math.fabs(z_error)<WP_TOL and TARGET_HEIGHT - neo_lab.height(drone)<WP_TOL:
        _wp+=1
    if _wp>3:
        _done = True


    # GOAL: visit each corner in WAYPOINTS in order, then finish.
    #
    # Tools: drone.physics.get_linear_velocity(); drone.get_delta_time();
    #        neo_lab.height(drone); uav_utils.clamp(...); drone.flight.send_pcmd(...).
    #
    # Integrate vx, vz into (_x, _z) like Module 1. If _wp has passed the last
    # waypoint, stop and finish. Otherwise steer toward WAYPOINTS[_wp] with the same
    # PD command per axis (roll for right, pitch for forward, throttle for height).
    # When you are within WP_TOL of the current corner on both axes, advance _wp += 1.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Fly a Square")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
