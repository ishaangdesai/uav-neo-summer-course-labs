"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 3 — Step 3: Orbit a Point
Circle a fixed point at constant radius. A circle demands constant acceleration toward its
center; feeding that acceleration forward is the outer loop of a geometric controller.
Position is dead-reckoned from velocity, so it drifts.
"""

import math

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
CENTER_RIGHT = 0.0
CENTER_FWD = 2.0      # start (0,0) sits on the circle, at its near edge
RADIUS = 2.0
PERIOD = 5.0          # seconds per revolution at full speed
REVOLUTIONS = 2
TARGET_HEIGHT = 3.0
KP_POS = 0.15         # position gain
KV_POS = 0.5          # velocity gain
KA = 0.15             # acceleration feedforward: desired accel -> tilt (geometric outer loop)
ALT_KP = 0.12
ROLL_LIMIT = 0.6
PITCH_LIMIT = 0.6
THROTTLE_LIMIT = 0.5

_W = 2.0 * math.pi / PERIOD     # full-speed angular rate (rad/s)
SPINUP_TIME = PERIOD            # ease from rest up to full rate over the first lap
_SPINUP_ANGLE = 0.5 * _W * SPINUP_TIME   # angle covered while spinning up (half-average rate)
DURATION = SPINUP_TIME + (REVOLUTIONS * 2.0 * math.pi - _SPINUP_ANGLE) / _W
_FD = 0.01                      # finite-difference step for velocity/acceleration

# -- Module-level state -----------------------------------------------------
_t = 0.0
_x = 0.0
_z = 0.0
_max_radial_err = 0.0
_done = False


def _angle(t):
    """Cumulative angle travelled by time t. The rate eases from 0 up to _W over the first
    lap (a smoothstep on angular RATE), so the drone accelerates with the circle instead of
    lurching to full speed from rest. After spin-up the rate is constant.
    """
    if t < SPINUP_TIME:
        x = t / SPINUP_TIME
        return _W * SPINUP_TIME * (x ** 3 - 0.5 * x ** 4)
    return _SPINUP_ANGLE + _W * (t - SPINUP_TIME)


def _point(t):
    """Position (right, forward) on the circle at time t. Angle starts at -90 deg so the
    circle passes through the launch point (0,0)."""
    a = -math.pi / 2.0 + _angle(t)
    return CENTER_RIGHT + RADIUS * math.cos(a), CENTER_FWD + RADIUS * math.sin(a)


def trajectory(t):
    """Desired position, velocity, and acceleration (right, forward) on the circle at time t.

    Velocity and acceleration come from finite-differencing the position, so they stay
    consistent with the spin-up. At full speed the acceleration is centripetal (points at
    the center) -- exactly the feedforward a geometric controller supplies for a curved path.
    """
    pos_r, pos_f = _point(t)
    rp, fp = _point(t + _FD)
    rm, fm = _point(t - _FD)
    vel_r = (rp - rm) / (2.0 * _FD)
    vel_f = (fp - fm) / (2.0 * _FD)
    acc_r = (rp - 2.0 * pos_r + rm) / (_FD * _FD)
    acc_f = (fp - 2.0 * pos_f + fm) / (_FD * _FD)
    return pos_r, pos_f, vel_r, vel_f, acc_r, acc_f


def reset():
    global _t, _x, _z, _max_radial_err, _done
    _t = 0.0
    _x = 0.0
    _z = 0.0
    _max_radial_err = 0.0
    _done = False


def update(drone):
    global _t, _x, _z, _max_radial_err, _done
    if _done:
        return True
    dt = drone.get_delta_time()
    _t += dt
    vx, vy, vz = drone.physics.get_linear_velocity()
    _x += vx * dt
    _z += vz * dt
    pos_r, pos_f, vel_r, vel_f, acc_r, acc_f = trajectory(_t)
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: hold the circle from trajectory() -- constant distance RADIUS from the center.
    #
    # Extend the Step 1 tracking law with a THIRD term, the acceleration feedforward:
    #   command = KP_POS * (desired position - your position)
    #           + KV_POS * (desired velocity - your velocity)
    #           + KA     * (desired acceleration)
    # The acceleration term (acc_r, acc_f) points at the center every frame; commanding it
    # directly is what lets you carve the circle instead of lagging outward into a bigger,
    # looser loop. Use roll for the right axis, pitch for the forward axis, clamp each to
    # its limit. Hold height with ALT_KP, then send_pcmd(...). See the README
    # ("Orbiting a point: the geometric controller") for why the acceleration term matters.

    ###### END PUT CODE HERE #########
    ##################################
    radial = ((_x - CENTER_RIGHT) ** 2 + (_z - CENTER_FWD) ** 2) ** 0.5
    _max_radial_err = max(_max_radial_err, abs(radial - RADIUS))
    if _t >= DURATION:
        drone.flight.stop()
        print(f"[Step 3] Orbit done: {REVOLUTIONS} revolutions, "
              f"max radius error {_max_radial_err:.2f} m")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(TARGET_HEIGHT)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Orbit a Point")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
