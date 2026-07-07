"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 3 — Step 1: Track a Timed Segment  (SOLUTION)
Follow a smooth position-vs-time trajectory from start to a goal, tracking both the moving
position and its velocity (feedforward). Position is dead-reckoned from velocity, so it drifts.
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
GOAL_RIGHT = 2.0
GOAL_FWD = 6.0
TARGET_HEIGHT = 3.0
DURATION = 5.0        # seconds to fly the segment
KP_POS = 0.15         # position gain: pull toward where you should be now
KV_POS = 0.5          # velocity gain: match the trajectory's speed (feedforward + damping)
ALT_KP = 0.12
ROLL_LIMIT = 0.25
PITCH_LIMIT = 0.25
THROTTLE_LIMIT = 0.5

# -- Module-level state -----------------------------------------------------
_t = 0.0
_x = 0.0
_z = 0.0
_max_err = 0.0
_done = False


def trajectory(t):
    """Desired (right, forward) position and velocity at time t along the segment.

    Uses a smoothstep time-scaling s(u)=3u^2-2u^3 (u=t/DURATION): it starts and ends at
    rest (zero velocity), so there is no jerk at takeoff or arrival. This is the segment's
    "trajectory" -- a path parameterized by time, not a single fixed waypoint.
    """
    u = min(max(t / DURATION, 0.0), 1.0)
    s = 3.0 * u * u - 2.0 * u * u * u
    s_dot = (6.0 * u - 6.0 * u * u) / DURATION
    pos_right = GOAL_RIGHT * s
    pos_fwd = GOAL_FWD * s
    vel_right = GOAL_RIGHT * s_dot
    vel_fwd = GOAL_FWD * s_dot
    return pos_right, pos_fwd, vel_right, vel_fwd


def reset():
    global _t, _x, _z, _max_err, _done
    _t = 0.0
    _x = 0.0
    _z = 0.0
    _max_err = 0.0
    _done = False


def update(drone):
    global _t, _x, _z, _max_err, _done
    if _done:
        return True
    dt = drone.get_delta_time()
    _t += dt
    vx, vy, vz = drone.physics.get_linear_velocity()
    _x += vx * dt
    _z += vz * dt
    pos_r, pos_f, vel_r, vel_f = trajectory(_t)
    roll = uav_utils.clamp(KP_POS * (pos_r - _x) + KV_POS * (vel_r - vx),
                           -ROLL_LIMIT, ROLL_LIMIT)
    pitch = uav_utils.clamp(KP_POS * (pos_f - _z) + KV_POS * (vel_f - vz),
                            -PITCH_LIMIT, PITCH_LIMIT)
    throttle = uav_utils.clamp(ALT_KP * (TARGET_HEIGHT - neo_lab.height(drone)),
                               -THROTTLE_LIMIT, THROTTLE_LIMIT)
    drone.flight.send_pcmd(pitch, roll, 0, throttle)
    err = ((pos_r - _x) ** 2 + (pos_f - _z) ** 2) ** 0.5
    _max_err = max(_max_err, err)
    if _t >= DURATION:
        drone.flight.stop()
        print(f"[Step 1] Segment done: right={_x:.2f} forward={_z:.2f} m, "
              f"max tracking error {_max_err:.2f} m")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(TARGET_HEIGHT)

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Track a Timed Segment")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
