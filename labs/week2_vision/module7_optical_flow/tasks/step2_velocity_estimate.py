"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 7 — Step 2: Velocity from Optical Flow
Turn the tracked features' average motion into a velocity estimate and compare it
against the drone's true velocity. The flow is in pixels per processed interval;
converting to meters/second needs the ground footprint of one pixel (grows with
altitude) and the time between processed frames.
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
IMAGE_WIDTH = 640
HFOV_TAN = 1.0         # tan(half of a 90 deg horizontal field of view)
PROBE_PITCH = 0.12     # forward drift to create measurable flow
RUN_TIME = 6.0
SKIP = 2               # do the vision work every Nth frame
MIN_PTS = 20
HOVER_TIME = 4.0       # time to hover before stopping and printing results
FEATURE_PARAMS = dict(maxCorners=80, qualityLevel=0.01, minDistance=8, blockSize=7)
LK_PARAMS = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# -- Module-level state -----------------------------------------------------
_prev_gray = None
_prev_pts = None
_timer = 0.0
_interval = 0.0        # time accumulated since the last processed frame
_frame = 0
_done = False

def reset():
    global _prev_gray, _prev_pts, _timer, _interval, _frame, _done
    _prev_gray = None
    _prev_pts = None
    _timer = 0.0
    _interval = 0.0
    _frame = 0
    _done = False


def update(drone):
    global _prev_gray, _prev_pts, _timer, _interval, _frame, _done
    if _done:
        return True
    
    drone.flight.send_pcmd(PROBE_PITCH, 0.0, 0.0, 0.0)  # keep the drone drifting forward
    _timer += drone.get_delta_time()  # keep track of how long we've been running
    _interval += drone.get_delta_time()  # accumulate time since last processed frame
    _frame += 1

    if _frame % SKIP != 0:  # only do the vision work every SKIP-th frame
        return False

    image = drone.camera.get_downward_image()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if _prev_pts is None or len(_prev_pts) < MIN_PTS:
        # detect new features to track
        _prev_pts = cv2.goodFeaturesToTrack(gray, **FEATURE_PARAMS)
        _prev_gray = gray
        return False
    
    new_coords, status, _ = cv2.calcOpticalFlowPyrLK(_prev_gray, gray, _prev_pts, None, **LK_PARAMS)
    if new_coords is not None and status is not None:
        # keep only the points that were successfully tracked
        good_new = new_coords[status.flatten() == 1]
        good_old = _prev_pts[status.flatten() == 1]
        
        if len(good_new) > 0:
            # calculate the average magnitude of the displacement
            disp = good_new - good_old
            dx = np.mean(disp[:, 0, 0])
            dz = np.mean(disp[:, 0, 1])
        
        _prev_pts = good_new.reshape(-1, 1, 2)
        _prev_gray = gray
    estimated_velocity = 0.0
    if _interval > 0:
        # convert the average pixel displacement to meters/second
        height = neo_lab.height(drone)
        pixel_footprint = (2 * height * HFOV_TAN) / IMAGE_WIDTH  # meters per pixel
        evx= (dx * pixel_footprint) / _interval  # m/s
        evz= (dz * pixel_footprint) / _interval  # m/s
        _interval = 0.0  # reset the interval timer after processing
    if _timer >= HOVER_TIME:
        drone.flight.stop()
        vx, vy, vz = drone.physics.get_linear_velocity()  # forward velocity in m/s
        print(f"[Step 2] Estimated  x velocity = {evx:.3f} m/s, Estimated z velocity = {evz:.3f} m/s. x velocity = {vx:.3f} m/s, z velocity = {vz:.3f} m/s")
        _done = True




    ##################################
    #### START PUT CODE HERE #########

    # GOAL: print an estimated horizontal velocity from optical flow next to the true
    # velocity, so you can see how well vision tracks motion.
    #
    # Tools: drone.camera.get_downward_image(); neo_lab.height(drone);
    #        drone.physics.get_linear_velocity(); drone.get_delta_time(); send_pcmd(...);
    #        plus the sparse optical-flow tracking you built in Step 1.
    #
    # Every frame: drift (PROBE_PITCH), add dt to _timer AND to _interval, and _frame += 1.
    # Only every SKIP-th frame: track corner points (sparse flow, like Step 1) and average
    # the kept points' displacement in pixels. Convert that to meters/second: one pixel's
    # ground footprint grows with height and the camera's field of view (use HFOV_TAN and
    # IMAGE_WIDTH), and divide by _interval (the time between PROCESSED frames, not one dt);
    # then reset _interval. The camera moves opposite the scene flow (sign flip). Finish at
    # RUN_TIME, printing the estimate vs. true velocity. See the README (Key terms).

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Velocity from Optical Flow")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
