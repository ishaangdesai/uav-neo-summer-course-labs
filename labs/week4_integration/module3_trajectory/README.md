# Week 4 · Module 3 — Trajectory Tracking

Fly a *path*, not a pin. In Module 1 you flew to a waypoint and stopped; in Module 2 you
chained waypoints into a square, braking at every corner. A race drone never stops at a gate —
it carries a smooth line through the whole course. This module builds that line as a
**trajectory** (a position for every instant in time) and a controller that rides it.

This is the drone-racing lab from MIT's VNAV course, rebuilt for the UAV Neo sim: generate a
smooth trajectory through the gates, then track it. The heavy trajectory-optimization math of
the original is replaced with a cubic **spline** you can build by hand.

## What you'll learn

- **Trajectory vs waypoint** — why "be here by time *t*" is a stronger command than "reach here
  eventually," and what that buys a racing drone.
- **Tracking with position + velocity gains** — a controller that follows a *moving* target by
  matching both where it is and how fast it is going (the feedforward idea from Week 3).
- **Time-scaling (smoothstep)** — starting and stopping a segment at rest so there is no jerk.
- **Cubic splines (Hermite)** — stitching gates into one continuous curve with matched slopes,
  so the path never kinks.
- **The racing line** — why flying *through* gates without stopping is both smoother and faster.
- **Acceleration feedforward (the geometric controller)** — why holding a *curved* path needs a third term beyond position and velocity, and how a drone turns a desired acceleration into a tilt.

## How it works

**A waypoint is a place; a trajectory is a place *for every instant*.** Write it as `p(t)`: at
`t = 0` you are at the start, at `t = DURATION` you are at the goal, and in between `p(t)` names
exactly where you should be right now. Because `p(t)` is a function of time you can also read off
its **velocity** `p'(t)` — how fast the target itself is moving. Those two numbers, position and
velocity, are everything the controller needs.

**Tracking: two gains (Step 1).** To follow a moving target, each axis uses two terms:

```
command = KP_POS · (desired position − your position)   +   KV_POS · (desired velocity − your velocity)
```

The first term is ordinary proportional control — pull toward where you should be. The second is
the **feedforward** from Week 3: the target is already moving at `desired velocity`, so command
that speed directly instead of waiting for a position error to build. With only the first term the
drone always trails a moving target; the second term is what lets it ride alongside. `KP_POS` is
the **position gain** (how hard you pull toward the path) and `KV_POS` is the **velocity gain**
(how closely you match its speed) — the same two knobs a full quadrotor controller exposes.

**Smoothstep: start and stop at rest (Step 1).** If `p(t)` moved at constant speed, the drone
would have to jump from standstill to full speed instantly — a jerk. Instead the segment uses a
**time-scaling** `s(u)`, with `u = t/DURATION` going 0→1, shaped so its *slope* is zero at both
ends: `s(u) = 3u² − 2u³`. Position is `start + s·(goal − start)`, so the drone eases out of the
start and eases into the goal. This one cubic is the whole trajectory for a single segment.

**Splines: one smooth curve through many gates (Step 2).** A course is a list of gates. Draw a
straight line between each pair and the path kinks at every gate — the drone would have to stop and
turn. A **cubic Hermite spline** fixes this: on each segment (fraction `s` from 0 to 1 between gate
`p0` and gate `p1`) the position is a blend of the two endpoints *and* a chosen slope (tangent) at
each end:

```
p(s) = h00(s)·p0 + h10(s)·m0 + h01(s)·p1 + h11(s)·m1

h00 = 2s³ − 3s² + 1     (weight on the start point)
h10 = s³ − 2s² + s      (weight on the start tangent m0)
h01 = −2s³ + 3s²        (weight on the end point)
h11 = s³ − s²           (weight on the end tangent m1)
```

At `s=0` this is exactly `p0`; at `s=1` exactly `p1`; in between it curves according to the
tangents. Pick the tangent at each gate to point from the *previous* gate to the *next* one
(a **Catmull-Rom** tangent, `m_i = ½(p_{i+1} − p_{i−1})`), and neighbouring segments leave and
enter each gate at the same slope — so the full path is smooth, with no kinks. You implement the
`hermite` blend; the tangents and the per-gate bookkeeping are provided.

**Why not stop at each gate?** Stopping means decelerating to zero and re-accelerating — slow, and
it wastes the drone's momentum. Carrying a smooth line through the gates keeps speed up and control
inputs gentle. That trade — hold the line, don't stop — is the core idea of drone racing.

**Orbiting a point: the geometric controller (Step 3).** A straight segment needs almost no
acceleration; a *circle* needs it constantly. To travel a circle of radius `R` at speed `v`, an
object must accelerate toward the center at `v²/R` the whole way — the **centripetal** acceleration.
A controller that only knows position and velocity feels this as a lag: it drifts *outward* into a
bigger, looser loop, because nothing is commanding the inward pull until an error has already opened
up. The fix is a third feedforward term — the **acceleration**:

```
command = KP_POS · (desired position − your position)
        + KV_POS · (desired velocity − your velocity)
        + KA     · (desired acceleration)
```

The desired acceleration on a circle points at the center every instant, so `KA · acceleration`
tilts the drone inward *before* it drifts out — and it carves the circle instead of spiraling wide.
This is the heart of a **geometric controller** (the kind used in MIT's VNAV labs): it converts a
desired acceleration into the thrust vector — and therefore the tilt — the drone should hold. The
full version outputs individual motor thrusts and reasons about orientation as a rotation matrix;
here the sim takes a tilt command directly, so we implement its outer loop — desired acceleration →
tilt — which is the part that fits this drone.

**Easing in (spin-up).** The drone starts at rest, but a circle at full speed demands an instant
sideways velocity. Jumping straight to it makes the drone lurch and swing wide on the first quarter
lap. So the orbit **spins up**: the angular rate eases from zero to full over the first lap, so the
drone accelerates *with* the circle. There is a hard ceiling on how fast you can orbit — the
centripetal acceleration `v²/R` can only come from tilt, and tilt is limited (`ROLL_LIMIT`), so past
some speed the drone physically cannot hold the radius. Shrinking `PERIOD` (a faster orbit) is what
eventually runs into that wall.

## Key terms

- **Trajectory** — a target position given for every instant, `p(t)`, together with its velocity `p'(t)`. Contrast a waypoint, which is a single fixed point.
- **Feedforward tracking** — commanding the target's own velocity (`KV_POS · desired velocity`) so the controller leads a moving reference instead of lagging it.
- **Position gain / velocity gain** — `KP_POS` pulls the drone toward the path; `KV_POS` matches its speed. Raise `KP_POS` for tighter tracking, raise `KV_POS` to damp overshoot.
- **Smoothstep (time-scaling)** — `s(u) = 3u² − 2u³`; zero slope at both ends, so a segment begins and ends at rest without a jerk.
- **Cubic Hermite spline** — a curve on each segment built from two endpoints and two tangents via the four basis functions `h00, h10, h01, h11`.
- **Catmull-Rom tangent** — the slope at a gate chosen as `½(next − previous)`, which makes adjacent spline segments join smoothly.
- **Racing line** — a single continuous trajectory carried through all the gates without stopping.
- **Centripetal acceleration** — the inward acceleration `v²/R` needed to travel a circle of radius `R` at speed `v`. On the orbit it always points at the center.
- **Acceleration feedforward** — the third control term, `KA · desired acceleration`, that commands the path's own acceleration so the drone holds a curve instead of drifting outward.
- **Geometric controller** — a controller that turns a desired acceleration into the thrust vector (here, the tilt) the drone should hold. Step 3 implements its outer loop; VNAV's full version also solves for individual motor thrusts.
- **Spin-up** — easing the orbit's angular rate from zero to full over the first lap so the drone accelerates with the circle rather than lurching to full speed from rest.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week4_integration/module3_trajectory/main.py            # all steps, your code
drone sim course/week4_integration/module3_trajectory/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_timed_segment.py`** — track one smooth timed segment from start to a goal (write the tracking controller)
2. **`step2_smooth_course.py`** — build a cubic spline through five gates and fly the whole course without stopping (write the `hermite` blend)
3. **`step3_orbit.py`** — circle a fixed point with acceleration feedforward, the geometric controller's outer loop (write the three-term controller)

## What to expect

Runs the three steps in order: glide smoothly out to a point and settle, sweep through a
five-gate slalom in one continuous line, then orbit a point for two laps, and land.

## You're done when

- Step 1: the drone eases from the start to `(GOAL_RIGHT, GOAL_FWD)` over `DURATION` seconds — starting and stopping gently, not lurching — and reports a small **max tracking error**.
- Step 2: the drone flies through all `len(GATES) − 1` segments in one smooth line (each `Through gate k` prints in order) and finishes near the last gate with a small max tracking error.
- Step 3: the drone eases into a circle and holds it for `REVOLUTIONS` laps, reporting a small **max radius error** (how far its distance from the center strays from `RADIUS`).

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| Step 1 always trails behind the target | You dropped the velocity term — add `KV_POS · (desired velocity − your velocity)` so you feed the trajectory's speed forward. |
| Step 1 lurches at the start | Make sure you are reading `pos_r, pos_f, vel_r, vel_f` from `trajectory(_t)` each frame, not driving straight to the goal. |
| Step 2 flies to the first gate and stops / path has corners | Check your `hermite` blend: at `s=0` it must return `p0` and at `s=1` return `p1`. If it returns only `p0`, every segment collapses to a point. |
| Step 2 overshoots wildly between gates | The tangents scale with gate spacing; keep the gates within a few meters of each other, or lengthen `SEG_TIME` to slow the pass. |
| Drifts off the course near the end | Position is dead-reckoned from velocity and drifts (same as Modules 1–2). See "Going further." |
| Step 3 spirals outward into a bigger circle | You left out the acceleration term — add `KA · desired acceleration` so the drone is pulled toward the center before it drifts out. |
| Step 3 can't hold the radius even with all three terms | The orbit is too fast for the tilt limit (`v²/R` exceeds what `ROLL_LIMIT` can supply). Raise `PERIOD` to slow it down. |

## Going further (optional)

- Position here is dead-reckoned (`position += velocity · dt`) and drifts. `neo_lab.world_position(drone)` returns the sim's true position on a new enough build — swap it in and watch the tracking error shrink.
- Add a third axis: make the gates `(right, up, forward)` and spline the height too, so the course climbs and dives.
- Tune for speed: shorten `SEG_TIME` until the drone can no longer hold the line. Where does it break — the controller, or the dead-reckoning drift?
- The real VNAV lab replaces this hand-built spline with a **minimum-snap** trajectory: instead of picking tangents by a rule, it solves for the polynomial through all gates that minimizes total *snap* (the fourth derivative of position), which is what keeps a quadrotor's motors from saturating. Same idea — a smooth timed path — found by optimization instead of by hand.
- Orbit tuning (Step 3): run it once with `KA = 0` and watch the drone spiral outward, then restore `KA` — that gap is what the acceleration feedforward buys. Then shrink `PERIOD` toward a faster orbit until the radius blows up, and read `v²/R` against the tilt limit to see why.
- Make the orbit point a *camera* target: yaw the drone to face the center as it circles (reuse the heading control from Module 4), turning the orbit into an inspection sweep of the point.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
