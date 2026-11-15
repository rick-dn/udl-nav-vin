# How it works

## The pipeline

```
Gazebo (warehouse + iris_with_vio_camera)
  |-- camera 30 Hz ----> ros_gz_bridge ----> /vio/camera/image ----+
  |-- VIO IMU 200 Hz --> gz_imu_bridge.py -> /vio/imu ------------+--> OpenVINS --> estimated pose
  |-- ground truth ----> ros_gz_bridge ----> /vio/ground_truth/odom          (compared later)
  |
  +-- ArduPilot plugin <--> ArduPilot SITL <--> MAVROS <--> vio_flight.py
```

There are two separate loops here. ArduPilot flies the drone using its own sensors and simulated GPS. OpenVINS only watches the camera and the VIO IMU, and nothing it estimates goes back to the autopilot. That keeps the experiment clean: when VIO fails, the drone keeps flying the same path, so we can measure the failure without it changing the flight.

## The drone

`iris_with_vio_camera` is the stock `iris_with_ardupilot` with three things bolted on:

- **A forward camera, rigidly fixed.** 640x480 grayscale at 30 Hz with a 90 degree field of view. The existing camera models were pointing down or sat on a gimbal, and VIO needs the camera locked to the IMU. If the camera can move relative to the IMU, the geometry OpenVINS relies on is wrong all the time.
- **A second IMU, just for VIO.** ArduPilot's IMU is mounted upside down (rolled 180 degrees) and has its own job. A separate IMU at 200 Hz, aligned with the body, keeps the calibration simple. It also means noise can be added to it later without upsetting the autopilot.
- **A ground truth publisher.** Gazebo's odometry plugin publishes the true pose at 100 Hz.

Nothing is noisy yet: no image noise, no IMU noise. The baseline is meant to be clean so that the stress tests are the only thing changing.

The world is the warehouse from the gnc_autonomy project. Shelves, barrels, crates and brick walls give lots of corners to track, which the runway world didn't have.

## Why there's a custom IMU bridge

The installed `ros_gz_bridge` was built for Gazebo Garden, but the sim runs Harmonic. Images and odometry come through fine. IMU messages come through with the acceleration as zeros, which is useless for VIO.

Rather than replace the system bridge (an apt install, or a source build of `ros_gz`), `gz_imu_bridge.py` reads the IMU with Harmonic's own Python bindings and republishes it as `sensor_msgs/Imu`. It keeps Gazebo's sim-time stamp. It's about 60 lines and holds 200 Hz with no gaps at roughly 18% of one core.

## The test flight

Stress tests only mean something if every run flies the same path, so the flight is scripted. `vio_flight.py` uses `mavros_adapter.py` for the MAVROS calls and does:

1. wait for the MAVROS link, switch to GUIDED, arm
2. take off to 1.5 m
3. fly a 2 m square (forward, left, back, right) keeping the heading fixed, so the camera always faces the shelves
4. land and wait for disarm

The adapter's commands are fire-and-forget: `arm()` returns as soon as the request is sent, not when the drone is armed. So the script checks telemetry after every step and re-sends a command if nothing happens. Each step has a timeout; on a timeout it lands.

## Getting OpenVINS to start, and to stay put

Two config changes were needed once it was actually flying.

It wouldn't initialise at first. OpenVINS starts from rest: it wants the drone still, then a clear jolt in the accelerometer. The default threshold is 1.5, and ArduPilot's smooth takeoff only reached about 0.62, so it waited forever and then saw the drone already moving and gave up. `init_imu_thresh` is now 0.3. OpenVINS has to be running before takeoff for this to work.

Then it tracked the square well but ran away after landing, tens of metres within a minute while the drone sat on the floor. Turning on `try_zupt` fixed that. There's more on why in the maths notes.

## Scoring a run

Scoring happens in two halves, so a flight only has to be flown once and can be re-scored as the scoring changes.

`record_run.py` runs during the flight and writes four plain text files: the OpenVINS pose, the Gazebo ground truth (both in TUM format, `t x y z qx qy qz qw`), OpenVINS's own position and orientation variances, and every arm / mode change from MAVROS. Everything is stamped in sim time from the message headers. MAVROS stamps with the wall clock, which is useless here because the sim runs at about half speed, so mode changes are tagged with the latest ground truth time instead.

`score_run.py` runs afterwards:

1. It crops to the part worth scoring, from OpenVINS initialising to the disarm after landing. The LAND mode event isn't a good end point: ArduPilot hovers for several seconds after it before descending.
2. Ground truth is at 100 Hz and the estimate at 30 Hz, so ground truth is interpolated to each estimate time.
3. OpenVINS starts its own world frame wherever it initialises, with an arbitrary heading. On the first run it came out turned almost exactly 90 degrees from Gazebo. So the estimate is rotated about the vertical and shifted to best fit the ground truth before comparing.
4. It reports the position and rotation error (ATE), the error in motion over one second windows (RPE), the scale error and the error at landing. It also plots the top view, the height, and the error next to OpenVINS's own 3-sigma.

The maths is in the maths notes.

The first run gave 3 cm ATE. Most of that comes from a 7 cm dip in height just after takeoff. OpenVINS's 3-sigma sits around 26 cm, roughly ten times the real error. It thinks it's much worse than it is, which fits with telling it the IMU is as noisy as EuRoC's when the sim IMU is perfect.

## Replaying a flight

Two live flights of the same script never come out quite the same. On two square flights OpenVINS scored 3.0 cm and 3.8 cm, and nothing had changed but the flight. That's a problem for any comparison: if a change to the estimator moves the error by a centimetre, it could just be the flight.

So a flight is recorded once as a ROS 2 bag (`record_bag.sh`) and replayed as often as needed (`replay_run.sh`). The bag holds the camera, the VIO IMU, ground truth and the MAVROS state. It's stamped with sim time, so a replay runs at sim speed, about twice as fast as the live sim.

Two details made this work:

- The MAVROS state is published as "transient local", which means a subscriber that joins late still gets the last message. The recorder subscribes that way, so the bag has to record and replay it the same way, otherwise the takeoff and landing events disappear on replay. `qos_overrides.yaml` sets that.
- `replay_run.sh` refuses to start if the sim or another OpenVINS is running, because their topics would mix with the bag's.

How repeatable is it? Replaying the same bag three times gave the same trajectory to under a millimetre, so any difference bigger than that is real. Replay against the live run agreed to 6 mm in ATE, with poses a median 1.6 cm apart. The gap most likely comes from timing: live, the IMU samples that have arrived by the time each image is processed vary a little; replayed, they don't. So comparisons are always replay against replay on the same bag.

## Injecting faults

`fault_injector.py` is a small node that sits between a replayed bag and OpenVINS. With `--faults`, `replay_run.sh` renames the bag's IMU and camera topics to `/vio/raw/...` on playback, so OpenVINS doesn't hear them directly. The injector listens to the raw topics, changes or drops messages while a fault is active, and republishes them under the normal names. OpenVINS sees what looks like ordinary sensor data with the damage already done, which is how a real fault would look to it. Ground truth and the MAVROS state skip the injector, so the score always compares against the true path.

What a fault does to a message:

- IMU noise, bias or drift: add numbers to the accelerometer and gyro readings
- time offset: add a fixed amount to each image's timestamp
- dropped frame: don't publish it
- frozen frame: publish the previous image's pixels with the current timestamp
- darkness: scale every pixel down and add a little noise
- blur: run a blur filter over the image

Faults are timed from the arm event in the bag, so "10 s after arming" lands on the same moment of the flight in every replay. The random numbers come from a seeded generator, so a replay with a fault is as repeatable as one without. A replay through the injector with no faults gave exactly the same trajectory as a direct replay, so the extra node doesn't change anything by itself.

One scoring detail: a run that diverges by hundreds of metres would drag the frame alignment and hide where it broke. So for runs with faults, the scorer aligns only on the clean part before the first fault, and the clean reference is scored on the same stretch (`--align-window`).

What came out of the first round (numbers in `results/faults`):

- IMU noise and a 1 s camera dropout barely matter. An IMU bias step and a 20 ms camera time offset roughly double or triple the error, but OpenVINS holds.
- Blurred, dark and frozen frames break it. The frozen camera is the clearest case: the images stop changing, so OpenVINS decides the drone has stopped and applies zero-velocity updates for the whole freeze while the drone is actually flying. By the time real images come back its state is wrong and it runs away. That's the catch noted in the maths notes, now measured.
- In every run that diverged, OpenVINS's own 3-sigma stayed far below the real error. It doesn't know it has failed. That's the gap the integrity work is meant to close.

## Start / stop script

`start_vio_sim.sh --start` opens one terminal tab per process, in the same style as the older `start_all.sh`. ArduPilot runs from `runs/sitl` so its parameters and logs don't mix with other projects.

`--stop` finds the sim processes by what program they actually are (gz, arducopter, mavros_node, parameter_bridge, or Python running sim_vehicle / mavproxy / ros2 / the IMU bridge). Matching on command-line text alone turned out to be a bad idea, because it also hit unrelated shells that merely mentioned those names. Each process gets Ctrl-C first, then TERM, then KILL if it's still hanging around, and then the tabs are closed.
