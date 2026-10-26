# Generate measured maps from the dashboard

The Maps page now has **Generate map / Mapping studio**. It generates real
occupancy maps from the running robot's primary lidar. It does not rasterize
the SDF or start another Gazebo instance.

## Run the demo

Restart the complete demo once after this installation so the updated coordinator,
navigators and obstacle-controller telemetry are loaded. In an Ubuntu desktop terminal:

```bash
cd ~/factory_ws
./run.sh
```

Stop the old launch with Ctrl+C first. The normal cameras, queue, emergency
requests, traffic rules, cargo and navigation controls remain available.

1. Wait until navigation is verified and **both robots are idle at HOME_1 / HOME_2**.
   Let their existing automatic return-home behavior finish.
2. Open **Maps → Generate map / Mapping studio**.
3. Choose **SLAM**, **OGM**, or **SLAM + OGM**, then **Start survey**.
4. Watch the live measured occupancy preview, robot heading/trajectory, scan/map/pose
   counts and survey-route progress. Preparation has an indeterminate bar; movement
   along the current route advances progress before reaching the first waypoint.
   Progress measures the planned survey, not the percentage of the factory mapped.
   **Open RViz when the survey starts** is selected by default. The **Open RViz**
   button opens/reopens the dedicated mapping view, showing the measured map,
   estimated survey trajectory, current navigation route, robot and lidar.
   The first-person camera remains available after closing the studio; reopening
   it shows the same coordinator-owned session. Closing the studio does not cancel it.
5. Let the guided survey finish, or press **Finish & save now** for a partial map.
   **Cancel / return home** discards the unsaved session and performs cleanup.
6. Wait for the robot to return home and the session to finish restoring the scene.
7. Select an output and press **Preview saved result**. Inspect its coverage in Maps,
   then explicitly press **Use this map**. Saving never automatically switches maps.
8. Wait for activation/localization verification, then send deliveries as usual.
   **Restore generated map** returns to the original SDF-generated reference map.

The queue is retained in memory throughout the survey. Allocation, emergencies
and automatic return-home scheduling are paused atomically while Robot 1 surveys.
Robot 2 remains at HOME under a dedicated mapping hold; it cannot be asked to
move aside. Workers/forklift are temporarily parked and the pallet moved out of
the aisles, with Gazebo service acknowledgements checked before surveying. Their
previous enabled state and pallet placement are restored before dispatch resumes.
Moving actors resume their existing routes; their exact pre-survey progress is
not rewound. The rest of the factory geometry is unchanged.

## What each mode produces

| Mode | Measured outputs |
| --- | --- |
| SLAM | `slam.yaml`: SLAM Toolbox scan matching and loop closure from lidar + wheel odometry |
| OGM | `ogm_odom.yaml`: existing log-odds OGM algorithm using lidar + wheel odometry |
| SLAM + OGM | `ogm_slam.yaml`: OGM using timestamp-matched online SLAM pose estimates; also standalone SLAM, wheel-odometry OGM and a separately labelled simulator-reference OGM |

The combined mode also compares each pair over cells both maps classify as
occupied or free. The studio and `session.json` report common cell count,
classification agreement and occupied intersection-over-union. Unknown cells
are excluded, never treated as free. These are map agreement metrics, not a
claim of absolute accuracy. The simulator-reference OGM uses ground truth and
is clearly labelled; it is not input to SLAM or the combined map.

Combined OGM accumulates **online** SLAM poses. It does not retrospectively rebuild
all historical rays after a later loop closure. The standalone SLAM grid reflects
SLAM Toolbox's own corrected graph. Outputs with insufficient occupied/free data
are omitted, and a missing primary result reports a failed generation session.

Every session has a unique directory:

```text
~/factory_ws/work/map_control/sessions/s_<id>/
    slam.yaml / slam.pgm
    ogm_odom.yaml / ogm_odom.pgm
    ogm_slam.yaml / ogm_slam.pgm
    ogm_truth.yaml / ogm_truth.pgm
    session.json
```

Only applicable/valid outputs are saved. Files have the same 5 cm resolution and
canonical factory bounds as the reference map; unexplored regions stay unknown.
Completed results join the saved-map list. Session metadata includes the mapping
method, initial anchor, scan/pair counts, comparison and image hashes. YAML/PGM
files and activation snapshots persist across demo restarts. The original
reference map is never overwritten. A restart still defaults to the original
map unless an explicit `map:=...` argument selects another saved map.

## Frames and navigation

This is a **guided mapping demonstration**, not autonomous exploration of an
unknown environment. The existing navigator/ICP/current map drive Robot 1 along
the existing `mapping_route.yaml` survey waypoints at a 0.20 m/s requested speed.
The map being collected is independently measured from sensors. No SDF pixels,
ICP scan corrections or simulator poses are supplied to SLAM.

At startup, the stopped robot's current ICP pose anchors the survey coordinate
frame. Raw wheel odometry is rebased relative to its first session sample;
the running drive controller is not reset. SLAM receives only isolated
`/mapping/scan`, `/mapping/tf` and `/mapping/tf_static`, with the frames
`mapping_map → mapping_odom → mapping_base → mapping_laser`. This avoids a
second publisher competing with navigation's global map/odom transforms.
Measured SLAM cells and pose-based OGM rays are transformed back to the factory
frame (`map = Gazebo world − (1, 1)`) using that initial rigid anchor. Consequently,
anchor error can affect alignment; a survey does not eliminate an incorrect
initial localization. Ground truth is used only for the labelled comparison.

Map activation uses the existing coordinated handoff for map server, both ICP
localizers/navigators, fleet assignment and traffic. Unknown areas are blocked
by the planner. Partial maps can therefore fail to cover some destinations, even
if both current robot poses pass activation preflight. Inspect coverage first.

## Failure and recovery

- Startup, survey waypoint, save, return-home and cleanup steps have finite
  steady-clock deadlines. Missing/stale telemetry cannot confirm completion.
- A survey failure stops the robot, cancels only its survey goal, stops owned
  mapper processes and attempts to return home on the unchanged active map.
- Cleanup failure keeps motion held and dispatch paused. The studio displays
  the error; **Retry cleanup / return home** retries that cleanup when possible.
- If dispatch-resume acknowledgement is uncertain, a new job may already have
  been assigned. Cleanup is not retried underneath it: motion stays held and
  the displayed instruction is to restart the complete demo.
- Mapper process output is under `work/map_control/sessions/navigation-*.log`.
  Live status is `/maps/status` with its `mapping` field; mapper data status is
  `/mapping/session_status`. It includes a specific readiness reason and the last
  received counters, retained after mapper exit. Phase changes and errors are
  saved to the session's `state.json`.
- The canonical engine output is `/mapping/live_map`; the coordinator retains it
  on `/mapping/preview` for the studio and RViz, including after the mapper stops.
  The survey trajectory is similarly retained on `/mapping/preview_path`.
  RViz uses the global factory frame and navigation TF for the robot/lidar; its
  map layer is the measured survey output, not the old `/map` reference.
  SLAM Toolbox's absolute map topic is explicitly configured/remapped to
  `/mapping/map`, with metadata at `/mapping/map_metadata`.

Do not run the older standalone `mapping.launch.py` beside the fleet demo: that
launch starts its own simulator. The dashboard uses `mapping_generation.launch.py`,
which contains only a sensor adapter and, when required, SLAM Toolbox.

## Verification

For an optional ROS transport/SLAM check without Gazebo or robot commands, use an
unused isolated domain. The manual check refuses a domain containing other nodes:

```bash
cd ~/factory_ws
source env.sh
ROS_DOMAIN_ID=93 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
python3 src/factory_bot/test/manual_mapping_wiring_check.py \
  --mode slam_ogm --move --finish --seconds 17 --expect-saved
```

It uses synthetic lidar/odometry, checks map readiness, live output and successful
export, and asserts that the mapper does not publish on global `/map`. Its files
are diagnostic fixtures under `work/mapping_wiring_checks/`, not factory survey
maps. The same check accepts `--mode slam` or `--mode ogm`.

Offline checks cover sensor/frame conversion, real map export and unknown cells,
time matching, comparison metrics, survey route reachability on the current map,
pause denial, finish/cancel, mapper ownership, restoration order, held failures,
uncertain dispatch acknowledgement, launch isolation, and native Qt controls.
The Gazebo simulation was not run for these checks. Isolated ROS tests also
reproduced the absolute `/map` topic mismatch and confirmed the corrected
SLAM/OGM/combined pipelines reach readiness, preview and save. Live survey performance, SLAM
coverage/alignment and subsequent delivery behavior still require the user's VM test.
