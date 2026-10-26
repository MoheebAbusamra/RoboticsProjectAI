# Factory delivery fleet (ROS 2 Jazzy)

## On-demand measured maps

The dashboard uses antialiased DejaVu Sans Mono (with system monospace fallbacks).
Mapping Studio includes live map/trajectory preview and a dedicated RViz view,
opened automatically by default, plus progress within each survey waypoint.

Start the complete demo with `./run.sh`, wait for both robots idle at HOME, then
open **Maps → Generate map / Mapping studio**. Choose SLAM, OGM, or SLAM + OGM.
Robot 1 follows a guided survey while fleet dispatch and moving obstacles pause;
it returns home and restores the scene before queued work resumes. Combined mode
produces a SLAM-pose OGM plus separate maps and numerical comparisons. Select a
saved result, preview it, and explicitly **Use this map**. The original reference
map remains available. See [the mapping guide](docs/on_demand_mapping.md) for
frames, saved files, limitations and failure handling.

## Northline Control: factory visuals, robot and Python dashboard

In an Ubuntu desktop terminal:

```bash
cd ~/factory_ws
./run.sh gui:=false movers:=false
```

The native Python/Qt dashboard opens automatically. It shows the factory model,
live ICP poses and routes, robot/fleet state, mission queue, traffic decisions,
speed, lidar returns, localization health, simulation speed and VM resources.
The factory model is a static Blender render with live ROS overlays; switch to
**Floor plan** for an unobstructed policy/obstacle view. Pan/zoom or click a robot
to inspect it. **Mission queue** and **Diagnostics** provide the detailed tables.

Use **New mission** or the quick dispatch form to select pickup, destination and
routine/urgent priority. Assignment is handled by the existing fleet manager.
**Emergency kit delivery** uses its existing emergency-call topic and policy.
**Run demo jobs** submits the same four example deliveries as `send_job --demo`.
Controls stay disabled until fresh fleet telemetry is available. A submitted
request is reported as sent; the fleet assignment/queue is the confirmation.

The first-person RGB camera switches between Robot 01 and Robot 02. **Expand**
opens a larger view. Only the selected camera is subscribed to; the lazy bridge
stops unused camera rendering. Cameras also go to standby on the queue/diagnostics
pages and when the dashboard is minimized. Frame age and measured receive rate
are shown; missing or stale images are not labeled live. Camera streams and
calibration are at `/robotN/camera/image_raw` and `/robotN/camera/camera_info`.

Useful launch options:

```bash
./run.sh gui:=false                    # dashboard + moving obstacles
./run.sh gui:=true                     # also open the Gazebo editor/viewer
./run.sh gui:=false dashboard:=false   # simulation without the dashboard
./run.sh gui:=false cameras:=false    # disable RGB sensors
./run.sh gui:=false camera_width:=384 camera_height:=216 camera_rate:=5.0
```

To open the dashboard separately against an already running fleet:

```bash
source ~/factory_ws/env.sh
ros2 run factory_bot dashboard
```

Stop the previous launch with Ctrl+C before starting a replacement. Closing only
the dashboard leaves the simulation running. It can be reopened using the command
above. `python3-pyqt5` is already installed on this VM and is declared as a package
dependency. No web server, browser login, or cloud service is required.

The current factory uses free Sketchfab machinery, shelving, a workbench and
industrial containers, alongside the existing factory shell, dark polished
concrete floor, lit materials, detailed robots and moving workers/forklift.
The static factory export contains 26,866 triangles, three material batches and
one 2048-pixel color/AO atlas. The SDF retains 20 static collision primitives.
Model attribution and modification notes are in
[`SKETCHFAB_CREDITS.md`](src/factory_bot/models/industrial_factory/SKETCHFAB_CREDITS.md).
The Python code's MIT license does not replace the separate asset licenses.

All runtime meshes, materials, textures and the saved map are included in this
repository. Blender and a Sketchfab connection are only needed for authoring.
The authoring scripts build the earlier base/premium assets; running those
scripts does not reproduce the later curated Sketchfab export and can replace it.
The checked-in model resources are the current runnable factory.

Deliveries display station cargo loading onto the robot deck and unloading at
the destination. This is a kinematic visualization driven by fleet state, not an
arm, gripper or contact-physics simulation. Stations and cargo types are listed
in `src/factory_bot/config/cargo.yaml`.

Obstacle handling uses the mapping lidar plus a lower forward safety lidar.
For normal and emergency deliveries, a blocked robot waits three simulated
seconds and checks fresh scans before continuing or replanning. Recovery is
bounded by motion-progress checks rather than accepting successful replans as
proof that the robot moved. The forklift uses one outer bounding-box collision
and a matching lidar-only envelope; its visible mesh does not define contacts.

Build and offline unit/geometry checks have been performed. Simulation speed
and successful deliveries should be tested on the target machine.

## Step 17: coordinated robot traffic

The complete demo now starts `traffic_manager` automatically:

```bash
cd ~/factory_ws
./run.sh movers:=false
```

Wait for both navigators/localizers and the traffic manager. In another Ubuntu terminal:

```bash
source ~/factory_ws/env.sh
ros2 run factory_bot send_job --demo
# Optional emergency after the jobs are underway:
ros2 run factory_bot call_emergency ASSEMBLY_A
```

`./run.sh` also enables the forklift/workers, as before. `traffic:=false` disables
the new coordinator for a deliberate comparison; it does not disable lidar
obstacle avoidance. `rviz:=true` opens the existing navigation visualization.
Stop an existing launch with Ctrl+C before starting a replacement.

The coordinator publishes `/robotN/traffic` commands `go`, `wait` or `aside`;
navigators preserve their delivery goal while yielding. It applies route conflict
priority, busy-station waiting and make-room behavior, with release hysteresis.
`/traffic/state` shows its current instructions and event counters.
Events append to `~/factory_ws/results/traffic.csv`.

The supplied guide's policies are retained: near-goal +100 can outrank an
emergency's +30; after 3 simulated seconds without traffic instructions,
navigators resume independent lidar-based navigation. This is a fail-open
fallback, not a guaranteed stop on coordinator failure.

Two reviewed bugs were corrected: an active status without a received path no
longer crashes progress checking, and escape-spot search uses the same
no-corner-cutting neighbors as the actual planner, with metric distance limits.
Path publication is retained so a traffic manager joining later gets the path.

For the guide's isolated head-on / parked-on-goal / following test, run these
in separate sourced terminals with the full fleet demo stopped:

```bash
# Terminal 1 (software-rendering/display settings from run.sh may be needed over SSH):
ros2 launch factory_bot fleet_sim.launch.py movers:=false gui:=true
# Terminal 2:
ros2 launch factory_bot navigation.launch.py
# Terminal 3:
ros2 run factory_bot traffic_manager --ros-args -p use_sim_time:=true
# Terminal 4, after localization is ready:
ros2 run factory_bot traffic_test --ros-args -p use_sim_time:=true -p 'rules:="on"'
```

The isolated test must not compete with the delivery fleet manager, which can
send return-home goals. The `rules` parameter on `traffic_test` labels results;
it does not itself start/stop the coordinator. Quote `on`/`off` as shown so ROS
does not interpret them as boolean YAML values. Tour results append to
`~/factory_ws/results/traffic_test.csv`. The native dashboard is described above.

Functional checks, including the four new traffic regressions:

```bash
source ~/factory_ws/env.sh
cd ~/factory_ws/src/factory_bot
python3 -m pytest test/test_{controllers,icp,live_layer,ogm,planners,planning_grid,routes,scheduler,factory_layout,traffic}.py -q
```

## Workspace on this VM

The repository is at `/home/ros/RoboticsProjectAI`.
`/home/ros/factory_ws` is a symlink to it, preserving the bag and result paths
used by the original project. The existing `arc_ws` is independent.

`build/`, `install/`, and `log/` are generated locally and intentionally ignored.
Maps, robot descriptions, configuration, and the factory world are tracked.
There are no external Gazebo model downloads required by the factory world.

## Build

```bash
cd ~/factory_ws
source /opt/ros/jazzy/setup.bash
# On a new installation, install dependencies (may prompt for sudo):
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ./env.sh
```

On this VM pygame 2.6.1 was installed for the `ros` user using pip because
SSH has no passwordless sudo. It imports successfully. Consequently rosdep's
apt-based check still reports `python3-pygame`; use this validation here:

```bash
rosdep check --from-paths src --ignore-src --skip-keys python3-pygame
python3 -c 'import pygame'
```

## Run the complete fleet

In an Ubuntu desktop terminal:

```bash
cd ~/factory_ws
./run.sh
# Optional visualization, or a lighter server without the Gazebo window:
# ./run.sh rviz:=true
# ./run.sh gui:=false
```

This starts Gazebo with two robots, sensors, controllers, obstacle motion,
map server, ICP localization, navigation, and the fleet manager.
The fleet launches after a 15-second simulation startup delay.
Wait for both controllers and localization before sending jobs.
Even server-only Gazebo uses a graphical display for the GPU lidar (Ogre).
The run script defaults to software rendering for VirtualBox and reuses the
logged-in Xwayland display when invoked over SSH. Other display configurations
may require setting DISPLAY and XAUTHORITY manually.

In a second terminal:

```bash
cd ~/factory_ws
source ./env.sh
ros2 run factory_bot send_job STORES ASSEMBLY_A
# Or four sample requests:
ros2 run factory_bot send_job --demo
```

Ctrl+C in the launch terminal stops its child processes. The original
`stop_all.sh` is a broad emergency cleanup that can stop other ROS projects,
so use it only when appropriate.

## Select the navigation map

The complete demo accepts one shared `map` argument:

```bash
./run.sh map:=/absolute/path/to/saved_map.yaml
```

The map server, both ICP localizers, both navigators, fleet assignment and
traffic side-position searches all receive that same map. With no override,
the included `factory_map.yaml` remains the default. A missing selected file
is an error; there is no automatic fallback to the generated map.

The dashboard's **Maps** tab can now preview and activate saved maps while both
robots are idle. The complete demo supervises a restartable navigation group;
Gazebo, cameras, the dashboard and the in-memory fleet queue remain running.
Activation preserves pose estimates and navigation defaults and waits for fresh
localization before dispatch resumes. A failed activation attempts to restore
the previous map. The generated map remains available through **Restore
generated map**. Map files must use the existing factory coordinate frame so
station locations and policy zones remain aligned.

Restart an older demo once to load the new coordinator. Standalone fleet launch
also supports `managed_maps:=true`; its default remains the original unmanaged
behavior. SLAM/OGM session-generation buttons are a subsequent step. See
[dashboard map controls](docs/dashboard_maps.md) and
[launch-map details](docs/map_selection.md).

## Mapping and offline tools

Source `env.sh` first in every terminal.

```bash
ros2 launch factory_bot mapping.launch.py
# After recording and stopping mapping:
ros2 launch factory_bot map_from_bag.launch.py
ros2 run factory_bot show_grid
ros2 run factory_bot plan_benchmark
ros2 run factory_bot track_benchmark
```

Recordings are under `~/factory_ws/bags`, outputs under `~/factory_ws/results`.
The saved factory map is already included; remapping is optional.

## Functional unit tests

```bash
source ./env.sh
python3 -m pytest src/factory_bot/test/test_{controllers,icp,live_layer,ogm,planners,planning_grid,routes,scheduler}.py -q
```

## Industrial factory layout (24 October; visuals upgraded 25 October 2026)

The active world is a 16 x 14 m industrial/logistics workshop with stocked
racks, two CNC machines, a conveyor, a guarded press, inspection furniture,
charging bays, a forklift, two workers and a placeable loaded pallet.
Front/side cutaway walls and an open roof keep the robots visible.

- The press cell is forbidden to robots.
- QC inspection admits a robot whose destination is inside it, and allows it
  to leave after a job. Ordinary through traffic routes around the bay.
- The amber assembly corridor limits the commanded linear speed to 0.15 m/s.
- Moving obstacles follow routes with simple box/cylinder collisions and give
  way to robots. Most scene geometry is a single mesh with shared materials.
- Premium runtime visuals total 35,228 triangles including both robots and workers,
  with 23 static collision primitives, two diffuse textures (2048 and 1024 px),
  two directional lights and four task lights. One directional light casts shadows;
  emissive fixtures provide visible lamp strips.

The coordinate convention remains **map = Gazebo world - (1, 1)**.
`factory_map.pgm` is now a geometric simulator reference map rasterized from
static collision footprints at the lidar's 0.23 m height. It is not a new SLAM
recording. Dynamic obstacles are deliberately absent from that static map.
`maps/factory_map_v1_untuned.*`, `factory_map_ogm_odom.*`, existing bag recordings,
and previously tracked experiment results describe the older layout.
New mapping runs use `mapping_industrial_*`; replay defaults select only these
recordings so a historical bag cannot silently reconstruct the old floor plan.

Assets are self-contained in `src/factory_bot/models/industrial_factory/`.
The launch file registers that model resource directory with Gazebo.
`manifest.json` records collision footprints, geometry counts and map provenance.
To rebuild premium visuals without changing the saved map, stations or policies
(requires Blender and Python NumPy/Pillow/PyYAML on the authoring machine):

```bash
python3 scripts/generate_factory.py --visual-source work/visual-dashboard/base
python3 scripts/make_floor_texture.py
blender --background --python scripts/build_premium_assets.py -- --root "$PWD" --source work/visual-dashboard/base
colcon build --symlink-install
```

Blender is needed only for authoring; the exported assets run in Gazebo without it.
The normal `generate_factory.py` mode intentionally rebuilds the original base
geometry, map and policies and will replace premium meshes until the premium
builder is run again. Manual generated-asset edits are overwritten on regeneration.
The pre-redesign workspace sources remain in `work/before-industrial-redesign.tar.gz`.

Functional/layout verification:

```bash
source ./env.sh
python3 -m pytest src/factory_bot/test/test_{controllers,icp,live_layer,ogm,planners,planning_grid,routes,scheduler,factory_layout}.py -q
```

GUI rendering compatibility: `fleet_sim.launch.py` passes `--render-engine-gui ogre`
so the desktop GUI uses the same renderer as the lidar/camera sensors. The previous
default Ogre2 GUI produced white materials and shader errors on this VM.
