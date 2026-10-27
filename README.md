# Factory Delivery Fleet

**Two autonomous robots that deliver parts around a simulated factory, built with ROS 2 Jazzy and Gazebo Harmonic.**

You give the system delivery jobs ("take a part from STORES to ASSEMBLY_A"). A fleet manager decides which robot does each job. Each robot then:

- plans its own route;
- drives it while avoiding walls, a moving forklift, walking workers and dropped pallets;
- loads the cargo, delivers it, and goes home when there is nothing left to do.

A traffic manager keeps the two robots from blocking each other. Emergencies jump the queue. Everything can be watched and controlled from a desktop dashboard.

Everything runs inside ROS 2 and Gazebo; there is no standalone simulator.

---

## Contents

1. [What is inside](#1-what-is-inside)
2. [The factory](#2-the-factory)
3. [Requirements](#3-requirements)
4. [Install and build (once)](#4-install-and-build-once)
5. [Run the project, step by step](#5-run-the-project-step-by-step)
6. [Your first delivery](#6-your-first-delivery)
7. [Scenarios to try](#7-scenarios-to-try)
8. [Launch options](#8-launch-options)
9. [Stopping](#9-stopping)
10. [Where results are saved](#10-where-results-are-saved)
11. [How it works](#11-how-it-works)
12. [Project structure](#12-project-structure)
13. [Tests and benchmarks](#13-tests-and-benchmarks)
14. [Troubleshooting](#14-troubleshooting)
15. [Credits and licence](#15-credits-and-licence)

---

## 1. What is inside

| Part | What it does |
|---|---|
| **Robots** | Two differential-drive robots described in URDF/xacro. Each has a 360° lidar, a low front safety lidar, a rear lidar, an IMU and an RGB camera. The wheels are driven by `ros2_control` (`diff_drive_controller`). |
| **World** | A 16 × 14 m factory in Gazebo. It has walls, racks, machines, a conveyor, a guarded press, 7 named stations, a moving forklift, two walking workers and a pallet you can place anywhere. |
| **Localization** | Each robot works out where it is on the map with **ICP** (Iterative Closest Point): it lines up its lidar scan with the walls on the map. |
| **Planning** | **A\*** (or Dijkstra or BFS) on a grid where the walls are grown by the robot's radius, so the robot can be treated as a single point. Restricted and slow zones are part of the grid. |
| **Path following** | **Pure Pursuit** (the default) or **PID** turns the route into speed and steering commands. |
| **Obstacles** | Anything not on the map makes the robot stop, wait 3 s, look again, and then replan around it. If it is stuck, it may back up briefly. |
| **Fleet manager** | Keeps the job queue (emergency > urgent > routine) and gives each job to the best robot. Emergencies can interrupt a busy robot. |
| **Traffic manager** | Gives right of way, waits at busy stations, keeps a following distance, and asks a robot to step aside when it blocks the other. |
| **Dashboard** | Desktop app (Python/Qt). It shows the live factory, robots, routes, cameras, the job queue, traffic decisions, maps and diagnostics, and it lets you send jobs. |
| **Mapping studio** | Builds new maps from the robot's own sensors (SLAM, OGM, or both), compares them, and switches the whole fleet to the chosen map. |

---

## 2. The factory

Coordinates are in metres in the **map frame**, which is Gazebo world − (1, 1). HOME_1 is at the map origin.

| Station | Where | Use |
|---|---|---|
| `HOME_1`, `HOME_2` | Bottom left, charging bays | Where robot1 and robot2 start, and where they go when idle |
| `STORES` | Top left | Parts store |
| `TOOL_CRIB` | Left, middle | Tools; the **emergency kit** is kept here |
| `QC_LAB` | Top, middle | Quality control. Robots may enter only when it is their destination |
| `ASSEMBLY_A` | Top right | Assembly station |
| `ASSEMBLY_B` | Right, middle | Assembly station |

Zones:

- **Press cell:** forbidden to robots.
- **QC inspection bay:** authorised entry only.
- **Amber assembly corridor:** robots are limited to 0.15 m/s.

The forklift and the workers follow fixed routes and pause when a robot comes close.

---

## 3. Requirements

- **Ubuntu 24.04** with a desktop session. The lidar uses Gazebo's renderer, so a display is needed even when the Gazebo window is hidden.
- **ROS 2 Jazzy** (desktop install).
- **Gazebo Harmonic**, the version that pairs with Jazzy.
- About 4 CPU cores and 8 GB RAM recommended. A virtual machine works; it runs with software rendering by default.

ROS packages used: `ros_gz` (sim, bridge, interfaces), `gz_ros2_control`, `ros2_controllers`, `nav2_map_server`, `nav2_lifecycle_manager`, `nav2_msgs`, `slam_toolbox`, `xacro`, `robot_state_publisher`, `rviz2` and `rosbag2`.

> **Why Nav2?** Nav2 is the standard navigation toolbox for ROS 2. This project uses only three small parts of it, all for handling the map file:
>
> - **Map server** (`nav2_map_server`): the map is saved as a picture plus a settings file (`factory_map.pgm` + `factory_map.yaml`). The map server opens that file and shares it on the `/map` channel, which is what RViz needs to draw the map. Using the standard tool meant we did not have to write our own map loader.
> - **Lifecycle manager** (`nav2_lifecycle_manager`): the map server waits to be switched on before it publishes anything. The lifecycle manager switches it on automatically at startup, so nobody has to do it by hand.
> - **`LoadMap` message** (`nav2_msgs`): when you choose a new map in the dashboard, the map coordinator tells the fleet manager "load this map file" using this ready-made message format instead of a custom one.
>
> The robots' "thinking" is this project's own code, not Nav2's: route planning (A\*), steering (Pure Pursuit / PID), knowing where they are (ICP), avoiding obstacles, and the fleet and traffic logic. Nav2 only handles the map file.

Python packages used: `numpy`, `yaml`, `matplotlib`, `pygame`, `PIL` (Pillow) and `PyQt5`.

---

## 4. Install and build (once)

### 4.1 Get the code

Several tools save results to `~/factory_ws/results`, so clone the repository as `~/factory_ws`:

```bash
git clone https://github.com/MoheebAbusamra/RoboticsProjectAI.git ~/factory_ws
cd ~/factory_ws
```

> If you already have the repository somewhere else, link it instead:
> `ln -s /path/to/RoboticsProjectAI ~/factory_ws`

### 4.2 Install dependencies

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` cannot install something, install the main packages by hand:

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-msgs ros-jazzy-slam-toolbox \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-rosbag2-storage-mcap \
  python3-numpy python3-yaml python3-matplotlib python3-pygame python3-pil python3-pyqt5 python3-pytest
```

### 4.3 Build

```bash
cd ~/factory_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ./env.sh
```

`env.sh` loads ROS 2 and this workspace. **Run `source ~/factory_ws/env.sh` in every new terminal** before using any `ros2` command for this project.

You only need to rebuild after adding files or changing `setup.py`. With `--symlink-install`, edits to Python files take effect straight away.

---

## 5. Run the project, step by step

### Step 1: start everything

Open a terminal **in the Ubuntu desktop**, not over plain SSH:

```bash
cd ~/factory_ws
./run.sh gui:=false
```

`gui:=false` hides the heavy Gazebo window; the simulation still runs and the dashboard shows everything. Use plain `./run.sh` if you also want the 3D Gazebo window.

### Step 2: wait about 30 seconds while it starts

This is what happens, in order:

1. **Gazebo** starts the factory and spawns `robot1` at HOME_1 and `robot2` at HOME_2.
2. The **dashboard** window ("Northline Control") opens.
3. After **15 seconds** the fleet starts: the map server, the ICP localizer and navigator for each robot, the fleet manager, the traffic manager and the map coordinator.
4. Each robot's ICP matches a few lidar scans to the map. When it succeeds, localization shows as OK.

**It is ready when** the dashboard shows both robots idle at home with localization OK, and the **New mission** and **Run demo jobs** buttons become clickable. They stay disabled until fresh data arrives from the fleet.

### Step 3: get to know the dashboard

The pages are along the top, and each has a shortcut:

| Page | Shortcut | What you see and can do |
|---|---|---|
| **Overview** | Alt+1 | Live factory view (switch between *Factory model* and *Floor plan*), robot positions and routes, robot state, speed, lidar, localization health, simulation speed, and the robot camera (switch robot, **Expand**). Buttons: **Run demo jobs**, **+ New mission** (Ctrl+N) and **Emergency kit delivery**. Click a robot to inspect it. |
| **Mission queue** | Alt+2 | Every job: waiting, assigned, picked up, delivered, with times |
| **Maps** | Alt+3 | Preview saved maps, switch the fleet to another map, and open the **Mapping studio** |
| **Obstacles** | Alt+4 | Place, move or remove the pallet on a floor grid |
| **Diagnostics** | Alt+5 | Detailed node and topic health, and logs |

---

## 6. Your first delivery

**From the dashboard:** click **+ New mission**, choose pickup `STORES`, destination `ASSEMBLY_A`, priority `routine`, and send.

**Or from a second terminal:**

```bash
source ~/factory_ws/env.sh
ros2 run factory_bot send_job STORES ASSEMBLY_A
```

**What you should see:**

1. The job appears in the queue and is given to the robot closer to STORES (usually robot1).
2. The robot's planned route appears, and the robot drives to STORES.
3. On arrival it turns to face the station, waits 3 s while the cargo is loaded (a box appears on its deck), then drives to ASSEMBLY_A. It slows to 0.15 m/s in the amber assembly corridor.
4. It unloads for 3 s, and the job is marked **DELIVERED** and saved to `~/factory_ws/results/jobs.csv`.
5. After 10 s with nothing to do, it drives back home.

---

## 7. Scenarios to try

For any scenario that uses the terminal, open a second terminal and run `source ~/factory_ws/env.sh` first.

### Scenario 1: a batch of jobs

Click **Run demo jobs**, or run:

```bash
ros2 run factory_bot send_job --demo
```

This sends four jobs, one per second:

- STORES → ASSEMBLY_A (routine)
- TOOL_CRIB → ASSEMBLY_B (routine)
- STORES → QC_LAB (**urgent**)
- ASSEMBLY_B → TOOL_CRIB (routine)

**Expect:**

- Both robots start working at once.
- The urgent job is served next, as soon as a robot becomes free. Urgent jobs jump the queue but never interrupt a robot.
- Only the robot delivering to QC_LAB enters the QC bay.

### Scenario 2: an emergency

While the demo jobs are running, click **Emergency kit delivery** (choose `ASSEMBLY_A`), or run:

```bash
ros2 run factory_bot call_emergency ASSEMBLY_A
```

**Expect:**

- The emergency goes to the top of the queue.
- If no robot is free, the fleet manager **interrupts** the best busy robot. That is the one closest to the kit, with a penalty if it is already carrying cargo.
- The robot fetches the kit from TOOL_CRIB and rushes it to ASSEMBLY_A at 0.40 m/s instead of 0.25 m/s.
- If the robot was already carrying a part, it keeps it on board and delivers it afterwards. If it had not picked anything up yet, that job goes back into the queue.

### Scenario 3: something blocks the aisle

Start a delivery, for example `send_job STORES ASSEMBLY_A`. While the robot is on its way, drop the pallet in its path:

- **Dashboard:** open **Obstacles** (Alt+4), click a free spot on the robot's route, then **Place pallet**. The dashboard refuses spots on walls, stations or forklift and worker routes.
- **Terminal:**

  ```bash
  ros2 topic pub --once /fleet/obstacle_event std_msgs/msg/String \
    "{data: '{\"model\": \"pallet_1\", \"place\": \"main_aisle\"}'}"
  ```

**Expect:** the robot sees an object that is not on the map and **stops**. It waits **3 s**, takes a fresh scan, and if the pallet is still there it **plans a new route around it** and carries on. If there is no way round at all, it keeps retrying and gives up after 30 s. The fleet manager then puts the job back in the queue to try again, and marks it **FAILED** after 3 attempts.

Remove the pallet with **Remove / park outside**, or with the same command and `"place": "parked"`.

### Scenario 4: people and forklifts

The forklift and workers move by default. Watch a robot meet one: it stops, waits a moment, and continues once they have moved on. The movers also pause when a robot comes near.

For a calm run without them:

```bash
./run.sh gui:=false movers:=false
```

### Scenario 5: two robots, one station (traffic rules)

Send two jobs that both start at STORES:

```bash
ros2 run factory_bot send_job STORES ASSEMBLY_A
ros2 run factory_bot send_job STORES ASSEMBLY_B
```

**Expect:** one robot loads at STORES while the other **waits outside** because the station is busy. If a robot is standing in the other's way, it is told to **step aside** to a free spot and then resume.

To see these decisions live:

```bash
ros2 topic echo /traffic/state
```

Every decision is also logged to `~/factory_ws/results/traffic.csv`.

To compare with the rules switched off, restart with `./run.sh gui:=false traffic:=false` and send the same jobs. Without the warden the robots rely only on their lidars, and can end up nose to nose.

### Scenario 6: compare algorithms and controllers

Restart with different choices:

```bash
./run.sh gui:=false controller:=pid algorithm:=dijkstra allocation:=optimal
```

| Option | Values |
|---|---|
| `controller` | `pure_pursuit` (default), `pid` |
| `algorithm` | `astar` (default), `dijkstra`, `bfs` |
| `allocation` | `greedy` (default), `optimal` |

Send the demo jobs and compare `jobs.csv` between runs. For each goal, `/robot1/nav_status` also reports planning time, cells explored, path length and how far the robot strayed from the route.

### Scenario 7: build your own map (Mapping studio)

1. Make sure both robots are **idle at home** and no jobs are running.
2. Open **Maps** (Alt+3), then **Generate map / Mapping studio**.
3. Choose **SLAM**, **OGM** or **SLAM + OGM**, and press **Start survey**.
4. Watch it work. robot1 drives a guided survey route slowly while robot2 stays home. Jobs, the forklift and the workers pause, and the pallet is moved aside. The new map grows live, and RViz can open alongside.
5. When it finishes (or you press **Finish & save now**), robot1 returns home and the scene is restored.
6. Pick a result, press **Preview saved result**, check what it covers, then press **Use this map**. The whole fleet switches over safely.
7. Press **Restore generated map** to go back to the original map at any time.

**What the three modes mean:**

| Mode | How the map is made |
|---|---|
| **SLAM** | SLAM Toolbox builds the map *and* corrects the robot's position as it goes, using scan matching and loop closure |
| **OGM** | The project's own occupancy-grid mapper. It marks lidar hits as walls and the space before them as free, using wheel odometry for the position, which drifts over time |
| **SLAM + OGM** | The OGM mapper fed with SLAM's corrected positions. It also saves the separate maps and scores how well they agree |

Results are saved under `~/factory_ws/work/map_control/sessions/`.

> **Which map do the robots use normally?** `src/factory_bot/maps/factory_map.yaml`. It is generated directly from the factory's geometry (walls and machines at lidar height), so it is an exact copy of what the simulated lidar can see. The robots never run SLAM while delivering. They use a fixed map and localize on it with ICP. SLAM and OGM are tools for building an alternative map from the robots' own sensors.

### Scenario 8: watch in RViz

```bash
./run.sh gui:=false rviz:=true
```

RViz shows the map, both robots, their lidar scans and their planned routes.

---

## 8. Launch options

All options are passed to `./run.sh` as `name:=value`.

| Option | Default | Meaning |
|---|---|---|
| `gui` | `true` | Open the Gazebo 3D window (`false` = lighter, simulation still runs) |
| `dashboard` | `true` | Open the dashboard |
| `rviz` | `false` | Open RViz |
| `movers` | `true` | Forklift and workers move |
| `traffic` | `true` | Traffic manager on |
| `controller` | `pure_pursuit` | `pure_pursuit` or `pid` |
| `algorithm` | `astar` | `astar`, `dijkstra` or `bfs` |
| `allocation` | `greedy` | `greedy` or `optimal` job assignment |
| `map` | `factory_map.yaml` | Absolute path of a saved map YAML used by the whole fleet |
| `cameras` | `true` | Robot RGB cameras on |
| `camera_width` / `camera_height` / `camera_rate` | `512` / `288` / `8.0` | Camera resolution and frame rate |
| `robots` | `robot1,robot2` | Robots to start |

Examples:

```bash
./run.sh gui:=false movers:=false                    # quiet factory
./run.sh gui:=false cameras:=false                   # lighter on slow machines
./run.sh map:=/home/$USER/maps/my_map.yaml           # start with another saved map
```

To reopen the dashboard while the simulation is running:

```bash
source ~/factory_ws/env.sh
ros2 run factory_bot dashboard
```

---

## 9. Stopping

Press **Ctrl+C** in the terminal running `./run.sh`. This stops everything it started. Closing only the dashboard leaves the simulation running.

If something is left over (for example a second Gazebo, or robots that do not respond after a restart), run:

```bash
~/factory_ws/stop_all.sh
```

> `stop_all.sh` stops *every* Gazebo/ROS process that matches common names, including ones from other projects. Use it only for cleanup.

Always stop the previous run before starting a new one.

---

## 10. Where results are saved

All files go to `~/factory_ws/results/`.

| File | Written by | Contents |
|---|---|---|
| `jobs.csv` | fleet manager | One row per finished job: robot, waiting / driving / total time, distance, interruptions |
| `traffic.csv` | traffic manager | Every wait or step-aside: who, for whom, which rule, how long, distance apart |
| `gazebo_nav.csv` | `nav_test` | Per goal: planning time, path length, travel time, goal error, cross-track error, replans |
| `loc_check_*.csv` | `loc_check` | Localization error (ICP position vs true position) |
| `traffic_test.csv` | `traffic_test` | Head-on / parked-on-goal / same-way test results |
| `planner_benchmark.csv`, `planner_example.png` | `plan_benchmark` | BFS vs Dijkstra vs A* on every station pair |
| `tracking_benchmark.csv`, `tracking_example.png` | `track_benchmark` | PID vs Pure Pursuit on every station pair |
| `planning_grid.png` | `show_grid` | Picture of the planning grid |

> Some result files already in the repository were recorded on an earlier version of the factory layout. Re-run the tools to get numbers for the current layout.

---

## 11. How it works

### One delivery, message by message

```text
Dashboard / send_job --/delivery_request--> fleet_manager --/robot1/goto--> navigator (robot1)
                                                                              |  A* plans a route
                                                                              |  /robot1/plan --> traffic_manager
                                                                              v
                       Gazebo robot <--/robot1/cmd_vel-- navigator (Pure Pursuit / PID, 20x per second)
                       Gazebo robot --/robot1/scan, /robot1/odom--> icp_localizer --TF map->odom--> navigator
navigator --/robot1/nav_status--> fleet_manager, traffic_manager, dashboard
traffic_manager --/robot1/traffic (go / wait / aside)--> navigator
fleet_manager --/fleet/state--> dashboard, traffic_manager, cargo_manager
```

### Main nodes

| Node | Job |
|---|---|
| Gazebo + `ros_gz_bridge` | The simulated world; passes lidar, IMU, camera, clock and true positions into ROS |
| `diff_drive_controller` (one per robot) | Turns speed commands into wheel speeds; publishes wheel odometry |
| `icp_localizer` (one per robot) | Works out where the robot is on the map |
| `navigator` (one per robot) | Plans, drives, handles obstacles and recovery, and reports its status |
| `fleet_manager` | Job queue, job allocation, emergencies, loading and unloading, return home |
| `traffic_manager` | Right-of-way rules between the robots |
| `obstacle_mover`, `cargo_manager` | Move the forklift, workers, pallet and cargo boxes in Gazebo |
| `map_server`, `map_coordinator` | Share the map; switch maps safely; run mapping surveys |
| `dashboard` | Operator interface |

### Making the map vs using the map

- **Making the map** happens once. The default map is generated from the factory geometry. The Mapping studio can build alternatives with SLAM or OGM.
- **Using the map** happens all the time: ICP answers "where am I?", A\* answers "which way?", Pure Pursuit or PID answers "how do I steer?". Lidar handles anything not on the map.

---

## 12. Project structure

```text
factory_ws/
├── run.sh                 # start the complete demo
├── env.sh                 # source this in every terminal
├── stop_all.sh            # emergency cleanup of leftover processes
├── results/               # CSV results and pictures (see section 10)
├── scripts/               # factory generation and asset-building scripts (authoring only)
├── bags/                  # recorded sensor data from earlier mapping runs
└── src/factory_bot/
    ├── factory_bot/       # all Python nodes and algorithms
    │   ├── navigator.py, planners.py, planning_grid.py, controllers.py
    │   ├── icp.py, icp_localizer.py, live_layer.py, normal_obstacle_policy.py
    │   ├── motion_progress.py, backup_recovery.py
    │   ├── fleet_manager.py, scheduler.py, traffic.py, traffic_manager.py
    │   ├── obstacle_mover.py, cargo_manager.py, pallet_placement.py
    │   ├── map_coordinator.py, mapping_engine.py, ogm.py, ogm_mapper.py
    │   ├── dashboard*.py
    │   └── send_job.py, call_emergency.py, nav_test.py, loc_check.py, ...
    ├── launch/            # demo, fleet_sim, fleet, navigation, localization, mapping ...
    ├── config/            # stations, zones, obstacles, cargo, controllers, SLAM settings
    ├── urdf/              # robot description (xacro)
    ├── worlds/            # factory.sdf
    ├── maps/              # factory_map.yaml/.pgm (default map)
    ├── models/            # factory, robot, cargo and dashboard assets
    ├── rviz/              # RViz layouts
    └── test/              # unit tests
```

---

## 13. Tests and benchmarks

**Unit tests** (no Gazebo needed):

```bash
source ~/factory_ws/env.sh
cd ~/factory_ws/src/factory_bot
python3 -m pytest test/test_{controllers,icp,live_layer,ogm,planners,planning_grid,routes,scheduler,factory_layout,traffic,backup_recovery,pallet_placement}.py -q
```

**Offline benchmarks** (no Gazebo needed; results go to `~/factory_ws/results/`):

```bash
ros2 run factory_bot show_grid          # picture of the planning grid + station checks
ros2 run factory_bot plan_benchmark     # BFS vs Dijkstra vs A*
ros2 run factory_bot track_benchmark    # PID vs Pure Pursuit
```

**Measurements in the running simulation.** Run these while the demo is running and the robots are idle:

```bash
ros2 run factory_bot loc_check --ros-args -p use_sim_time:=true   # localization error, Ctrl+C for summary
```

**Single-robot and traffic tests.** These need the simulation and navigation *without* the fleet manager, which would otherwise send the robots home in the middle of a test. Stop the demo, then use separate terminals:

```bash
# Terminal 1
ros2 launch factory_bot fleet_sim.launch.py movers:=false
# Terminal 2
ros2 launch factory_bot navigation.launch.py
# Terminal 3, after both robots are localized: pick one
ros2 run factory_bot nav_test --ros-args -p use_sim_time:=true -p controller:=pid
ros2 run factory_bot odom_test --ros-args -p use_sim_time:=true
# Traffic test: also start the traffic manager, then run the test
ros2 run factory_bot traffic_manager --ros-args -p use_sim_time:=true
ros2 run factory_bot traffic_test --ros-args -p use_sim_time:=true -p 'rules:="on"'
```

---

## 14. Troubleshooting

| Problem | What to do |
|---|---|
| `Build first: ... colcon build` when sourcing `env.sh` | Run the build in [section 4.3](#43-build). |
| `nobody is listening on /delivery_request` | The fleet starts 15 s after the simulation. Wait, and check that `./run.sh` is still running. |
| Dashboard buttons are greyed out | The fleet hasn't sent fresh data yet. Wait until both robots show localization OK. |
| Robots don't move, or "localization lost" | Give ICP a few seconds. If it persists, stop everything (Ctrl+C, then `./stop_all.sh`) and start again. Two simulations running at once is the most common cause. |
| Very slow simulation | Use `gui:=false`, `cameras:=false`, `movers:=false`. Close RViz. The dashboard shows the simulation speed. |
| Errors about the display or lidar when started over SSH | Start `./run.sh` from a terminal inside the Ubuntu desktop. The lidar needs a graphical display, even when the Gazebo window is hidden. |
| Gazebo shows white or missing materials | The launch already uses the Ogre renderer, which works in VirtualBox. Make sure you start it with `./run.sh`, which also enables software rendering. |
| A map can't be activated | Robots must be idle and stopped at a location the new map covers. Check its coverage in the Maps preview first. |

---

## 15. Credits and licence

- Python code: MIT licence (see `src/factory_bot/package.xml`).
- 3D models: Sketchfab assets under **CC BY 4.0**. Creators and changes are listed in [`SKETCHFAB_CREDITS.md`](src/factory_bot/models/industrial_factory/SKETCHFAB_CREDITS.md). Textures are from ambientCG (**CC0**). The asset licences are separate from the code licence.
- Built on ROS 2 Jazzy, Gazebo Harmonic, ros2_control, Nav2 map server and SLAM Toolbox.
- Course project for **AI Planning for Robotics Systems (2026–27)**. Course labs: [farahaymen/AI-Planning-for-Robotics-Systems-2026-27](https://github.com/farahaymen/AI-Planning-for-Robotics-Systems-2026-27).

