# Dashboard saved-map preview and activation

Start a fresh complete demo once after installing this feature:

```bash
cd ~/factory_ws
./run.sh
```

The demo enables `managed_maps` on its fleet launch. Its map coordinator owns
only a child navigation group: map server/lifecycle manager, both ICP localizers,
both navigators and traffic manager. Gazebo, sensors/controllers, cargo, dashboard
and fleet manager are outside that group. A standalone fleet launch can enable
the same coordinator with `managed_maps:=true`; the standalone default remains
unmanaged. A dashboard connected to an older or unmanaged fleet displays
"Coordinator offline" and disables activation.

## Use the Maps tab

1. Finish active deliveries and wait until both robots are idle and measured
   stopped. Queued requests can remain in the fleet's memory; dispatch is paused
   atomically before the navigation group is touched.
2. Open **Maps**, then **Browse map YAML…** or select a recent source.
3. Inspect its actual occupancy grid, dimensions, resolution, image origin and
   known-space percentage. Cream is occupied, burgundy free, mauve unknown.
   Stations and semantic policy zones are overlays, not fabricated map pixels.
4. Press **Use this map**. Previewing a file alone never activates it.
5. Wait for the activation status to confirm readiness. The active-map label
   reports confirmed data separately from the selected preview. The footer also
   displays active-map state on other pages.
6. **Restore generated map** explicitly activates the included reference map.

The current generated map and source files are never overwritten. Each
activation creates a verified, content-addressed YAML/PGM copy under
`~/factory_ws/work/map_control/snapshots/`. All map consumers use that same copy.
The panel displays the original source and the actual loaded copy path.
Recent source paths persist in `work/map_control/recent.json`; a full demo restart
still defaults to the generated map unless a different `map:=...` launch argument
is supplied. Immutable activation copies remain available even if their original
source files are subsequently edited or removed.

## Activation sequence

- Inspect metadata/image and reject changed preview fingerprints, unsupported
  images or excessive planning work before touching the active group.
- Fleet manager rechecks idle/stopped state in its own executor and pauses
  allocation and automatic return-home activity. Incoming jobs remain queued,
  including emergencies. Existing active jobs cannot be interrupted for a switch.
- Hold navigation commands. Capture each robot's current estimated map pose and
  current per-robot algorithm/controller defaults, not a stale completed goal's
  options. Keep the scheduler and job records alive.
- Stop only processes retained as descendants of the coordinator's owned launch
  process, checking Linux process start identities before signals. Do not use
  global process-name kills. Do not start a replacement until the old group is gone.
- Reload the paused fleet's assignment grid and start the new navigation group
  using the captured poses/defaults. Wheel odometry remains running; ICP initializes
  its map-to-odom correction from the supplied current map poses.
- Require the fleet's loaded-map acknowledgement; new navigator, ICP and traffic
  instance IDs with matching loaded paths; measured stopped/held navigation;
  at least two accepted, sensor-fresh ICP results per robot; and active map-server
  lifecycle state plus parameter readback. Old messages cannot authorize activation.
- Release navigation hold, wait for its acknowledgement, then resume dispatch.

New and rollback localization readiness each have a default 60-second steady-clock
limit (`map_coordinator.ready_timeout`). Control/dwell/mission timing remains on
simulation time. Status publication and coordinator deadlines use steady time so
an overloaded/paused VM does not silently freeze activation control.

On activation failure before dispatch resumes, the coordinator attempts to reload
its verified previous-map copy with the original pose/default snapshot. It reports
the failure even when rollback succeeds. If rollback cannot be confirmed, robots
remain held and dispatch remains paused. Use the displayed error and navigation
logs in `work/map_control/navigation-*.log`; restore the generated map when the
idle gate is available, or restart the complete demo if required services are
unavailable. An uncertain resume reply never triggers a restart underneath a
possibly newly dispatched job. Unexpected navigation-process failure during an
active mission holds motion and requires operator recovery.

## Map compatibility

- Saved ROS YAML with an 8-bit grayscale PGM, trinary thresholds, finite positive
  resolution, and zero image-origin yaw. PGM input is normalized to a standard
  binary copy before activation.
- Same factory coordinate frame: Gazebo world minus `(1, 1)`. Image origin,
  bounds and resolution may vary; a different coordinate frame must be registered
  first. This feature does not automatically align an unrelated map.
- Current robot poses must fall in known, passable cells under the existing
  clearance and policy configuration. Unknown/unmapped stations may still make
  later routes unreachable; inspect coverage before using a partial map.
- Inputs are limited to 16 MiB images and two million cells. A planning-work
  budget rejects impractically fine/large grids; the current 5 cm factory grid
  is supported. Preflight errors leave the current group untouched.

The **Generate map / Mapping studio** control now creates measured SLAM, OGM and
combined maps on demand. See [On-demand mapping](on_demand_mapping.md) for the
guided survey, comparison outputs and how to select the saved result.

## A harmless activation test before a new SLAM map exists

Make a separately named copy of the current reference map (it is still a
geometric reference, not a SLAM result):

```bash
cd ~/factory_ws
mkdir -p work/map_test
cp src/factory_bot/maps/factory_map.pgm work/map_test/factory_map.pgm
cp src/factory_bot/maps/factory_map.yaml work/map_test/copied_reference.yaml
```

Browse `~/factory_ws/work/map_test/copied_reference.yaml`, activate while idle,
then restore the generated map. The identical geometry lets you test the new
instance acknowledgements and handoff without introducing a different layout.
Test busy rejection by starting a delivery and inspecting the disabled controls.

## Verification performed during development

Offline tests exercise launch wiring, individual pose/default handoff, real map
pixels/immutable copies, queue retention, idle races, stale instances, ICP freshness,
timeouts, rollback, lost resume replies, owned-process identity checks and native
Qt interactions. No Gazebo or real ROS navigation processes are started by these
tests. Runtime handoff performance must be tested in the VM by the user.
