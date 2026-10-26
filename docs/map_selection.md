# One map selection for the navigation stack

The launch argument `map` selects a saved occupancy-map YAML for the whole
delivery stack. It is separate from the planner (`algorithm`) and path follower
(`controller`). Omitting it keeps the generated factory map as the default.

```bash
cd ~/factory_ws
# Stop the previous demo first. Replace this path with an existing saved map:
./run.sh map:=/absolute/path/to/saved_map.yaml
```

Absolute paths, existing relative paths, `~`, and environment variables are
resolved to the same absolute path. Quote the entire argument when it contains
spaces, for example `"map:=/home/ros/maps/demo session/slam.yaml"`.

The file must be a valid saved-map YAML referring to an image supported by the
existing loaders (PGM). Relative image filenames are resolved from the YAML's
directory. An empty or missing selected path fails explicitly; it does not
substitute the generated map.

## Consumers

| Component | Startup parameter |
|---|---|
| Nav2 map server | `yaml_filename` |
| Each ICP localizer | `map_yaml` |
| Each navigator | `map_yaml` |
| Fleet manager's assignment-cost grid | `map_yaml` |
| Traffic manager's side-position grid | `map_yaml` |

The same argument is also accepted by the standalone `fleet.launch.py`,
`navigation.launch.py`, and `localization.launch.py` entry points. If you launch
nodes separately rather than through these files, supply matching parameters
to all relevant nodes yourself.

The launch chain is:

```text
demo(map)
  -> fleet(map)
       -> fleet_manager(map_yaml)
       -> traffic_manager(map_yaml), when enabled
       -> navigation(map)
            -> navigator robot1/robot2(map_yaml)
            -> localization(map)
                 -> map_server(yaml_filename)
                 -> ICP robot1/robot2(map_yaml)
```

Each custom consumer reports the resolved map path in its startup log; the map
server reports the YAML it loads. The new offline tests expand these actual
launch descriptions and inspect every consumer's parameters. They also run
the real constructors as far as map loading, using a distinct small map and
fake ROS-node setup, before any publishers/subscriptions/timers are created.

## Scope and coordinates

The launch argument is startup selection. The dashboard now also offers a
coordinated activation workflow that restarts the map server, ICP, navigators
and traffic group while retaining the fleet manager and queue. See
[dashboard map controls](dashboard_maps.md). Merely setting a running node's
parameter still does not reload its existing grid or ICP wall lookup.

The selected map must describe the existing factory in the existing `map`
coordinate frame: Gazebo world coordinates minus `(1, 1)`. Its YAML image
origin, dimensions and resolution are read by the map loaders; they may differ
without changing that coordinate frame. Rotating/translating the frame itself
would require registering the map and updating robot initial-pose handling.
Stations, semantic policy zones and current spawn-pose assumptions are unchanged.

The overview retains its factory art and collision-manifest view. The **Maps**
tab separately previews the selected file's actual occupancy grid, labels the
confirmed active source and allows idle-only activation. SLAM/OGM session
generation is a later step. Map activation does not modify the world, cargo,
or planner/controller algorithms.

The selected map is also used by the navigator's static rear-clearance checks
through its existing `PlanningGrid`, so backup recovery remains on the same map
as forward routing.
