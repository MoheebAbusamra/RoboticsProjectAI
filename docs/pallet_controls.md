# Dashboard pallet placement

Restart the complete demo once to load the updated dashboard and obstacle
controller. In the Ubuntu desktop terminal, stop the old launch with Ctrl+C,
then run:

```bash
cd ~/factory_ws
./run.sh
```

Open **Obstacles** in the dashboard (Alt+4).

1. Click a floor-grid position, or enter X and Y. Coordinates describe the
   pallet centre in the factory map frame, snapped to a **0.25 m grid**.
2. Inspect the dashed target footprint and validation message. Red corridors
   reserve the complete forklift and worker routes. Green marks the last
   confirmed pallet position.
3. Press **Place pallet**. Once placed, selecting another valid point changes
   the action to **Move pallet**.
4. Press **Remove / park outside** to return the existing model to its original
   outside parking location, Gazebo world `(19, 8)`.

The same `pallet_1` model is reused. Moving it is an instantaneous simulator
placement, not a robot delivery or a pallet-motion controller. The existing SDF
visual/collision geometry is retained. The robot detects the placed object using
its existing lidar and obstacle-replanning logic; the static navigation map is
not edited and no ground-truth obstacle is injected into the planner.

## Placement rules

- Validation covers the **whole 1.05 × 1.00 m pallet collision footprint**, plus
  0.05 m against the static map, not just its selected centre.
- The full footprint must be inside known free factory floor, outside walls
  and machines. Validation uses the factory reference map for physical layout,
  rather than a partial experimental SLAM map.
- Every configured station, including **HOME_1** and **HOME_2**, has a protected
  0.60 × 0.60 m docking area centred on its goal position. The whole pallet must
  clear this area by another 0.15 m, even when no robot is docked. Blue outlines
  show these areas on the grid; rejection messages identify the blocked station.
- Collision bounds are read from the existing SDF primitive collisions.
  Swept polygons protect every configured route segment, in both directions,
  including the forklift shell's offset. A further **0.15 m gap** is required
  between the pallet and those polygons.
- Route protection remains active when the movers are disabled or parked;
  otherwise enabling them later could collide with a placed pallet.
- Placement/movement also rejects proximity to either robot's current position.
  The backend requires fresh ground-truth robot positions for this simulated
  placement guard. This data is not used to navigate the robots.
- The UI previews the rule, and the backend independently checks it again before
  sending the Gazebo request. Invalid or nonfinite coordinates are rejected.
  Older named drop presets also undergo validation; a preset crossing a mover
  route is now rejected rather than overriding this restriction.

Red polygons show the physical actor sweep. The target pallet must also clear
their edges and the extra gap, so a centre outside a red corridor can still be
invalid if its pallet footprint clips that corridor.

## Confirmation, mapping and wiring

- Dashboard commands use `/fleet/pallet_request` (`std_msgs/String` JSON):
  `{id, action: "place", x, y}` or `{id, action: "remove"}`. Coordinates are
  map metres; the adapter converts them to Gazebo world by adding `(1, 1)`.
- The obstacle controller calls its existing `/world/factory/set_pose` service.
  `/fleet/obstacles_state.pallet` reports the confirmed place/pose, pending state
  and recent request-ID acknowledgements. The UI never labels an unconfirmed
  target as the current pallet position.
- Only one placement may be in flight. Duplicate request IDs do not send another
  pose. Failure preserves the previous confirmed position. An overdue reply is
  explicitly marked uncertain and further placement is held until it arrives.
- Controls are disabled offline, in design preview, and during mapping/map
  switching. The backend enforces the mapping lock too.
- Mapping waits for the controller's lock acknowledgement and any preceding
  pallet placement to finish, then snapshots the confirmed location. The survey
  parks the pallet and restores its exact selected grid position afterwards.

Modules are separated into pure geometry (`pallet_placement.py`), acknowledged
placement state (`pallet_control.py`), the existing ROS simulator adapter
(`obstacle_mover.py`) and the dashboard page (`dashboard_pallet.py`). Factory
geometry, mover motion/give-way behavior and robot delivery/navigation algorithms
are unchanged.

## Validation

Offline tests cover SDF dimensions, coordinate conversion, all three route
exclusions, footprint clipping, walls/bounds, robot proximity, request IDs,
pending/failure/uncertainty, mapping locks and grid restoration, and native Qt
click/select/place/move/remove behavior. A real ROS transport check uses isolated
domain 94 and a mock Gazebo service to verify rejection and acknowledged pose
commands without starting a simulator or commanding real robots. Physical
rendering, contact behavior and lidar rerouting remain for the user's demo test.
