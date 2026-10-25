"""Pure scene state for bounded, reusable delivery props; no ROS or navigation logic.

Stations provide replenishing demo supplies. The same visible transfer prop stays
with a job from its pickup tray through carriage and deposit. This is a kinematic
visual transfer, not a gripper, finite-stock, fluid or payload-dynamics simulation.
"""
import math


def stock_name(station, index):
    return f"cargo_stock_{station}_{index}"


def payload_name(robot, slot, kind):
    return f"cargo_payload_{robot}_{slot}_{kind}"


def offset_pose(pose, dx=0.0, dy=0.0):
    x, y, z, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x+c*dx-s*dy, y+s*dx+c*dy, z, yaw)


def job_key(job):
    return (job['robot'], job['id'], job['created'])


class CargoScene:
    def __init__(self, config):
        self.config = config
        self.bindings = {}
        self.desired = {}
        self.carried = {}          # model -> (robot, deck slot)
        self.deposits = {}         # receiving tray slot -> model
        self.deposit_counts = {}
        for station, spec in config['stations'].items():
            for i, (dx, dy) in enumerate(config['stock_offsets']):
                self.desired[stock_name(station, i)] = offset_pose(spec['pickup_pose'], dx, dy)
        for robot in config['robots']:
            for slot in range(len(config['deck_slots'])):
                for kind in config['types']:
                    self.desired[payload_name(robot, slot, kind)] = tuple(config['parked'])

    def _park(self, name):
        self.desired[name] = tuple(self.config['parked'])
        self.carried.pop(name, None)
        for key, model in list(self.deposits.items()):
            if model == name:
                del self.deposits[key]

    def update_jobs(self, records):
        """Consume active jobs plus a bounded recent terminal history from fleet state."""
        cfg = self.config
        records = {job_key(j): j for j in records
                   if j.get('robot') in cfg['robots'] and j.get('pickup') in cfg['stations']
                   and j.get('dropoff') in cfg['stations']}
        # Finish old bindings before assigning slots to new jobs. Suspended picked
        # jobs remain active records, retaining their original deck slot.
        for key, binding in list(self.bindings.items()):
            job = records.get(key)
            if job is not None and job['state'] in ('ASSIGNED', 'PICKED'):
                continue
            name = binding['model']
            self.carried.pop(name, None)
            if job is not None and job['state'] == 'DELIVERED':
                station = job['dropoff']
                count = self.deposit_counts.get(station, 0)
                slot = count % len(cfg['receiving_offsets'])
                old = self.deposits.get((station, slot))
                if old:
                    self._park(old)
                self.deposits[(station, slot)] = name
                self.deposit_counts[station] = count + 1
                dx, dy = cfg['receiving_offsets'][slot]
                self.desired[name] = offset_pose(cfg['stations'][station]['dropoff_pose'], dx, dy)
            else:
                # Failed/cancelled visual jobs reset to the replenishing supply;
                # never leave a stale floating payload in the robot's next mission.
                self._park(name)
            del self.bindings[key]

        for key, job in records.items():
            if job['state'] not in ('ASSIGNED', 'PICKED'):
                continue
            if key not in self.bindings:
                used = {b['slot'] for k, b in self.bindings.items() if k[0] == job['robot']}
                slot = next((i for i in range(len(cfg['deck_slots'])) if i not in used), None)
                if slot is None:
                    continue
                kind = cfg['stations'][job['pickup']]['type']
                name = payload_name(job['robot'], slot, kind)
                self._park(name)     # recycle this bounded prop's previous completed delivery
                self.bindings[key] = {'model': name, 'slot': slot}
            binding = self.bindings[key]
            name = binding['model']
            if job['state'] == 'PICKED':
                self.carried[name] = (job['robot'], binding['slot'])

        # Reserved props replace a source display at exactly the same tray pose.
        # A new source unit becomes visible when the previous unit is carried away.
        for station, spec in cfg['stations'].items():
            reserved = [b for k, b in self.bindings.items()
                        if records[k]['pickup'] == station and records[k]['state'] == 'ASSIGNED']
            for i, (dx, dy) in enumerate(cfg['stock_offsets']):
                pose = offset_pose(spec['pickup_pose'], dx, dy)
                self.desired[stock_name(station, i)] = tuple(cfg['parked']) if i < len(reserved) else pose
                if i < len(reserved):
                    self.desired[reserved[i]['model']] = pose

    def update_robot_pose(self, robot, pose):
        """Use simulator truth only to draw cargo; it never feeds localization/planning."""
        for name, (owner, slot) in self.carried.items():
            if owner == robot:
                dx, dy = self.config['deck_slots'][slot]
                x, y, yaw = pose
                self.desired[name] = offset_pose((x, y, self.config['deck_z'], yaw), dx, dy)
