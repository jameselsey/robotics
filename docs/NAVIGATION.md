# Semantic room context and navigation

RoboPi separates human map meaning from the occupancy map:

- `maps/house.yaml` and `maps/house.pgm` describe free, occupied, and unknown space.
- `src/senses/config/rooms.yaml` describes named polygons and reviewed goal poses in the `map` frame.
- SLAM Toolbox or AMCL provides the live `map -> odom` transform.
- Wheel encoders provide `odom -> base_link`.
- Nav2 plans and controls a collision-aware route to a `NavigateToPose` goal.

## Agent tools

Both the Strands `brain` node and the Nova/Strands `voice_agent` expose tools to:

- list and describe known rooms;
- report the robot's map pose and current room;
- navigate to an exact room name;
- report navigation status; and
- cancel an active navigation goal.

Spoken number words are normalized, so `bed one` resolves to `bed 1`. Ambiguous
names are rejected rather than selecting a room arbitrarily. Navigation requires
an explicit, reviewed `navigate_pose`; polygon centroids are not treated as safe
goals automatically.

## Live-map navigation

Use this while building or extending a map with SLAM Toolbox:

```bash
make build
make launch-navigation
```

SLAM Toolbox owns `map -> odom`. Nav2 consumes `/map`, `/scan`, `/odom`, and TF,
then exposes `/navigate_to_pose`. Starting Nav2 does not move the robot; movement
begins only after an accepted goal.

## Saved-map localization and navigation

Use this after restarting in an already mapped environment:

```bash
make build
make launch-localized
```

This loads `maps/house.yaml`, starts AMCL and Nav2, and does not start online SLAM.
AMCL still needs an initial pose after the robot or map changes. Set it in Foxglove
or publish `geometry_msgs/PoseWithCovarianceStamped` on `/initialpose`. Confirm
that the scan aligns with the saved walls before asking the agent to navigate.

Do not run SLAM Toolbox and AMCL as competing `map -> odom` publishers.

## Room annotations

The automatic room-marker node reads the same installed configuration as the
agent and publishes `/visualization_marker_array`. To publish an alternate file
manually:

```bash
make publish-room-markers ROOMS_CONFIG=maps/house.rooms.yaml
```

Each navigable room should contain:

```yaml
rooms:
  example:
    navigate_pose: {x: 1.0, y: 2.0, yaw: 0.0}
    polygon:
      - [0.5, 1.5]
      - [1.5, 1.5]
      - [1.5, 2.5]
      - [0.5, 2.5]
```

Choose the goal on known free map space with enough clearance for the configured
robot footprint and inflation radius. Test a goal manually before relying on a
voice command.

## Safe first test

1. Keep the emergency stop and Xbox controller in reach.
2. Start `make launch-navigation` in the mapped environment.
3. Check that `/navigate_to_pose` reports one action server.
4. In Foxglove, confirm the robot, scan, and map align.
5. Ask `What room are you in?`; this cannot command motion.
6. Send one nearby Nav2 goal manually and be ready to cancel it.
7. Only then try `Drive to bed one` through the agent.

The Xbox deadman remains useful for manual driving, but it is not a Nav2 goal
cancel mechanism. Use the agent's `cancel_navigation` tool or cancel the Nav2
action explicitly.

## Official references

- [Nav2 overview](https://docs.nav2.org/)
- [AMCL configuration](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
- [NavigateToPose action](https://docs.nav2.org/configuration/packages/bt-plugins/actions/NavigateToPose.html)
