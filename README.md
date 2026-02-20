# Dobot MG400 MoveIt2 Waypoints

Plan collision-free trajectories for the Dobot MG400 in RViz using MoveIt2 + OMPL, export waypoints to JSON, and execute on the real robot via TCP/IP.

## Packages

| Package | Description |
|---------|-------------|
| `mg400_description` | URDF model and meshes for the MG400 |
| `mg400_moveit_waypoints` | MoveIt2 config, trajectory exporter, and robot executor |

## Requirements

- ROS2 Humble
- MoveIt2
- pick_ik (`ros-humble-pick-ik`)

## Install & Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Alan-Wilson-111/dobot_mg400_moveit2_waypoints.git
cd ~/ros2_ws
colcon build --packages-select mg400_description mg400_moveit_waypoints
source install/setup.bash
```

## Launch MoveIt2 + RViz

```bash
ros2 launch mg400_moveit_waypoints demo.launch.py
```

In a separate terminal, start the parallel linkage compensation (keeps the SCARA linkage visually correct):

```bash
python3 ~/ros2_ws/src/dobot_mg400_moveit2_waypoints/mg400_moveit_waypoints/scripts/parallel_link_publisher.py
```

## Export Waypoints

With MoveIt2 running, start the trajectory exporter in another terminal:

```bash
python3 ~/ros2_ws/src/dobot_mg400_moveit2_waypoints/mg400_moveit_waypoints/scripts/trajectory_exporter.py
```

1. Drag the interactive marker in RViz to a target pose
2. Click **Plan** in the MotionPlanning panel
3. Press `s` in the exporter terminal to save

Waypoints are saved as JSON in a `waypoints/` directory with joint angles in degrees.

| Key | Action |
|-----|--------|
| `s` | Save trajectory |
| `p` | Preview waypoints |
| `i` | Show trajectory info |
| `q` | Quit |

## Execute on Real MG400

Connect the MG400 via Ethernet (default IP: `192.168.1.6`).

**JointMovJ mode** (point-to-point with CP blending):

```bash
python3 mg400_executor.py waypoints/trajectory_XXXXXX.json --cp 50 --speed 30
```

**ServoJ mode** (smooth streaming for dense waypoints):

```bash
python3 mg400_executor.py waypoints/trajectory_XXXXXX.json --servo --servo-t 0.2
```

**Dry run** (preview without executing):

```bash
python3 mg400_executor.py waypoints/trajectory_XXXXXX.json --dry-run
```

### Executor Options

| Flag | Default | Description |
|------|---------|-------------|
| `--ip` | `192.168.1.6` | Robot IP address |
| `--cp` | `50` | Continuous path smoothing 0-100 (JointMovJ mode) |
| `--speed` | `30` | Speed factor 1-100 |
| `--servo` | off | Use ServoJ streaming mode |
| `--servo-t` | `0.03` | ServoJ time per point in seconds |
| `--interval` | `0.03` | ServoJ send interval in seconds |
| `--dry-run` | off | Preview waypoints without executing |
| `--skip-check` | off | Skip joint limit validation |

## Note on J3 Conversion

The MG400 is a SCARA-type arm with a parallelogram linkage. MoveIt2 uses **relative** J3 angles (relative to link2), while the real robot expects **absolute** J3 angles (relative to ground). The executor handles this conversion automatically: `real_J3 = moveit_J3 + J2`.

## License

MIT
