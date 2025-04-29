# CARLA Walker Agent

An simple walker agent that can follow a given route.

## Publications

| Topic                              | Type                | Description                 |
| ---------------------------------- | ------------------- | --------------------------- |
| `/carla/<ROLE NAME>/walker_control_cmd` | [carla_msgs.CarlaWalkerControl](https://github.com/carla-simulator/ros-carla-msgs/tree/master/msg/CarlaWalkerControl.msg) | Walker control command |



# **Carla Walker Agent ROS Node Guide**

*Generated: 2025‑04‑28 20:23 *

---

## 1. Overview
`carla_walker_agent` is a lightweight autonomous walking agent for the CARLA simulator.  
It converts high‑level **waypoints** and a **target speed** into low‑level
`/walker_control_cmd` messages that drive a pedestrian actor (the “walker”) along a path.  
The node is written against `ros_compatibility`, so the same Python code can run
under **ROS 1 (Noetic)** *or* **ROS 2 (Foxy ↔ Humble)** without modification. citeturn0file1

---

## 2. Runtime Architecture

```mermaid
graph TD
    subgraph CARLA Bridge
        Odom[nav_msgs/Odometry]
        Ctrl[/carla/<role>/walker_control_cmd]
    end

    Odom -->|pose| Agent[carla_walker_agent.py]
    Waypoints[/carla/<role>/waypoints<br>(nav_msgs/Path)] --> Agent
    TargetSpeed[/carla/<role>/target_speed<br>(std_msgs/Float64)] --> Agent
    Agent -->|CarlaWalkerControl| Ctrl
```

* **Update loop:** `run_step()` executes at **20 Hz** (50 ms).  
* **Navigation:** The agent always steers toward the next waypoint until it is
  within **0.5 m** (`MIN_DISTANCE`). When all waypoints are consumed the walker stops.  
* **Shutdown:** On node exit a *zero* `CarlaWalkerControl` is published to ensure the
  actor halts. citeturn0file1

---

## 3. Interfaces

| Direction | Topic | Type | Purpose |
|-----------|-------|------|---------|
| **Sub** | `/carla/<role>/odometry` | `nav_msgs/Odometry` | Current pose of the walker |
| **Sub** | `/carla/<role>/waypoints` | `nav_msgs/Path` *(latched)* | Ordered list of waypoints |
| **Sub** | `/carla/<role>/target_speed` | `std_msgs/Float64` | Live speed override (m/s) |
| **Pub** | `/carla/<role>/walker_control_cmd` | `carla_msgs/CarlaWalkerControl` | Normalized direction vector + speed |

### Parameters

| Name | Default | Description |
|------|---------|-------------|
| `role_name` | `"ego_vehicle"` | Namespace for CARLA actor topics |
| `target_speed` | `2.0` | Desired walking speed in m/s |
| `mode` | `"vehicle"` | Reserved—kept for API symmetry |

Launch‑time arguments map 1:1 to the parameters above. citeturn0file0

---

## 4. Launching the Node (ROS 2)

```bash
ros2 launch carla_walker_agent carla_walker_agent.launch.py   role_name:=walker_01 target_speed:=1.4 mode:=walker
```

The launch description:

* declares CLI arguments,
* then starts one **`Node()`** action running `carla_walker_agent` with the chosen parameters and screen output enabled. citeturn0file0

> **ROS 1:** Source your workspace then run `rosrun carla_walker_agent carla_walker_agent.py _role_name:=walker_01 _target_speed:=1.4 _mode:=walker`.

---

## 5. Supplying Waypoints & Speed at Runtime

A minimal helper script is included:

```python
rosrun carla_walker_agent move_walker.py
```

It publishes a `Path` with two far‑apart waypoints and sets a target speed of **1 m/s**.  
Feel free to adapt `points[]` for your own scenarios. citeturn0file2

**Remember:**
* Frame‑IDs (`map`, `odom`) must match the CARLA bridge.
* The *Path* topic is latched—newly started agents will fetch the last plan automatically.

---

## 6. Extending the Agent

1. **Dynamic replanning:**  
   Send a new `nav_msgs/Path` whenever route changes; the agent resets its internal list.

2. **Speed envelopes:**  
   Continuously stream `std_msgs/Float64` to modulate walking speed (e.g. slowing near obstacles).

3. **Custom stopping distance:**  
   Edit the constant `MIN_DISTANCE` inside `carla_walker_agent.py` to tweak waypoint tolerance.

---

## 7. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Agent logs *“waiting for /odometry”* forever | CARLA bridge not running or wrong `role_name` | Check that `/carla/<role>/odometry` is active with `ros2 topic echo`. |
| Walker spins in place | Waypoints stacked very close together | Reduce density or increase `MIN_DISTANCE`. |
| No motion after path publish | Forgetting to set **z** field (CARLA rejects underground points) | Ensure `pose.position.z` matches walker height. |

---

## 8. Version & License
Written by Intel Labs, released under **MIT License** (see header in source). citeturn0file1
