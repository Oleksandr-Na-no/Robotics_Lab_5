## How to Run Lab 5: Obstacle Avoidance

> Previous labs:
> [Lab 3](https://www.google.com/search?q=https://github.com/Oleksandr-Na-no/Robotics_Lab_3-4) (required)
> [Lab 4](https://www.google.com/search?q=https://github.com/Oleksandr-Na-no/Robotics_Lab_3-4)

---

### 1. Prepare the Environment & Docker

Open the Docker container from the **[robotics_lpnu](https://github.com/RybOlya/robotics_lpnu/tree/master)** repository.

```bash
cd robotics_lpnu/
./scripts/cmd run
```

---

### 2. Build Workspace (IMPORTANT)

> ⚠️ **Lab 5 depends on Lab 3**, so both packages must be built.

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/ws
colcon build --packages-select lab3 lab5
source install/setup.bash
```

---

### 3. Setup Environment Before Run

Before launching anything, make sure environment is sourced:

```bash
cd /opt/ws
source install/setup.bash
source /opt/ros/jazzy/setup.bash
```

---

### 4. Running Lab 5: Obstacle Avoidance

This lab implements obstacle avoidance using **LIDAR (`/scan`)** and **odometry (`/odom`)**.

**Launch simulation + node:**

```bash
ros2 launch lab5 obstacle_avoidance_bringup.launch.py
```

---

### 5. (Optional) Run with Custom Goal

You can change the target point using parameters:

```bash
ros2 run lab5 obstacle_avoidance --ros-args -p goal_x:=2.0 -p goal_y:=-2.0
```

---

### 6. World Setup (Required)

Before running, you should **add obstacles** into the environment:

Edit file:

```bash
lab3/turtlebot3/worlds/room.sdf
```

Add objects like:

* walls
* cylinders (pillars)
* boxes

This is required so the robot has obstacles to avoid.
