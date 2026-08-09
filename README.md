# 🚀 Running the Robot

## 1. Start the LiDAR Driver

```bash
sudo ln -s /dev/ttyUSB0 /dev/sc_mini
---

```bash
ros2 run cspc_lidar cspc_lidar
```

## 2. Start SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=/home/htutaungkhant/ros2_ws/slam_custom.yaml
```

---

## 3. Publish Static TF *(Only if you do NOT have your own URDF)*

If your robot does **not** publish the transform between `base_footprint` and `laser_link`, run:

```bash
ros2 run tf2_ros static_transform_publisher \
0 0 0 0 0 0 base_footprint laser_link
```

> **Note:** Skip this step if your robot already publishes TF through a URDF and `robot_state_publisher`.

---

## 4. Start the Robot Controller

Open a new terminal and run:

```bash
ros2 run my_robot_controller cmd_vel
```

This node subscribes to `/cmd_vel` and sends velocity commands to the robot.

---

## 5. Control the Robot with Keyboard

Open another terminal and run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args \
-p speed:=0.08 \
-p turn:=0.4
```

### Keyboard Controls

| Key   | Action       |
| ----- | ------------ |
| **i** | Forward      |
| **,** | Backward     |
| **j** | Rotate Left  |
| **l** | Rotate Right |
| **k** | Stop         |

---

### This is arduino code for motor&encoder pins 
| Motor    | Encoder     |
| -------- | ---------- |
|    1,0   |    7,10    |
|    3,4   |     5,6    |

---

# Launch Order

Start the nodes in the following order:

1. LiDAR Driver
2. SLAM Toolbox
3. Static TF *(only if required)*
4. Robot Controller (`cmd_test`)
5. Keyboard Teleoperation (`teleop_twist_keyboard`)

Once all nodes are running, you can drive the robot and create a 2D map using **SLAM Toolbox**.
