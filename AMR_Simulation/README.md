# AMR_Simulation

Workspace ROS 2 cho `mybot_navigation2`.

## Mục đích

Hướng dẫn này giúp bạn thiết lập và chạy workspace trên máy mới.

## Yêu cầu

- ROS 2 đã cài (một distro ROS 2 tương thích, ví dụ Humble/Galactic/Iron).
- `colcon` để build workspace.
- `rosdep` để cài phụ thuộc.
- Gói ROS: `navigation2`, `robot_state_publisher`, `joint_state_publisher`, `rviz2`, `nav2_bringup`, và các package liên quan.

## Cài đặt trên máy mới

1. Cài ROS 2 theo hướng dẫn chính thức của distro bạn dùng.
2. Cài các công cụ build:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

3. Khởi tạo và cập nhật rosdep nếu chưa làm:

```bash
sudo rosdep init
rosdep update
```

4. Vào thư mục workspace của bạn:

```bash
cd ~/AMR_Simulation
```

5. Cài phụ thuộc cho source package:

```bash
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Thay `<distro>` bằng tên ROS 2 bạn cài, ví dụ `humble`.

## Build workspace

```bash
cd ~/AMR_Simulation
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
```

## Chạy workspace

Sau khi build xong, bạn có thể chạy mô phỏng và mapping theo các bước sau:

1. Mở terminal 1 và source môi trường ROS + workspace:

```bash
source /opt/ros/<distro>/setup.bash
source ~/AMR_Simulation/install/setup.bash
```

2. Trong terminal 1, chạy launch file mô phỏng:

```bash
ros2 launch mybot_navigation2 launch_sim.launch.py
```

3. Mở terminal 2 và source lại môi trường:

```bash
source /opt/ros/<distro>/setup.bash
source ~/AMR_Simulation/install/setup.bash
```

4. Trong terminal 2, chạy launch file mapping:

```bash
ros2 launch mybot_navigation2 mapping.launch.py use_sim_time:=true
```

## Các launch file cần dùng

### 1. `launch_sim.launch.py`

Launch file này:

- khởi chạy `rsp.launch.py` để publish robot description và TF
- mở Gazebo với world `src/mybot_navigation2/worlds/obstacles.world`
- spawn robot vào môi trường mô phỏng

### 2. `mapping.launch.py`

Launch file này:

- khởi chạy node `slam_toolbox` để thực hiện mapping
- cần dùng `use_sim_time:=true` khi chạy cùng Gazebo

## Ghi chú


