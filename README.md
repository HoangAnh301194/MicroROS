# ESP32-WROOM micro-ROS — So sánh Odometry Gazebo vs ESP32 (MVP)

Nhánh **`feature/arduino-microros-v2`** triển khai MVP đối chiếu odometry 2D do **ESP32-WROOM** tính theo thời gian thực với odometry tham chiếu từ **Gazebo (TurtleBot3 Burger)** trên **ROS 2 Humble** qua **micro-ROS Wi-Fi UDP**.

## 1. Hợp đồng ROS 2

| Topic | Kiểu bản tin | QoS | Vai trò |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | RELIABLE | `host/odom_experiment.py` phát ở **10 Hz** theo sim time; Gazebo và ESP32 cùng subscribe (`linear.x`, `angular.z`). |
| `/odom` | `nav_msgs/msg/Odometry` | RELIABLE | Gazebo diff-drive phát (`odom` → `base_footprint`), làm tham chiếu. |
| `/esp32/odom_init` | `geometry_msgs/msg/PoseStamped` | RELIABLE | Khi robot đứng yên, `odom_experiment.py` gửi `header` + `pose` từ `/odom` sang ESP32 để đặt trạng thái ban đầu. |
| `/clock` | `rosgraph_msgs/msg/Clock` | BEST_EFFORT (`ClockQoS`) | Gazebo phát (giới hạn 25 Hz trong `sim/config/gazebo_params.yaml`). ESP32 tính `dt` theo sim time; giữ pose khi pause và chờ `/esp32/odom_init` mới nếu đồng hồ nhảy lùi. |
| `/esp32/odom` | `nav_msgs/msg/Odometry` | RELIABLE | ESP32 xác nhận init và xuất pose (`frame_id=odom`, `child_frame_id=base_footprint`) + twist đã dùng để tích phân midpoint 2D. |

*(Lưu ý: `host/imu_cmd_vel_bridge.py` là script riêng cho bài demo IMU trước đó, không chạy đồng thời trong lúc đo odometry.)*

## 2. Cấu hình Wi-Fi local & Build/Flash ESP32

Firmware dùng `ROS_DOMAIN_ID = 94` (trong `src/main.cpp`), cấu hình flash **2 MB** factory-only (`partitions_2mb.csv`, tối đa `2031616` bytes) và `micro_ros_platformio` (`espressif32@6.12.0`).

Tạo file `include/wifi_config.h` (đã nằm trong `.gitignore`, không commit lên GitHub):

```bash
cp include/wifi_config.example.h include/wifi_config.h
```

Điền SSID Wi-Fi **2.4 GHz**, mật khẩu, IPv4 của PC chạy Agent (`hostname -I`) và cổng `8888`.

Build và nạp qua PlatformIO Core trong `~/.platformio/penv/bin/pio`:

```bash
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyUSB0
~/.platformio/penv/bin/pio device monitor -p /dev/ttyUSB0 -b 115200
```

## 3. Chạy thực tế trên PC

Đảm bảo các terminal dùng chung `ROS_DOMAIN_ID=94` (mặc định trong shell hiện tại) và source ROS 2 Humble.

### Terminal 1 — Mở micro-ROS Agent (local `agent_ws`)

```bash
cd /media/hoang_anh/5A1479B014798FAD/MicroROS
source /opt/ros/humble/setup.bash
source agent_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

*(Nếu dùng Docker trên máy khác: `docker run --rm -it --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6`.)*

### Terminal 2 — Launch Gazebo headless (TurtleBot3 Burger, Empty World)

Launch chỉ chạy `gzserver` (tắt GUI, LiDAR và IMU mô phỏng), giữ `/clock`, `/cmd_vel` và `/odom`:

```bash
cd /media/hoang_anh/5A1479B014798FAD/MicroROS
source /opt/ros/humble/setup.bash
ros2 launch sim/tb3_headless.launch.py
```

### Terminal 3 — Mở App so sánh quỹ đạo & sai số

```bash
cd /media/hoang_anh/5A1479B014798FAD/MicroROS
source /opt/ros/humble/setup.bash
python3 host/odom_compare_app.py
```

App ghép `/esp32/odom` với nội suy tuyến tính từ `/odom` theo `header.stamp`, vẽ 2 quỹ đạo XY cùng tỷ lệ trục, sai số vị trí (m), sai số yaw chuẩn hóa về `[-180°, 180°]` theo thời gian, sai số cuối lượt và RMSE vị trí.

### Terminal 4 — Khởi động từng bài đo kịch bản

Đảm bảo ESP32 đã bật và kết nối tới Agent trước khi chạy kịch bản:

- **Bài 1 — Đi thẳng** (`linear.x=0.15 m/s`, `angular.z=0.0`, `8.0 s`):
  ```bash
  source /opt/ros/humble/setup.bash
  python3 host/odom_experiment.py --scenario straight
  ```
- **Bài 2 — Quay tại chỗ** (`linear.x=0.0`, `angular.z=0.5 rad/s`, `7.0 s`):
  ```bash
  source /opt/ros/humble/setup.bash
  python3 host/odom_experiment.py --scenario turn
  ```
- **Bài 3 — Chạy cung** (`linear.x=0.15 m/s`, `angular.z=0.3 rad/s`, `10.0 s`):
  ```bash
  source /opt/ros/humble/setup.bash
  python3 host/odom_experiment.py --scenario arc
  ```

Có thể tùy chỉnh thêm qua `--linear-x`, `--angular-z`, `--duration` (giây sim time).

## 4. Tự kiểm tra nhanh không cần phần cứng

- Build firmware: `~/.platformio/penv/bin/pio run`
- Self-test nội suy & xử lý góc qua biên ±180° của app:
  ```bash
  python3 host/odom_compare_app.py --self-test
  ```
- Kiểm tra CLI node kịch bản:
  ```bash
  python3 host/odom_experiment.py --help
  ```
