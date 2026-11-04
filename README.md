# ESP32-WROOM micro-ROS — Arduino + PlatformIO

Đây là nhánh **feature/arduino-microros**, độc lập với firmware ESP-IDF.

- ESP32-WROOM publish IMU giả lập `/imu/data` (`sensor_msgs/msg/Imu`) **20 Hz** (50 ms).
- PC ROS 2 Humble subscribe IMU và publish `/cmd_vel` (`geometry_msgs/msg/Twist`) **10 Hz** (100 ms).
- ESP32 subscribe `/cmd_vel`, in `linear.x` và `angular.z` ra Serial. **Không điều khiển motor.**
- Giao thức: **Wi-Fi UDP / micro-ROS Agent Humble**. IMU giả có nhiễu ngẫu nhiên, quaternion quay quanh Z và trọng lực 9.81 m/s².

## 1. Clone trên phân vùng Linux ext4 (khuyến nghị)

Trước đó bản ESP-IDF nằm trên NTFS `/media/...`; để tránh lỗi build micro-ROS, dùng thư mục home ext4:

```bash
cd ~
git clone -b feature/arduino-microros https://github.com/HoangAnh301194/MicroROS.git MicroROS-Arduino
cd ~/MicroROS-Arduino
```

Nếu tiếp tục trong repo cũ, lưu các thay đổi cục bộ (ví dụ sdkconfig đã chỉnh) trước khi chuyển nhánh:

```bash
git fetch origin
git switch --track origin/feature/arduino-microros
```

Không chạy `pio run -t menuconfig`: đây là project Arduino, không phải ESP-IDF.

## 2. Tạo cấu hình Wi-Fi local

```bash
cp include/wifi_config.example.h include/wifi_config.h
nano include/wifi_config.h
```

Sửa 4 mục: SSID Wi-Fi **2.4 GHz**, mật khẩu, IPv4 của PC chạy Agent (xem bằng `hostname -I`), và cổng UDP 8888. Ví dụ:

```c
#define MICROROS_WIFI_SSID "YourSSID"
#define MICROROS_WIFI_PASSWORD "YourPassword"
#define MICROROS_AGENT_IP "192.168.1.100"
#define MICROROS_AGENT_PORT 8888
```

File `include/wifi_config.h` được Git bỏ qua; không đưa mật khẩu lên GitHub.

## 3. Build / upload (ESP32)

Dùng PlatformIO Core mới đã cài, tránh `/usr/bin/pio` cũ phiên bản 4.3.4:

```bash
cd ~/MicroROS-Arduino
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyUSB0
~/.platformio/penv/bin/pio device monitor -p /dev/ttyUSB0 -b 115200
```

Thay /dev/ttyUSB0 bằng cổng của ESP32. Lần build đầu micro_ros_platformio vẫn sử dụng công cụ Python/CMake/colcon bên trong thư viện. Nếu thiếu công cụ hệ thống:

```bash
sudo apt update
sudo apt install -y git cmake python3-pip python3-venv
```

Firmware đã ghim **espressif32@6.12.0**, thư viện [micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio) tại commit cfee17fa..., ROS 2 **Humble**, transport **wifi**.

**Flash:** log ESP-IDF trước báo thấy chip **2 MB** thay vì 4 MB. Bản này đặt flash 2 MB và bảng phân vùng **factory-only (không OTA)**, có app tối đa 0x1F0000 byte = 1,984 KiB. Nếu xác minh chip thực tế 4 MB, cần điều chỉnh lại cấu hình. Nếu firmware lớn hơn phân vùng app thì không được flash.

## 4. Agent trên PC — terminal 1

```bash
docker run --rm -it --net=host \
  microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

Sau khi Agent đang chạy, khởi động ESP32. Bảo đảm PC và ESP32 cùng mạng LAN có thể truyền UDP; kiểm tra firewall nếu không kết nối.

## 5. Node ROS 2 — terminal 2

Giữ nguyên Python node từ nhánh trước:

```bash
source /opt/ros/humble/setup.bash
cd ~/MicroROS-Arduino
python3 host/imu_cmd_vel_bridge.py
```

Mặc định gửi Twist 0 ở 10 Hz. Có thể thử số khác 0:

```bash
python3 host/imu_cmd_vel_bridge.py --ros-args \
  -p linear_x:=0.20 -p angular_z:=0.10
```

Nếu IMU mất quá 0.5 giây, node vẫn phát cmd_vel 10 Hz nhưng đặt vận tốc về 0.

## 6. Kiểm tra thực tế — terminal 3

```bash
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic hz /imu/data --qos-reliability best_effort
ros2 topic hz /cmd_vel
ros2 topic echo /imu/data --qos-reliability best_effort
```

QoS IMU trên ESP32 là BEST_EFFORT; cmd_vel là RELIABLE. Serial Monitor in số IMU publish/cmd_vel receive mỗi giây; dự kiến ~20/~10 trên đường truyền ổn định. Nếu Agent mất nhiều lần, ESP32 khởi động lại và kết nối lại.

**Trạng thái:** đã triển khai firmware và cấu hình trên GitHub, **chưa xác nhận build/nạp hoặc tần số thực tế trên phần cứng**. IMU chỉ dùng cho test, không dùng để điều khiển hoặc định vị robot thực.
