# ESP32-WROOM + micro-ROS: IMU 20 Hz và cmd_vel 10 Hz

Dự án kiểm thử truyền thông hai chiều giữa **ESP32-WROOM** (ESP-IDF/PlatformIO) và **Ubuntu chạy ROS 2 Humble**.

| Thiết bị | Vai trò | Topic | Kiểu bản tin | Tần số |
|---|---|---|---|---|
| ESP32 → PC | Publisher IMU giả lập | /imu/data | sensor_msgs/msg/Imu | 20 Hz |
| PC → ESP32 | Publisher vận tốc thử nghiệm | /cmd_vel | geometry_msgs/msg/Twist | 10 Hz |

ESP32 nhận lệnh và ghi ra UART, **không điều khiển động cơ**. Giá trị IMU chỉ dùng để kiểm tra micro-ROS, không phải dữ liệu từ cảm biến thật.

## 1. Kiến trúc

    ESP32-WROOM                          Ubuntu PC / ROS 2 Humble
      Publisher /imu/data                    Subscriber /imu/data
                |                                     ^
                +---- Wi-Fi UDP -> micro-ROS Agent ---+
                |                                     |
      Subscriber /cmd_vel                    Publisher /cmd_vel
                ^                                     |
                +---- Wi-Fi UDP <- micro-ROS Agent ---+

- `/imu/data`: bản tin IMU chuẩn, có `header.frame_id = imu_link`, quaternion mô phỏng góc quay quanh trục Z, vận tốc góc và gia tốc trọng trường kèm nhiễu nhỏ.
- `/cmd_vel`: `geometry_msgs/msg/Twist`. Phía PC gửi mỗi 100 ms; mặc định mọi vận tốc đều bằng 0 để kiểm tra đường truyền an toàn.
- QoS: IMU **BEST_EFFORT** ở cả hai đầu; lệnh `cmd_vel` dùng **RELIABLE**.
- ESP32 đồng bộ dấu thời gian với Agent; nếu không đồng bộ được, `header.stamp` bằng 0.

## 2. Chuẩn bị

Cần PlatformIO Core hoặc VS Code + PlatformIO, Git, ESP32-WROOM/ESP32-DevKit, Wi-Fi **2.4 GHz**, Ubuntu/ROS 2 Humble và Docker (để chạy Agent).

```bash
git clone https://github.com/HoangAnh301194/MicroROS.git
cd MicroROS

# Cho đến khi nhánh tính năng được merge vào main:
git switch feature/esp32-imu20-cmdvel10
git submodule update --init --recursive

# Sau khi được merge vào main, có thể dùng:
# git clone --recurse-submodules https://github.com/HoangAnh301194/MicroROS.git
```

Dự án đã tham chiếu submodule `micro_ros_espidf_component` tại commit `4ddd8c26e721662319ed8af981cb7cdc9ae05382` (nhánh Humble). File `.gitmodules` được bổ sung để clone submodule chính xác.

**Lưu ý tương thích:** `platformio.ini` hiện dùng `espressif32@8.0.0` (ESP-IDF 6.1), trong khi component đang dùng chỉ công bố kiểm thử đến ESP-IDF 6.0. Chưa thể xác nhận build thành công trên 6.1 nếu không kiểm tra thực tế. Nếu gặp lỗi phiên bản, thử `espressif32@7.0.1` (ESP-IDF 6.0.1) và tái tạo cấu hình trong một bản sao dự án. Giữ micro-ROS **Humble**, không tự đổi sang Rolling.

## 3. Cấu hình Wi-Fi và micro-ROS Agent

ESP32 và PC phải cùng mạng nội bộ (hoặc có định tuyến UDP qua lại). Xác định địa chỉ IPv4 của PC:

```bash
hostname -I
```

Chọn địa chỉ mạng LAN thích hợp, ví dụ `192.168.1.10` (không dùng `127.0.0.1`). Tại thư mục dự án, chạy:

```bash
pio run -t menuconfig
```

Trong mục **micro-ROS Settings**, thiết lập:

1. Middleware: **Micro XRCE-DDS**.
2. Network interface: **WLAN interface**.
3. WiFi Configuration: SSID và mật khẩu Wi-Fi 2.4 GHz.
4. Micro-ROS Agent IP: địa chỉ IPv4 của PC; port: **8888**.
5. Mục **micro-ROS demo task settings**: stack **16000 bytes**, priority **5**.

Lưu cấu hình. Nếu thay đổi IP của PC, cần cập nhật Agent IP rồi nạp lại ESP32.

**Bảo mật:** `sdkconfig.esp32dev` đang là file được Git theo dõi. Sau khi nhập mật khẩu Wi-Fi, không đưa thay đổi của file này lên GitHub. Kiểm tra `git diff` và `git status` trước khi commit. `.gitignore` không thể bỏ theo dõi một file đã track.

## 4. Biên dịch và nạp ESP32

Component micro-ROS cần các công cụ Python `catkin_pkg`, `colcon`, `lark` và `empy<4` (với phiên bản Humble đang ghim). Phải cài vào **đúng môi trường Python mà ESP-IDF/PlatformIO sử dụng**.

```bash
python3 -m pip install catkin_pkg colcon-common-extensions lark 'empy<4'
pio run
pio run -t upload --upload-port /dev/ttyUSB0
pio device monitor -p /dev/ttyUSB0 -b 115200
```

Thay `/dev/ttyUSB0` bằng cổng USB-UART thực tế. Nếu thiếu thư viện Python khi build, hãy xác định Python interpreter do PlatformIO/ESP-IDF gọi và cài gói vào môi trường đó thay vì chỉ cài vào Python hệ thống.

**Thứ tự khởi động:** chạy micro-ROS Agent trước, sau đó khởi động hoặc reset ESP32. Phiên bản thử nghiệm này chưa có cơ chế tự tạo lại node khi Agent mất kết nối.

## 5. Chạy micro-ROS Agent trên PC — Terminal 1

```bash
docker run --rm -it --net=host microros/micro-ros-agent:humble \
  udp4 --port 8888 -v6
```

Lệnh Docker dùng chế độ host network của Linux. Nếu có firewall, cho phép UDP port 8888. Đảm bảo PC và ESP32 truy cập được nhau qua mạng.

## 6. Chạy node ROS 2 trên PC — Terminal 2

```bash
source /opt/ros/humble/setup.bash
cd MicroROS
python3 host/imu_cmd_vel_bridge.py
```

Node sẽ **subscribe IMU** và **publish cmd_vel đúng chu kỳ cấu hình 100 ms**. Mặc định lệnh phát ra bằng 0. Để thử truyền số khác 0 (firmware chỉ ghi log, không điều khiển động cơ):

```bash
python3 host/imu_cmd_vel_bridge.py --ros-args \
  -p linear_x:=0.20 -p angular_z:=0.10
```

Nếu trong **0,5 giây** không nhận được IMU mới, node vẫn phát `cmd_vel` ở 10 Hz nhưng bắt buộc vận tốc về 0. Có thể điều chỉnh ngưỡng bằng tham số ROS `imu_timeout_s`.

## 7. Kiểm thử — Terminal 3

```bash
source /opt/ros/humble/setup.bash

ros2 node list
ros2 topic list -t
ros2 topic echo /imu/data --qos-reliability best_effort
ros2 topic hz /imu/data --qos-reliability best_effort
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
ros2 topic info /imu/data --verbose
ros2 topic info /cmd_vel --verbose
```

Kết quả kỳ vọng trên kết nối ổn định:

- `/imu/data`: khoảng **20 Hz**, chu kỳ **50 ms**.
- `/cmd_vel`: khoảng **10 Hz**, chu kỳ **100 ms**.
- PC in thống kê **IMU RX** và **cmd_vel TX** mỗi giây.
- UART ESP32 hiện `cmd_vel #N` với `linear.x`, `angular.z` nhận được và thống kê số lượng mỗi giây.

Tần số đo thực tế có thể lệch một chút do FreeRTOS, Wi-Fi và cơ chế lập lịch của ROS 2. Các con số 20/10 Hz hiện là **tần số thiết kế**, chưa phải kết quả thử nghiệm trên phần cứng.

## 8. Xử lý sự cố và giới hạn

- **Không thấy topic:** kiểm tra SSID, Agent IP, cổng UDP 8888, firewall và micro-ROS Agent.
- **Topic IMU hiện nhưng không nhận dữ liệu:** kiểm tra QoS. Publisher là BEST_EFFORT nên subscriber không được yêu cầu RELIABLE.
- **PC dùng ROS_DOMAIN_ID khác 0:** cần bảo đảm các node ROS 2 và Agent được chạy trong cùng miền ROS 2 thích hợp; đồng thời kiểm tra biến môi trường ở terminal/container.
- **Agent restart:** reset ESP32, do bản này chưa có state machine khôi phục kết nối.
- **Timestamp bằng 0:** kiểm tra đồng bộ epoch với Agent.
- **Build lỗi:** phiên bản ESP-IDF 6.1 chưa được component hiện tại xác nhận chính thức; kiểm tra log build trước khi thay đổi công cụ. Nếu firmware vượt kích thước phân vùng ứng dụng, cần cấu hình phân vùng lớn hơn phù hợp với dung lượng flash thực tế.
- **IMU là dữ liệu mô phỏng:** covariance cũng là giá trị giả định, không được dùng làm phép đo để điều khiển robot thực.
