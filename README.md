# ESP32-WROOM micro-ROS: IMU 20 Hz + cmd_vel 10 Hz

Demo hai chieu cho ESP32-WROOM (ESP32-DevKit) va PC chay ROS 2 Humble.

- ESP32 publish **/imu/data** (sensor_msgs/msg/Imu) moi 50 ms = **20 Hz**.
- PC subscribe /imu/data va publish **/cmd_vel** (geometry_msgs/msg/Twist) moi 100 ms = **10 Hz**.
- ESP32 subscribe /cmd_vel, ghi gia tri nhan qua UART; **khong dieu khien dong co**.
- IMU la du lieu **gia lap**: quaternion quay quanh Z, van toc goc, gia toc trong truong 9.81 m/s² va nhieu ngau nhien nho. Khong duoc dung de dinh vi/kiem soat robot that.

## So do

    ESP32-WROOM                         Ubuntu PC (ROS 2 Humble)
       |                                        |
       | /imu/data: Imu, 20 Hz                  |
       +---- micro-ROS -> Wi-Fi UDP -> Agent -->+---- rclpy subscribe
       |                                        |
       | /cmd_vel: Twist, 10 Hz                 |
       +<--- micro-ROS <- Wi-Fi UDP <- Agent <--+---- rclpy publish

Micro XRCE-DDS chay tren ESP32; micro-ROS Agent lam cau noi sang ROS 2.

## 1. Chuẩn bị project và submodule

Yeu cau: PlatformIO, ESP32-WROOM/ESP32-DevKit, Git, Wi-Fi 2.4 GHz,
Ubuntu PC da cai ROS 2 Humble, Docker de chay micro-ROS Agent.

```bash
git clone --recurse-submodules https://github.com/HoangAnh301194/MicroROS.git
cd MicroROS
# Neu clone truoc do:
git submodule update --init --recursive
```

Component da duoc pin tai commit 4ddd8c26e721662319ed8af981cb7cdc9ae05382
(nhanh Humble). Khong can tu dong update component.

Repo su dung PlatformIO espressif32 8.0.0 / ESP-IDF 6.1.
**Can kiem tra build tren may that:** component tai commit nay ghi nhan
kiem thu toi ESP-IDF 6.0, chua xac nhan tuong thich 6.1.
Neu gap loi do phien ban, co the thu pin platform = espressif32@7.0.1
(ESP-IDF 6.0.1) va tai tao sdkconfig/buid sach, nhung chi sau khi luu
ban sao cau hinh cu. Tuyet doi khong doi ROS distro tren ESP32 sang rolling
khi PC dang chay Humble.

## 2. Cau hinh Wi-Fi va dia chi Agent

ESP32 va PC phai cung mang co the truy cap UDP. Tim IPv4 LAN cua PC:

```bash
hostname -I
```

Chon IP Wi-Fi/Ethernet phu hop (vi du 192.168.1.10; khong chon 127.0.0.1).
Chay:

```bash
pio run -t menuconfig
```

Trong **micro-ROS Settings**:
- micro-ROS middleware: **Micro XRCE-DDS**.
- micro-ROS network interface: **WLAN interface**.
- WiFi Configuration: nhap **Wi-Fi SSID** va **Wi-Fi Password**.
- micro-ROS Agent IP: IP LAN cua PC (vi du 192.168.1.10).
- micro-ROS Agent Port: **8888**.
- Trong **micro-ROS demo task settings**, giu stack 16000 bytes, priority 5.

Luu va thoat. ESP32-WROOM chi ho tro Wi-Fi 2.4 GHz.

**Bao mat:** sdkconfig.esp32dev dang la file da duoc Git theo doi.
KHONG commit/push file nay sau khi nhap mat khau Wi-Fi.
Kiem tra git diff truoc khi git add/commit, va chi stage nhung file can thiet.
.gitignore khong the an file da duoc track.

## 3. Build va flash ESP32

micro-ROS component can catkin_pkg, colcon, lark va voi nhanh Humble nay
can empy<4. Cai trong **Python environment ma ESP-IDF/PlatformIO build
thuc su su dung** (khong nhat thiet la Python he thong):

```bash
python3 -m pip install catkin_pkg colcon-common-extensions lark 'empy<4'
pio run
pio run -t upload --upload-port /dev/ttyUSB0
pio device monitor -p /dev/ttyUSB0 -b 115200
```

Thay /dev/ttyUSB0 bang cong thuc te (/dev/ttyACM0 neu phu hop).
Cac package Python phai ton tai trong dung IDF virtual environment.

**Quan trong:** bat Agent (muc 4) truoc khi reset/boot ESP32,
vi demo co the dung khi tao ROS entities neu Agent chua san sang.

## 4. Agent (Terminal 1, tren PC)

```bash
docker run --rm -it --net=host microros/micro-ros-agent:humble \
  udp4 --port 8888 -v6
```

Docker --net=host dung tren Ubuntu/Linux. Neu firewal dang bat, cho phep
UDP port 8888. Su dung Agent **humble** cho ROS 2 Humble.
Sau do reset ESP32 de no ket noi voi Agent.

## 5. ROS 2 bridge (Terminal 2, tren PC)

```bash
source /opt/ros/humble/setup.bash
cd MicroROS
python3 host/imu_cmd_vel_bridge.py
```

Mac dinh host publish **Twist bang 0** o 10 Hz de test duong truyen.
Co the thu gia tri khac (chi de debug; firmware khong lai motor):

```bash
python3 host/imu_cmd_vel_bridge.py --ros-args \
  -p linear_x:=0.20 -p angular_z:=0.10
```

Khi khong nhan IMU qua 0.5 s, bridge van publish o 10 Hz nhung tu
dong dua linear.x/angular.z ve **0**. Tham so timeout: imu_timeout_s.

QoS:
- /imu/data: ESP32 BEST_EFFORT, PC subscription sensor_data BEST_EFFORT.
- /cmd_vel: PC RELIABLE, ESP32 subscription RELIABLE.

## 6. Xac minh

Mo Terminal 3:

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

Ket qua ky vong tren ket noi on dinh:
- /imu/data khoang **20 Hz**, chu ky **50 ms**.
- /cmd_vel khoang **10 Hz**, chu ky **100 ms**.
- UART ESP32 log "cmd_vel #N" va thong ke so ban tin nhan.
- Host log ti le IMU rx va cmd_vel tx moi giay.
Sai lech nho do scheduling/Wi-Fi la binh thuong; do thuc te bang ros2 topic hz.

## Gioi han va xu ly loi

- IMU gia, khong phai do thuc. Covariance chi la gia tri gia dinh de test.
- Header stamp chi co epoch ROS hop le neu sync voi Agent thanh cong; neu khong bang 0.
- Demo toi gian chua co tu khoi tao lai ROS entities khi Agent/Wi-Fi ngat.
  Khi Agent khoi dong lai, reset ESP32. Chung ta co the bo sung state machine reconnect sau.
- PlatformIO/ESP-IDF 6.1 va component pin hien tai chua duoc test build/hardware o day.
- Neu topic hien ra nhung echo IMU khong co data, xem lai **QoS BEST_EFFORT**.
- Neu ESP32 ket noi Wi-Fi nhung khong thay node, kiem tra Agent IP (khong dung
  localhost), UDP 8888, tuong lua, ROS_DOMAIN_ID, va Agent dang chay.
