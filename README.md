# easy_robot_driver
easy robot software with safe_drive and Unity

# Usage
create my_workspace
```
mkdir -p my_ws/src
```
move directory
```
cd my_ws/src
```
Get source
```
git clone --recursive https://github.com/UrgNetworks/urg_node2.git
git clone https://github.com/motii8128/easy_robot_driver.git
git clone https://github.com/motii8128/haya_imu_ros2.git
git clone https://github.com/soarbear/haya_imu_msgs.git
```
Set serial
```
chmod +x ~/my_ws/src/haya_imu_ros2/script/create_rules.sh
~/my_ws/src/haya_imu_ros2/script/create_rules.sh
sudo udevadm control --reload-rules && udevadm trigger
reboot
```
Update dependencies
```
rosdep update
rosdep install -i --from-paths urg_node2
```
Build
```
cd ..
colcon build --symlink-install
source ./install/setup.bash
```

# detail
please see [this](https://github.com/soarbear/haya_imu_ros2)..
