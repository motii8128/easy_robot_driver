# easy_robot_driver
easy robot software with safe_drive and Unity

# Usage
```
cd my_ws/src
```
```
git clone --recursive https://github.com/UrgNetworks/urg_node2.git
git clone https://github.com/motii8128/easy_robot_driver.git
```
```
rosdep update
rosdep install -i --from-paths urg_node2
```
```
cd ..
colcon build --symlink-install
source ./install/setup.bash
```
