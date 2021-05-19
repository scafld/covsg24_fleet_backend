ROS2 based backend server of the application, including RMF & FF server setup, traffic maps and RMF GUI configurations

# Installation
Works on ROS 2 Foxy on Ubuntu 20.04

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

Setup your computer to accept Gazebo packages from packages.osrfoundation.org.

```bash
sudo apt update
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install all non-ROS dependencies of RMF packages,

```bash
sudo apt update && sudo apt install \
  git cmake python3-vcstool curl \
  qt5-default \
  python3-shapely python3-yaml python3-requests \
  -y
sudo apt-get install python3-colcon*
```

Install this repository

```bash
git clone --recursive https://github.com/project-covsg24/covsg24_fleet_management
```

Ensure all ROS 2 prerequisites are fulfilled,

```bash
cd ~/rmf_demos_ws
rosdep install --from-paths src --ignore-src --rosdistro foxy -yr
```

Build the packages

```bash
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Install RMF Panel Dashbaord

```bash
python3 -m pip install flask-socketio

# change the npm prefix according to the path to "rmf_demo_panel/static/"
npm install --prefix src/rmf/rmf_demos/rmf_demo_panel/rmf_demo_panel/static/
npm run build --prefix src/rmf/rmf_demos/rmf_demo_panel/rmf_demo_panel/static/

colcon build --packages-select rmf_demo_panel
```

## TODO automatically setting up gazebo models via cmakelists