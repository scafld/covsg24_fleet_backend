ROS2 based backend server of the application, including RMF & FF server setup, traffic maps and RMF GUI configurations

# Installation
Works on ROS 2 Foxy on Ubuntu 20.04

```bash
sudo apt-get install python3-pip
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
git clone --recursive https://github.com/project-covsg24/covsg24_fleet_backend
```

Ensure all ROS 2 prerequisites are fulfilled,

```bash
cd <your-catkin-workspace>
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
npm install --prefix src/covsg24_fleet_backend/rmf/rmf_demos/rmf_demo_panel/rmf_demo_panel/static/
npm run build --prefix src/covsg24_fleet_backend/rmf/rmf_demos/rmf_demo_panel/rmf_demo_panel/static/

colcon build --packages-select rmf_demo_panel
```

# Usage examples
## Robot: Jackal
Running Clearpath Jackal in Delta 3rd floor. 
### With Real Robot
With a real robot that's all you need to set up on the server's side
```bash
ros2 launch covsg24_fleet_server jackal_in_delta.launch.xml
```

### Jackal Simulaion
If Jackal is running in ROS1 simulation then:
```bash
ros2 launch covsg24_fleet_server jackal_in_delta.launch.xml use_sim_time:=true
```
and set up the ROS1-ROS2 bridge (needed to propagate /clock topic across the system). This example is based on ***ROS2 Foxy*** and ***ROS1 Noetic***:
```bash
sudo apt install ros-foxy-ros1-bridge

# First source ROS1
source /opt/ros/noetic/setup.bash

# Then source ROS2
source /opt/ros/foxy/setup.bash

# If ROS1 and ROS2 are running on different machines then change "localhost" to appropriate ROS1 master ip
export ROS_MASTER_URI=http://localhost:11311

# Run the bridge
ros2 run ros1_bridge dynamic_bridge
```

### Command the Robot via web GUI
On the local machine, just open up the browser and go to `http://localhost:5000/`. On a different device (which should still be in the same network) replace `localhost` with the server's ip address.

<br/>

***Now everything should be set on the server/backend side. Follow the client-side instructions to get the robot going.*** 
