# robowind Perception AI toolkit (Peep)

Perceptual Element Ensemble Pipeline (peep), which is platform than includes:

    Modular Perception Elements
    Temporal & Spatial Elements & Fuses any cameras.   
    Algorithm Agnostic & Standardized Scenegraph   


In details we have four main components

    Rover system
    Drone system
    Ground Control
    User ipad/iphone/Android Interface

Peep its structure as follow:

    peep_control: Contains all the system controller

    ac_motor_controller
    dc_motor_controller
    arduino
    robot_joystick controller

    peep_msg: Contains all the general message and fuse scenegraph

    peep_external: Third parties software like realsense drivers and deep AI frameworks
    
    peep_services:
    1. ros publish to webserver to be able to display on ipad
    2. ground control system drone rover

    peep_simulation: interaction with simulation system
    
    peep_slam: Location and Mapping

At Rover and the Drone system is organized as:

    Autonomous system
    Drive system
    Odometry system

On the Autonomous system we have following nodes:

    Multiple camera Ros node (For depth and Slam) 
    RGB_odometry. Its computed using visual features and depth information 
    rtabmap: Creates a 3d point cloud of environments and 2d grid for navigation
    move_base: Given a goal in the world it will attempt to control robot to reach 
    cost_map: Creates a costmap of the world and represents how safe its to be on any location
    Global planner: Finds minimum plan from start point to end point
    Local Planner: Given a plan and a costmap local planner produces velocity and commands to send to motor
    3D location and mapping with Lidar:

    Octo mapping 
    Trajectory visualization 
    Laser mapping 
    Odom estimation 


Odometry system:

    IMU hardware:

    Local location: Fuse local position in the odom frame
    Global location: Fuse globally position in the map frame

    GPS hardware

       Navigation transform: Set navigation gals using latitude and longitude

Drive system we have:

     Xbox controller: joystick control

On Drone we have done bridge Mission planner with ground control




Choose ROS version:

# Install ROS Melodic
```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  sudo apt update
  sudo apt install ros-melodic-desktop-full
  sudo rosdep init
  rosdep update
  echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools ros-melodic-robot-upstart
```

# Install ROS Noetic
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

```

# Install OpenVINO
```
  wget http://registrationcenter-download.intel.com/akdlm/irc_nas/15013/l_openvino_toolkit_p_2018.5.445.tgz
  tar -zxf l_openvino_toolkit_p_2018.5.445.tgz
  cd l_openvino_toolkit_p_2018.5.445
  sudo -E ./install_cv_sdk_dependencies.sh
  sudo ./install.sh
  echo "source /opt/intel/computer_vision_sdk/bin/setupvars.sh" >> ~/.bashrc
  source ~/.bashrc
  cd /opt/intel/computer_vision_sdk/install_dependencies/
  sudo ./install_NEO_OCL_driver.sh
  cd /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/install_prerequisites
  sudo ./install_prerequisites.sh 
```

# Set up System (Deps, Git, SSH key)
```
  sudo apt install gtk+-3.0 libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libc++-dev libc++abi-dev python-pip libgflags-dev libopenblas-dev
  git config --global user.email "$USER@$HOSTNAME" 
  git config --global user.name "$USER@$HOSTNAME"  
  ssh-keygen -t rsa -C "$USER@$HOSTNAME"
```

# Install git lfs
```
    sudo add-apt-repository ppa:git-core/ppa 
    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
    sudo apt install git-lfs
    git lfs install
```

# Create a new workspace in 'peep_ws'.
```
mkdir peep_ws
cd peep_ws
wstool init src
```

# Clone and install PEEP
```
  cd ~/peep_ws/src
  git clone https://github.com/robo-wind/peep/peep.git
  cd peep
  git config credential.helper store
  cd ~/peep_ws/src
  git clone https://github.com/robo-wind/peep/peep_models.git
  cd ~/peep_ws/src/peep_models
  git lfs pull
```


# Build/Install DLIB
```
  cd peep/peep_external/dlib/dlib/
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
  make -j4:wq
  sudo make install
```
# Build/Install ccore_all
```
  cd ~/peep_ws/src/peep/peep_external/ccore_all
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DLIBBSON_SUPPORT=True
  make -j4
  sudo make install

```


# Optional

## Build/Install librealsense

# peep_external/librealsense/
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j4
  sudo make install
  
  ## Setup Robotupstart
  
rosrun robot_upstart install peep_launch/launch/local_node_intel_pipeline.launch --job peep --setup /home/robowind/peep_ws/src/peep/source_intel.sh

#  3-D Localization and Mapping:

## 2. Prerequisites


### 2.1 **Ubuntu** and **ROS**

Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Tested with 1.8.1

### 2.4 **OctoMap**
Follow [OctoMap Installation](http://wiki.ros.org/octomap).

```bash
$ sudo apt install ros-melodic-octomap*
```

### 2.5. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed


## 4.4 Launch ROS
```
    roslaunch peep peep_slam_L515.launch
```



  
  
  
  
```
