# robowind Perception AI toolkit (Peep)

# Install ROS
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
  git clone https:/robowind/peep/peep.git
  cd peep
  git config credential.helper store
  cd ~/peep_ws/src
  git clone https://robowind/peep/peep_models.git
  cd ~/peep_ws/src/peep_models
  git lfs pull
```


# Build/Install DLIB
```
  cd peep/peep_external/dlib/dlib/
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
  make -j4
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
```
  cd ~/peep_ws/src/peep/peep_external/librealsense/
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j4
  sudo make install
```

## Setup Robotupstart
```
rosrun robot_upstart install peep_launch/launch/local_node_intel_pipeline.launch --job peep --setup /home/robowind/peep_ws/src/peep/source_intel.sh
```
