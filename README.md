
# ROS2 ORB SLAM3

## 1. Installation:

```bash
#!/bin/bash

# Set non-interactive mode
export DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages
sudo apt update && sudo apt install -y curl gnupg2 lsb-release build-essential cmake git python3 python3-pip python3-rosdep python3-opencv libeigen3-dev libssl-dev libdlib-dev libopencv-dev ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-vision-opencv

# Ensure Python dependencies are installed
sudo -H pip3 install --break-system-packages wheel

# Install Pangolin
cd ~
mkdir -p ~/softwares
cd ~/softwares

# Increase Git buffer size to handle large files
git config --global http.postBuffer 524288000

# Shallow clone Pangolin repository
if ! git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git; then
    echo "Failed to clone Pangolin repository. Please check your network connection."
    exit 1
fi

cd Pangolin
sudo apt install -y libglew-dev

# Ensure the install_prerequisites.sh script is executable
if [ ! -f ./scripts/install_prerequisites.sh ]; then
    echo "install_prerequisites.sh script not found. Please check the Pangolin repository."
    exit 1
fi

chmod +x ./scripts/install_prerequisites.sh
./scripts/install_prerequisites.sh recommended

mkdir -p build
cd build
cmake ..
cmake --build . -j$(nproc)
sudo cmake --install .

# Configure Dynamic Library Path
if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
    sudo ldconfig
fi

# Add Pangolin, OpenCV, and Python site-packages to CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/lib/cmake/Pangolin:/usr/include/opencv4:/usr/local/lib/python3.12/dist-packages

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Update Cache
sudo ldconfig

# Source the correct ROS 2 distribution setup script
source /opt/ros/jazzy/setup.bash

# Create the workspace for ORB-SLAM3
mkdir -p ~/ros2/orbslam3_ws/src
cd ~/ros2/orbslam3_ws/src

# Retry cloning ORB-SLAM3 repository in case of network issues
until git clone --depth 1 https://github.com/Kawai-Senpai/ros2_orb_slam3.git; do
    echo "Retrying clone of ORB-SLAM3 repository..."
    sleep 5
done

cd ros2_orb_slam3
pip install --break-system-packages -r requirements.txt

# Manually install libssl-dev to resolve libcrypto dependency
sudo apt install libssl-dev

# Install ROS 2 dependencies for ORB-SLAM3
cd ~/ros2/orbslam3_ws
rosdep install -i --from-path src --rosdistro jazzy -y

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/jazzy

# Build the workspace
colcon build --symlink-install --executor sequential --parallel-workers 1

# Source the workspace
if ! grep -q "source ~/ros2/orbslam3_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2/orbslam3_ws/install/setup.bash" >> ~/.bashrc
fi
source ~/ros2/orbslam3_ws/install/setup.bash

# Create symbolic link to correct opencv version
sudo ln -s /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.6.0 /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5d

echo "ORB-SLAM3 installation completed!"
```

## 2. Monocular Example:

Run the builtin example to verify the package is working correctly
In one terminal [cpp node]

```bash
cd ~/ros2/oebslam3_ws/
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

In another terminal [python node]

```bash
cd ~/ros2/oebslam3_ws/
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```

Both nodes would perform a handshake and the VSLAM framework would then work as shown in the following video clip

https://github.com/Mechazo11/ros2_orb_slam3/assets/44814419/af9eaa79-da4b-4405-a4d7-e09242ab9660