#!/bin/bash

# "###############################"
# "# Seaos Inc.                  #"
# "# Automatic Xavier type       #"
# "# keycart installer           #"
# "# Created by Yusuke Kobayashi #"
# "# y-kobayashi@seaos.co.jp     #"
# "###############################"

# Change nvpmodel to max mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Prepare the enviroment
git config --global credential.helper cache

# Installation SSD and Wifi card

sudo parted /dev/nvme0n1 mklabel gpt
sudo parted /dev/nvme0n1 mkpart xavier_ssd 0% 100%
sudo mkfs.ext4 /dev/nvme0n1p1

uuid=(`sudo blkid /dev/nvme0n1p1  | xargs`)
for i in ${uuid[@]}
do
    if [[ $i =~ ^UUID=* ]]; then
        uuid="$i /xavier_ssd ext4 defaults 0 2"
        break
    fi
done

sudo mkdir /xavier_ssd
sudo mount /dev/nvme0n1p1 /xavier_ssd # <-- At here, /xavier_ssd won't belong to nvidia in above way...
sudo chown nvidia:nvidia /xavier_ssd
sudo chmod 755 /xavier_ssd

sudo cp /etc/fstab /etc/fstab.bkup
sudo sh -c "echo $uuid >> /etc/fstab"
## to here

cd /home
cp -r nvidia/ /xavier_ssd/
sudo mv nvidia/ nvidia_bkup/
sudo ln -s /xavier_ssd/nvidia

# Compile and install OpenCV 3.4.6
sudo apt purge libopencv*
mkdir ~/src
cd ~/src
git clone https://github.com/yuusuke0126-seaos/buildOpenCVXavier.git
cd buildOpenCVXavier/
git checkout 3.4.6
./buildOpenCV.sh

cd ~
sed -i -e "s/LD_LIBRARY_PATH=/LD_LIBRARY_PATH=/usr/local/lib:/g" .bashrc
sudo apt install libboost-all-dev libpcl-dev

# Installation ZED SDK
cd ~/src
mkdir ZED_SDK
cd ZED_SDK
wget https://www.stereolabs.com/developers/downloads/ZED_SDK_JP4.2_v2.8.1.run
chmod +x ZED_SDK_JP4.2_v2.8.1.run
./ZED_SDK_JP4.2_v2.8.1.run

# Compile and install gtsam, g2o, and rtabmap
cd ~/src
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam/
git checkout 4.0.0-alpha2
mkdir build
cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make
sudo make install

cd ~/src
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o/
git checkout master
sudo apt install libeigen3-dev
sudo apt install libsuitesparse-dev qtdeclarative5-dev qt5-qmake
sudo apt install libqglviewer-headers
mkdir build
cd build
cmake -DBUILD_WITH_MARCH_NATIVE=OFF ..
make
sudo make install

cd ~/src
git clone https://github.com/SeaosRobotics/rtabmap.git
cd rtabmap/build
cmake ..
make
sudo make install

# ROS installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-ros-base
sudo rosdep init
rosdep update
source /opt/ros/melodic/setup.bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

cd ~/src
git clone https://github.com/yuusuke0126-seaos/setupJetsonTX2-Xavier.git
cd setupJetsonTX2-Xavier
git checkout develop
cp .bash_ros ~/
echo "source ~/.bash_ros" >> ~/.bashrc

mkdir -p ~/ros/catkin_ws/src
cd ~/ros/catkin_ws/
catkin_make
bash
sudo apt install ros-melodic-rosserial-python

# Logiler pkgs installation
## TODO git config not to input usrname and pswd for each times
cd ~/ros/catkin_ws/src
git clone https://github.com/SeaosRobotics/cast_milestones.git
git clone https://github.com/rst-tu-dortmund/costmap_converter.git
git clone https://github.com/ros-perception/depthimage_to_laserscan.git
git clone https://github.com/ros-perception/image_transport_plugins.git
git clone https://github.com/SeaosRobotics/key_cart.git
git clone https://github.com/SeaosRobotics/logiler_bringup.git
git clone https://github.com/SeaosRobotics/logiler_description.git
git clone https://github.com/SeaosRobotics/logiler_navigation.git
git clone https://github.com/ros-planning/navigation.git
git clone https://github.com/DLu/navigation_layers.git
git clone https://github.com/SeaosRobotics/obstacle_monitor.git
git clone https://github.com/SeaosRobotics/obstacle_msgs.git
git clone https://github.com/SeaosRobotics/pipeline_planner.git
git clone https://github.com/SeaosRobotics/roboline.git
git clone https://github.com/GT-RAIL/robot_pose_publisher.git
git clone https://github.com/SeaosRobotics/ros_ultrasonic_msgs.git
git clone https://github.com/SeaosRobotics/rtabmap_ros.git
git clone https://github.com/SeaosRobotics/teb_local_planner.git
git clone https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/SeaosRobotics/zed-ros-wrapper.git

cd cast_milestones; git checkout feature/service;
cd ../depthimage_to_laserscan; git checkout melodic-devel;
cd ../image_transport_plugins; git checkout indigo-devel;
cd ../key_cart; git checkout develop;
cd ../logiler_bringup; git checkout develop;
cd ../logiler_description; git checkout release/0.1.0.1;
cd ../logiler_navigation; git checkout release/0.1.0.0;
cd ../navigation; git checkout melodic-devel;
cd ../navigation_layers; git checkout melodic;
cd ../obstacle_monitor; git checkout feature/dumpcostpoints;
cd ../obstacle_msgs; git checkout release/0.1.0.0;
cd ../pipeline_planner; git checkout develop;
cd ../roboline; git checkout develop;
cd ../ros_ultrasonic_msgs; git checkout develop;
cd ../teb_local_planner; git checkout melodic-devel;
cd ../vision_opencv; git checkout melodic;
cd ../zed-ros-wrapper; git checkout develop;

cd ~/ros/catkin_ws
rosdep install -r --from-paths src --ignore-src # Be careful not to install libopencv*
sudo apt purge ros-melodic-libg2o
catkin_make

# udev rules and k2k setting
sudo sh -c 'echo "nvidia ALL=NOPASSWD: ALL" >> /etc/sudoers.d/logilerOverrides'
sudo usermod -aG dialout nvidia
cd ~/src
git clone https://github.com/SeaosRobotics/k2k.git
cd k2k
git checkout develop
cd service
./install.sh

# rmc and pm2 installation
cd ~/src
git clone https://github.com/SeaosRobotics/logiler_utils.git
cd logiler_utils
git checkout feature/installer
cd script/include
sed -i -e "s/  exit/#  exit/" nodejs.sh
sed -i -e "s/\$HOME/\/xavier_ssd\/nvidia/" nodejs.sh
chmod +x nodejs.sh
./nodejs.sh

cd ~
sed -i -e "s/\$HOME/\/xavier_ssd\/nvidia/" .bashrc
