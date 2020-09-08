#!/bin/bash

# "###############################"
# "# Seaos Inc.                  #"
# "# Automatic Xavier type       #"
# "# keycart installer           #"
# "# Created by Yusuke Kobayashi #"
# "# y-kobayashi@seaos.co.jp     #"
# "###############################"

echo -e "\e[33m###################################"
echo -e "\e[33m# Seaos Inc.                      #"
echo -e "\e[33m# Automatic st-keycart installer. #"
echo -e "\e[33m###################################"

# Change nvpmodel to max mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Prepare the enviroment
git config --global credential.helper cache

# Installation SSD and Wifi card

echo -e "\e[33m#####################################"
echo -e "\e[33m# SSD installation and HOME setting #"
echo -e "\e[33m#####################################"

sudo parted /dev/nvme0n1 mklabel gpt
sleep 1
sudo parted /dev/nvme0n1 mkpart xavier_ssd 0% 100%
sleep 2
sudo mkfs.ext4 /dev/nvme0n1p1

uuid=(`sudo blkid /dev/nvme0n1p1  | xargs`)
for i in ${uuid[@]}
do
    if [[ $i =~ ^UUID=* ]]; then
        uuid="$i /xavier_ssd ext4 defaults 0 2"
        break
    fi
done

if [[ $uuid =~ ^UUID=* ]]; then
    echo "Format SSD is succeeded!"
else
    echo "Somthing error occurs during SSD format!"
    exit 1
fi

sudo mkdir /xavier_ssd
sudo mount /dev/nvme0n1p1 /xavier_ssd
sudo chown nvidia:nvidia /xavier_ssd
sudo chmod 755 /xavier_ssd

sudo cp /etc/fstab /etc/fstab.bkup
sudo sh -c "echo $uuid >> /etc/fstab"

cd /home
cp -r nvidia/ /xavier_ssd/
sudo mv nvidia/ nvidia_bkup/
sudo ln -s /xavier_ssd/nvidia

# Make swapfile

mkdir /xavier_ssd/swap_dir
mkdir ~/src
cd ~/src
git clone https://github.com/JetsonHacksNano/installSwapfile.git
cd installSwapfile/
./installSwapfile.sh -d /xavier_ssd/swap_dir -s 10
sudo sed -i -e "/^\/xavier_ssd\/swap_dir/s/defaults/defaults,pri=10/" /etc/fstab

# Compile and install OpenCV 3.4.6

echo -e "\e[33m#####################################"
echo -e "\e[33m# OpenCV and ZED SDK installation   #"
echo -e "\e[33m#####################################"

sudo apt purge libopencv*
sudo apt autoremove
cd ~/src
git clone https://github.com/yuusuke0126/buildOpenCVXavier.git
cd buildOpenCVXavier/
git checkout 3.4.6
./buildOpenCV.sh

sudo apt install libboost-all-dev libpcl-dev

# Installation ZED SDK
cd ~/src
mkdir ZED_SDK
cd ZED_SDK
wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/2.8/ZED_SDK_JP4.2_v2.8.1.run
chmod +x ZED_SDK_JP4.2_v2.8.1.run
./ZED_SDK_JP4.2_v2.8.1.run
cd ~
sed -i -e "s/LD_LIBRARY_PATH=/LD_LIBRARY_PATH=\/usr\/local\/lib:/g" .bashrc
source ~/.bashrc

# Compile and install gtsam, g2o, rtabmap, apriltag, logger, and monitoring
echo -e "\e[33m#########################################################"
echo -e "\e[33m# gtsam, g2o, rtabmap, apriltag, logger, and monitoring #"
echo -e "\e[33m#########################################################"


cd ~/src
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam/
git checkout 4.0.0-alpha2
mkdir build
cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j7
sudo make install

cd ~/src
git clone https://github.com/SeaosRobotics/g2o.git
cd g2o/
git checkout master
sudo apt install libeigen3-dev
sudo apt install libsuitesparse-dev qtdeclarative5-dev qt5-qmake
sudo apt install libqglviewer-headers
mkdir build
cd build
cmake -DBUILD_WITH_MARCH_NATIVE=OFF ..
make -j7
sudo make install

cd ~/src
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo/
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j7
sudo make install

cd ~/src
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher/
mkdir build
cd build/
cmake ..
make -j7
sudo make install

cd ~/src
git clone https://github.com/SeaosRobotics/rtabmap.git
cd rtabmap/
git checkout develop
cd build/
cmake ..
make -j7
sudo make install

cd ~/src
git clone https://github.com/SeaosRobotics/apriltag.git
cd apriltag/
cmake .
sudo make install

sudo apt install python-pip

cd ~/src
git clone https://github.com/SeaosRobotics/logger.git
cd logger/
git checkout v0.0.4
python2.7 setup.py bdist_egg --exclude-source-files

cd ~/src
git clone https://github.com/SeaosRobotics/zed-python-api.git
cd zed-python-api
python -m pip install cython numpy
python setup.py build
sudo python setup.py install

cd ~/src
git clone https://github.com/SeaosRobotics/monitoring.git
cd monitoring
git checkout v0.0.7
python2.7 setup.py --user nvidia
sudo systemctl start monitor.service


# ROS installation
echo -e "\e[33m#####################################"
echo -e "\e[33m# ROS and logiler pkgs installation #"
echo -e "\e[33m#####################################"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-ros-base
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
source /opt/ros/melodic/setup.bash

cd ~/src
git clone https://github.com/SeaosRobotics/setupJetsonTX2-Xavier.git
cd setupJetsonTX2-Xavier
git checkout feature/python_rmc  # this line is just temporary, after merge to master will be removed
cp .bash_ros ~/
echo "source ~/.bash_ros" >> ~/.bashrc

sudo apt install ros-melodic-rosserial-python ros-melodic-kobuki-msgs
sudo apt install ros-melodic-velodyne-pointcloud
mkdir -p ~/ros/catkin_ws/src
cd ~/ros/catkin_ws/
catkin_make
source ~/.bashrc

# Logiler pkgs installation
## TODO git config not to input usrname and pswd for each times
cd ~/ros/catkin_ws/src
git clone https://github.com/SeaosRobotics/apriltag_ros.git
git clone https://github.com/SeaosRobotics/cast_milestones.git
git clone https://github.com/rst-tu-dortmund/costmap_converter.git
git clone https://github.com/ros-perception/depthimage_to_laserscan.git
git clone https://github.com/ros-perception/image_transport_plugins.git
git clone https://github.com/SeaosRobotics/key_cart.git
git clone https://github.com/SeaosRobotics/logiler_bringup.git
git clone https://github.com/SeaosRobotics/logiler_description.git
git clone https://github.com/SeaosRobotics/logiler_navigation.git
git clone https://github.com/ros-planning/navigation.git
git clone https://github.com/SeaosRobotics/obstacle_msgs.git
git clone https://github.com/SeaosRobotics/pipeline_planner.git
git clone https://github.com/SeaosRobotics/range_sensor_layer.git
git clone https://github.com/SeaosRobotics/roboline.git
git clone https://github.com/GT-RAIL/robot_pose_publisher.git
git clone https://github.com/SeaosRobotics/ros_ultrasonic_msgs.git
git clone https://github.com/SeaosRobotics/rtabmap_ros.git
git clone https://github.com/SeaosRobotics/teb_local_planner.git
git clone https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/SeaosRobotics/zed-ros-wrapper.git

git clone https://github.com/SeaosRobotics/obstacle_monitor.git
git clone https://github.com/SeaosRobotics/pin_stop_points.git

cd cast_milestones; git checkout feature/service;
cd ../depthimage_to_laserscan; git checkout melodic-devel;
cd ../image_transport_plugins; git checkout indigo-devel;
cd ../key_cart; git checkout develop;
cd ../logiler_bringup; git checkout develop;
cd ../logiler_description; git checkout release/0.1.0.1;
cd ../logiler_navigation; git checkout develop;
cd ../navigation; git checkout melodic-devel;
cd ../obstacle_msgs; git checkout develop;
cd ../pipeline_planner; git checkout develop;
cd ../range_sensor_layer; git checkout develop;
cd ../roboline; git checkout develop;
cd ../ros_ultrasonic_msgs; git checkout develop;
cd ../rtabmap_ros; git checkout f8d5c66;
cd ../teb_local_planner; git checkout melodic-devel;
cd ../vision_opencv; git checkout melodic;
cd ../zed-ros-wrapper; git checkout develop;

cd ../obstacle_monitor; git checkout develop;

cd ~/ros/catkin_ws
rosdep install -r --from-paths src --ignore-src # Be careful not to install libopencv*
sudo apt purge ros-melodic-libg2o
catkin_make

# udev rules and k2k setting
echo -e "\e[33m#####################################"
echo -e "\e[33m# k2k and rmc installation          #"
echo -e "\e[33m#####################################"

sudo sh -c 'echo "nvidia ALL=NOPASSWD: ALL" >> /etc/sudoers.d/logilerOverrides'
sudo usermod -aG dialout nvidia
cd ~/src
git clone https://github.com/SeaosRobotics/k2k.git
cd k2k
git checkout develop
cd service
./install.sh

# soracom and bluetooth setting
sudo apt install libbluetooth-dev

cd ~/src/
git clone https://github.com/SeaosRobotics/soracom_setup.git
cd soracom_setup
sudo ./soracom.sh
sudo sed -i -e "s/After=sys-subsystem-net-devices-%i.device/#\ After=sys-subsystem-net-devices-%i.device/g" /lib/systemd/system/ifup@.service

sudo apt-get install libical-dev
sudo apt-get install libreadline-dev
cd ~/src/
mkdir bluetooth_setting
cd bluetooth_setting
wget http://www.kernel.org/pub/linux/bluetooth/bluez-5.50.tar.xz
tar -xf bluez-5.50.tar.xz
wget https://ftp.osuosl.org/pub/blfs/conglomeration/bluez/bluez-5.50-obexd_without_systemd-1.patch
cd bluez-5.50
patch -Np1 -i ../bluez-5.50-obexd_without_systemd-1.patch
sudo apt-get install libudev-dev

./configure --prefix=/usr --mandir=/usr/share/man --sysconfdir=/etc --localstatedir=/var --enable-library
make -j4
sudo make install

if [ -e "/lib/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf" ]; then
    sudo sed -i -e "/^ExecStart=\//c\ExecStart=\/usr\/lib\/bluetooth\/bluetoothd\ --compat" /lib/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf
    sudo sed -i -e "/^ExecStart=\//a ExecStartPost=\/bin\/chmod\ 666\ \/var\/run\/sdp" /lib/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf
fi
sudo sed -i -e "/^ExecStart=\//c\ExecStart=\/usr\/lib\/bluetooth\/bluetoothd\ --compat" /lib/systemd/system/bluetooth.service
sudo sed -i -e "/^ExecStart=\//a ExecStartPost=\/bin\/chmod\ 666\ \/var\/run\/sdp" /lib/systemd/system/bluetooth.service

sudo systemctl daemon-reload
sudo systemctl restart bluetooth.service

# python_rmc installation
sudo apt install python3-pip
python3 -m pip install --upgrade pip
python3 -m pip install pycuda
cd ~/src
git clone https://github.com/SeaosRobotics/rmc-sdk.git
cd rmc-sdk/rmc-core/
python3 -m pip install -r requirements.txt
INSTALL_EGG=True python3 setup.py bdist_egg --exclude-source-files

mkdir -p ~/.rmc-sdk/keycart/res/keycart
cp ~/src/setupJetsonTX2-Xavier/rtabmap.yaml ~/.rmc-sdk/keycart/res/keycart/
cd ~/src
git clone https://github.com/SeaosRobotics/rmc-keycart.git
cd rmc-keycart/
python3 -m pip install -r requirements.txt
INSTALL_EGG=True SERVICE=TRUE USER=$USER ROBOTID=$HOSTNAME PORT=3000 RESOURCES=/home/$USER/.rmc-sdk/keycart/res/keycart python3 setup.py bdist_egg --exclude-source-files

sudo systemctl start rmc_keycart.service

echo -e "\e[33m###############################"
echo -e "\e[33m# rmc installation finished!  #"
echo -e "\e[33m###############################"

echo -e "\e[33m###########################################"
echo -e "\e[33m# All installation finished successfully  #"
echo -e "\e[33m###########################################"
