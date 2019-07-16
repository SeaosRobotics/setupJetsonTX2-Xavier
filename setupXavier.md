# Setup for Jetson AGX Xavier

## Flush Jetpack to Xavier with SDK Manager
- Get SDK Manager on your host PC from [here](https://developer.nvidia.com/embedded/downloads).
- Flush Jetpack to Xavier. Refer to [here](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html).
- user name should be "nvidia", hostname should be "x(number)".

## Change nvpmodel to max mode
```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

## Installation SSD and Wifi card
- Follow [this instruction](https://medium.com/@ramin.nabati/installing-an-nvme-ssd-drive-on-nvidia-jetson-xavier-37183c948978).
- Change home directory path under ssd.
```bash
cd /home
cp -r nvidia/ /xavier_ssd/
sudo mv nvidia/ nvidia_bkup/
sudo ln -s /xavier_ssd/nvidia
```

## Compile and install OpenCV 3.4.6
```
mkdir ~/src
cd ~/src
git clone https://github.com/yuusuke0126-seaos/buildOpenCVXavier.git
cd buildOpenCVXavier/
git checkout 3.4.6
./buildOpenCV.sh

# After installing...
cd ~
sed -i -e "s/LD_LIBRARY_PATH=/LD_LIBRARY_PATH=/usr/local/lib:/g" .bashrc
sudo apt install libboost-all-dev libpcl-dev
```

## Installation ZED SDK
```
cd ~/src
mkdir ZED_SDK
cd ZED_SDK
wget https://www.stereolabs.com/developers/downloads/ZED_SDK_JP4.2_v2.8.1.run
chmod +x ZED_SDK_JP4.2_v2.8.1.run
./ZED_SDK_JP4.2_v2.8.1.run
```

## Compile and install gtsam, g2o, and rtabmap
```
# TODO: Confirm current x3 remote URL and branch
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
```
## ROS installation
- Follow [this instruction](http://wiki.ros.org/melodic/Installation/Ubuntu).
```
cd ~/src
git clone https://github.com/yuusuke0126-seaos/setupJetsonTX2-Xavier.git
cd setupJetsonTX2-Xavier
git checkout develop
cp .bash_ros ~/
echo "source ~/.bash_ros" >> ~/.bashrc
```
```
mkdir -p ~/ros/catkin_ws/src
cd ~/ros/catkin_ws/
catkin_make
bash
sudo apt install ros-melodic-rosserial-python
```

## Logiler pkgs installation

