# Setup for Jetson AGX Xavier

## Flush Jetpack to Xavier with SDK Manager
- Get SDK Manager on your host PC from [here](https://developer.nvidia.com/embedded/downloads).
- Flush Jetpack to Xavier. Refer to [here](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html).
- user name should be "nvidia".

## Change nvpmodel to max mode
```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

## Installation SSD and Wifi card
- Follow [this instruction](https://medium.com/@ramin.nabati/installing-an-nvme-ssd-drive-on-nvidia-jetson-xavier-37183c948978).

## Install
```
mkdir ~/temp
cd ~/temp/
git clone https://github.com/yuusuke0126-seaos/setupJetsonTX2-Xavier.git
cd setupJetsonTX2-Xavier/
source install.sh
```