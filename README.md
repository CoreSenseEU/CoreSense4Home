# CoreSense4Home

## Installation

1. Create a dedicated workspace and clone this repo in it

```
mkdir -p robocup/src
cd robocup/src
git clone https://github.com/CoreSenseEU/CoreSense4Home.git
```

2. Install [Whisper_ROS](https://github.com/mgonzs13/whisper_ros)

   1. [Install CUDA (V12.1)](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#ubuntu)
   2. Follow instructions on [whisper_ros repo](https://github.com/mgonzs13/whisper_ros)

3. Install internal dependencies

```
robocup/src $ vcs import --recursive < CoreSense4Home/robocup_bringup/thirdparty.repos
robocup/src $ cd ..
robocup $ rosdep install --from-paths src --ignore-src -r -y
```

## Usage 

### Launch current carry my luggage implementation

First kill navigation, localization and hri_body_detect (you are going to launch your own) modules inside tiago robot. Then in separate terminals launch:

```bash
ros2 launch robocup_bringup carry_my_luggage_dependencies.launch.py
``` 
Finally:

```bash
ros2 launch robocup_bringup bt_test carry_my_luggage.launch.py
```
