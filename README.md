# CoreSense4Home

## Installation
```bash
cd <ros2-workspace>/src
vcs import --recursive < CoreSense4Home/robocup_bringup/thirdparty.repos
sudo apt install portaudio19-dev
pip3 install -r ThirdParty/audio_common/requirements.txt
pip3 install -r ThirdParty/whisper_ros/requirements.txt

cd <ros2-workspace>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Usage
### Navigation
```bash
ros2 launch robocup_bringup navigation.launch.py
```

### Follow navigation with small objects
```bash
ros2 launch robocup_bringup navigation_follow.launch.py
```