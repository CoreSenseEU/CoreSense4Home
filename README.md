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
### Demo moveit
inside tiago, first kill move_group and then:
```bash
ros2 launch tiago_moveit_config move_group.launch.py
```
Launch the percetion system with the remaps for the tiago, and activate the object detection node
```bash
ros2 launch perception_system perception3d.launch.py
```
launch the speaking system:
```bash
ros2 run audio_common tts_node
ros2 run audio_common audio_player_node
ros2 launch whisper_bringup whisper.launch.py
```
launch the manipulation system:
```bash
ros2 launch action_server server.launch.py
```

execute the test:

```bash
ros2 run bt_test pick_demo_test
```

