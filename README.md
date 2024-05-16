# CoreSense4Home

## Installation

See [Software setup](https://github.com/CoreSenseEU/CoreSense4Home/wiki/C-Software-Setup)


## Usage 
### Navigation
```bash
ros2 launch robocup_bringup navigation.launch.py
```
### Launch current carry my luggage implementation

First kill move_group node inside tiago robot. Then in separate terminals launch:

```bash
ros2 launch robocup_bringup navigation_follow.launch.py rviz:=True
```
```bash
ros2 launch attention_system attention.launch.py
```
```bash
ros2 launch robocup_bringup carry_my_luggage_dependencies.launch.py
``` 
```bash
ros2 launch whisper_bringup whisper.launch.py
```
Finally:

```bash
ros2 run bt_test carry_my_luggage_test
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

### Demo Dialog
```bash
ros2 launch robocup_bringup dialog.launch.py
```

Execute the test:

```bash
ros2 run bt_test ask_test
```
