# CoreSense4Home

CoreSense4Home is the CoreSense social testbed developed by the Gentlebots RoboCup@Home team. It combines perception, dialogue, navigation and manipulation through ROS 2 lifecycle components and behaviour trees.

## CoreSense role

The terms below follow the [CoreSense Ontology (CSO)](https://w3id.org/coresense/cso).

- The robot is an [Agent](https://w3id.org/coresense/cso#Agent) that executes or commands actions.
- Perception, dialogue and task control are [Cognitive Functions](https://w3id.org/coresense/cso#CognitiveFunction): they process information used by the robot.
- These functions realise [Cognitive Capabilities](https://w3id.org/coresense/cso#CognitiveCapability), including detecting people and objects, interacting with people, navigating and manipulating objects.
- A RoboCup scenario is a [Task](https://w3id.org/coresense/cso#Task). Its behaviour tree organises planned [Actions](https://w3id.org/coresense/cso#Action) that contribute to a [Goal](https://w3id.org/coresense/cso#Goal), such as welcoming a guest or delivering luggage.

## System flow

~~~mermaid
flowchart LR
    environment["People and environment"] --> perception["Perception and dialogue"]
    perception --> state["Task context and robot knowledge"]
    state --> bt["Behaviour-tree task control"]
    bt --> actions["Navigation, speech and manipulation actions"]
    actions --> environment
~~~

The main ROS 2 packages are `perception`, `hri`, `configuration`, `motion`, `arm`, `bt_test` and `robocup_bringup`.

## Requirements

Use Ubuntu 22.04 and ROS 2 Humble. The full testbed is developed for the TIAGo robot; individual packages can also be built and tested separately. Follow the [CoreSense4Home software setup](https://github.com/CoreSenseEU/CoreSense4Home/wiki/C-Software-Setup) for the robot-specific dependencies.

## Build

~~~bash
mkdir -p ~/robocup24_ws/src
cd ~/robocup24_ws/src
git clone https://github.com/CoreSenseEU/CoreSense4Home.git
vcs import --recursive < CoreSense4Home/robocup_bringup/thirdparty.repos
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
~~~

## Run

Carry My Luggage:

~~~bash
ros2 launch robocup_bringup carry_my_luggage_dependencies.launch.py
ros2 launch robocup_bringup carry_my_luggage.launch.py
~~~

Receptionist:

~~~bash
ros2 launch robocup_bringup receptionist_dependencies.launch.py
ros2 launch robocup_bringup receptionist.launch.py
~~~

The dependencies and task launch files should be started in separate terminals after sourcing the workspace in each terminal.

The [CoreSense4Home wiki](https://github.com/CoreSenseEU/CoreSense4Home/wiki) describes the original system setup, architecture and behaviour-tree tasks. General project documentation is available on the [CoreSense technical site](https://coresenseeu.github.io/).
