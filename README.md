# mini cheetah MPC + ROS

forked from https://github.com/Derek-TH-Wang/quadruped_ctrl

### MIT mini cheetah simulation in Raisim
MIT mini cheetah use customized simulator and lcm framework, which is not a popular way to do the robot development. Now, we extract the algorithm and do the simulation using ros and Raisim. This can be simple to deploy the system into different custom robot or plantform, and easy to learn the algorithm.

### System requirements:
Ubuntu 20.04, ROS Noetic

### Dependency:
use Logitech gamepad to control robot
```
git clone https://github.com/Derek-TH-Wang/gamepad_ctrl.git
```
install Raisim:
https://raisim.com/index.html

### Build
```
cd {your workspace}
catkin make
source devel/setup.bash
```

#### Install Python dependencies

```bash
pip3 install -r requirements.txt
```

### Terrain
you can modify the ```config/quadruped_ctrl_cinfig.yaml/terrain``` to deploy different terrains, there are four terrains supported in the simulator now, for example:
```
"plane"
"stairs"
"random1"
"random2"
"racetrack"
```

### Running:
run the gamepad node to control robot:
```
roslaunch gamepad_ctrl gamepad_ctrl.launch
```
run the controller in simulator:
```
roslaunch quadruped_ctrl quadruped_ctrl.launch
```

also can switch the gait type:
```
rosservice call /gait_type "cmd: 1"
```

gait type:
```
0:trot
1:bunding
2:pronking
3:random
4:standing
5:trotRunning
6:random2
7:galloping
8:pacing
9:trot (same as 0)
10:walking
11:walking2
```
