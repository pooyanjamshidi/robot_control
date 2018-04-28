# Functionality
This repository provide robot controllers which interface gazebo simulator and execute missions using robot in different environmental conditions. Also, it provides utilities to perturb the environment, e.g., place obstacle in the world, set charge for the robot battery.  

# Dependency

```bash
https://github.com/cmu-mars/brass_gazebo_config_manager
https://github.com/cmu-mars/brass_gazebo_battery
https://github.com/cmu-mars/cp1_base
```

# Install

This package uses cp1 battery plugins services, the gazebo packages should be installed:

```bash
git clone https://github.com/cmu-mars/cp1_controllers.git
cd cp1_controllers
make
```

# Usage

After running `roscore` service and launching the robot `roslaunch launch/cp1-base-test.launch`, we can use the `cli` as follows:

```bash
python cli.py execute_task_reactive l2 l3 l4 l5
python cli.py execute_task l2 l3 l4 l5
python cli.py place_obstacle -19.08 11.08
python src/cli.py set_charge 32560.0
python src/cli.py go_directly l1 l2
python src/cli.py remove_obstacle Obstacle_0
```

