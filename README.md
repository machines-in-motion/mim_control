# BLMC Controllers

## What it is

This packages contains generic controllers that can be used for a biped, quadruped, hopper and any other robot. The controllers are created based on the nature of the robot and its properties such as number of end effectors etc..

## Dependencies
```
- Pinocchio
- PyBullet
- Quadprog
- Matplotlib (Optional, needed to run demos)
- Robot_Properties_Solo (Optional, needed to run demos)
- Robot_Properties_Bolt (Optional, needed to run demos)
```
## Installation
```
git clone git@github.com:avadesh02/blmc_controllers.git
cd blmc_controllers
pip3 install .
```

## Running Demos
To run the impedance controller on Solo12 follow the below mentioned steps 
```
source /opt/openrobots/setup.bash (source open robots)
cd demo
python3 demo_robot_impedance.py
```

To run the Center of Mass controller on Solo12 follow the below mentioned steps
```
source /opt/openrobots/setup.bash (source open robots)
cd demo
python3 demo_robot_com_ctrl.py
```

## Authors
- Avadesh Meduri
- Paarth Shaah 
- Julian Viereck

## Cite
```
Grimminger, Felix, Avadesh Meduri, Majid Khadiv, Julian Viereck, Manuel Wüthrich, Maximilien Naveau, Vincent Berenz et al. "An open torque-controlled modular robot architecture for legged locomotion research." IEEE Robotics and Automation Letters 5, no. 2 (2020): 3650-3657.
```

## Copyrights

Copyright(c) 2019-2020 New York University, Max Planck Gesellschaft

## License

BSD 3-Clause License


