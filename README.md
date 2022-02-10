mim_control
-----------

This packages contains generic controllers that can be used for a biped, quadruped, hopper and any other robot. The controllers are created based on the nature of the robot and its properties such as number of end effectors etc..

### Installation

#### Dependencies
```
- Pinocchio
- PyBullet
- Quadprog
- Matplotlib (Optional, needed to run demos)
- BulletUtils (Optional, needed to run demos)
- Robot_Properties_Solo (Optional, needed to run demos)
- Robot_Properties_Bolt (Optional, needed to run demos)
```
#### Download the package

External dependencies:
See [this tutorial](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/laas_package_from_binaries)
in order to install `eiquadprog` and `pinocchio`.
See [treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep) and [colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon) for their usage.

Local and specific dependencies and the actual repo we need to compile:
```
mkdir devel
cd devel
git clone git@github.com:machines-in-motion/treep_machines_in_motion
pip install -U treep
treep --clone MIM_CONTROL
```

#### Build the package

We use [colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)
to build this package:

```
cd devel/workspace
colcon build
```

### Usage

#### Running Demos

To run the demos, please install the following additinal packages:

```
treep --clone bullet_utils
treep --clone robot_properties_solo
treep --clone robot_properties_bolt
```

rebuild your workspace using `colcon build` and source it.

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

### Cite
```
@article{grimminger2020open,
  title={An open torque-controlled modular robot architecture for legged locomotion research},
  author={Grimminger, Felix and Meduri, Avadesh and Khadiv, Majid and Viereck, Julian and W{\"u}thrich, Manuel and Naveau, Maximilien and Berenz, Vincent and Heim, Steve and Widmaier, Felix and Flayols, Thomas and others},
  journal={IEEE Robotics and Automation Letters},
  volume={5},
  number={2},
  pages={3650--3657},
  year={2020},
  publisher={IEEE}
}
```

### License and Copyrights

BSD 3-Clause License

Copyright(c) 2019-2020 New York University, Max Planck Gesellschaft
