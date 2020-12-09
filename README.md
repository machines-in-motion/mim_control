# BLMC_controllers

## What it is

This packages contains all controllers for the quadruped, hopper and anyother BLMC robot that is fabricated in the future

## Dependencies
```
- Pinocchio
- PyBullet
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
python3 demo_solo_impedance.py
```

To run the impedance controller on Bolt follow the below mentioned steps
```
source /opt/openrobots/setup.bash (source open robots)
cd demo
python3 demo_bolt_impedance.py
```

## Authors
- Avadesh Meduri
- Paarth Shaah 
- Julian Viereck

## Copyrights

Copyright(c) 2019-2020 New York University, Max Planck Gesellschaft

## License

BSD 3-Clause License


