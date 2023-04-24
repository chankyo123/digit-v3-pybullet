# PyBullet for Digit and Reinforcement Learning

## Directory ./urdf/
Digit is described in URDF format, origial repo from https://github.com/adubredu/DigitRobot.jl/tree/main/urdf

## Directory ./xml/
Digit is described in XML format that ball joints are changed to hinge joints since PyBullet don't provide ball joint

## Directory ./
1. digit-xml.py

* Load Digit of XML in PyBullet
* Assign jointName
* Provide Closed Loop Constraints
* Fix Base
* Set Camera Position - Allow Keyboard/Mouse Events

2. python-viewer-mujoco.py

Simple code to check modified Digit XML file (e.g. modifing joint position)

3. digit-urdf.py

Load Digit of URDF in PyBullet
  
4. other miscellaneous codes

Useful functions for PyBullet included