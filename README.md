# PyBullet for Digit and Reinforcement Learning

PyBullet Simulation Environment for the bipedal robot "Digit"

Chankyo Kim

RoahmLab (http://www.roahmlab.com/)

University of Michigan



## Robot Model
A lot of robotic dynamics softwares (such as Matlab robotics toolbox, Pinocchio, and PyBullet) are sensitive to the order of joints & links in URDF. If you want to create URDF for a branched robot, you should CROSS-CHECK (e.g. Inertia Matrix) it among Matlab robotics toolbox, PyBullet, and Roy Featherstone spatial dynamics library.

### Directory '/urdf/' and '/xml'
Robot described in URDF and XML format each.

### Directory '/'
1. digit-xml.py

* Load Digit of XML in PyBullet
* Assign jointName
* Provide Closed Loop Constraints
* Fix Base
* Set Camera Position - Allow Keyboard/Mouse Events

2. python-viewer-mujoco.py

* Simple code to check modified Digit XML file (e.g. modifing joint position)

3. digit-urdf.py

* Load Digit of URDF in PyBullet
  
4. other miscellaneous codes

* Useful functions for PyBullet included
