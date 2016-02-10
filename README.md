lfd_trick
==================
Python interface to use Baxter, MoveIt! collision checker interface for Python, pouring script, FK/IK, optimizers, etc.
lfd_trick supports ROS (Robot Operating System) architecture.

ROS:
http://wiki.ros.org/

This is an implementation of our robot pouring work.

Akihiko Yamaguchi, Christopher G. Atkeson, and Tsukasa Ogasawara: Pouring Skills with Planning and Learning Modeled from Human Demonstrations, International Journal of Humanoid Robotics, Vol.12, No.3, pp.1550030, July, 2015.

Video: https://www.youtube.com/watch?v=GjwfbOur3CQ


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Acknowledgment
==================
TPR2Mannequin class (mannequin controller for PR2) in src/base/ros_rbt_pr2.py is based on a code implemented by Scott Niekum, originally it was stored in pr2_lfd_utils/src/recordInteraction.py.

CMA-ES (src/cma.py) is implemented by Nikolaus Hansen.  Read src/CMA1.0.09-README.txt for more information.


Requirements
==================
Install following packages before building lfd_trick.

Binaries are available (you can use **apt-get**):
- ROS core system, rospy, roscpp, std_msgs, std_srvs, geometry_msgs, tf, ...
- Python: core, numpy
- ros-ROS_DISTR-moveit-full  (ROS_DISTR: groovy, hydro, indigo, etc.)
- ros-ROS_DISTR-moveit-resources

Need to build from source (NOTE: read **Requirements** and **Build** of each package):
- lfd_vision: https://github.com/akihikoy/lfd_vision
- lfd_sim: https://github.com/akihikoy/lfd_sim

Working with Baxter/PR2 (OPTIONAL):
- Need some packages for Baxter/PR2
- Information would be found in: http://akihikoy.net/notes/?text


Build
==================
The repository directory should be in ROS workspace (e.g. ~/ros_ws/).
Build lfd_trick with rosmake.

```
$ rosmake lfd_trick
```

After rosmake, you will find some executables in bin/ directory.
There are some build directories made by ROS.


cui_tool
==================
The core program is `scripts/cui_tool.py`.  Execute it from the root directory of lfd_trick.

```
$ scripts/cui_tool.py
```

Note: roscore should be working beforehand.
This program provides a CUI interface to execute sub scripts stored in `scripts/motions/`.  This is useful since we can execute many programs without ROS reconnection to robots.

If robotic environment is not setup, it will just display:

```
NoRobot:trick or quit|L>
```

Type `test.test` and press the enter:

```
NoRobot:trick or quit|L> test.test
<core_tool.TCoreTool object at 0x4f37850>
Hello world!
Test.
  Usage: test [FLOAT_VALUE]
```

It executed `scripts/motions/test/test.py` (`test.` specifies a sub directory).

If a robot is ready (it also works with simulated robots), we can run robot control scripts.  For example,

```
Baxter:trick or quit|L> keyctrl
Baxter:trick or quit|L> test.throw1
```

are a keyboard teleoperation program and a throwing a paper-plane program.  test.throw1 works on Baxter only.

Use `-help` option to see a brief explanation of each script; e.g.

```
NoRobot:trick or quit|L> test.test -help
Test.
  Usage: test [FLOAT_VALUE]
```

The script `pour` is the robot pouring.


Other tools
==================
Some launch files are provided to execute PR2 and Baxter robots.  The state validity checker is also launched.


Troubles
==================
Send e-mails to the author.  I accept only from my students.
