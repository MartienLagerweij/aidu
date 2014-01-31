Automated Indoor Delivery Unit (AIDU)
====

This is the software for the robot AIDU, the delivery robot for offices. It is built on top of ROS, with additional dependencies on arduino, threemxl, sklearn, opencv, numpy and scipy.

The software is developed into several packages, some of which are interdependent.

In order to compile the software use:

`git clone https://github.com/rjagerman/aidu aidu`
`rosmake aidu`

To run it use:

`roscd aidu`
`roslaunch aidu.launch`

