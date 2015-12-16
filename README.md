# README #

### splines ###
Receives a waypoint list and either generates splines or linear segments.

### Maintainer ###
* Brett Lopez (btlopez@mit.edu)

### Usage ###
roslaunch splines splines.launch 
roslaunch splines splines.launch vis:=false (will not send path to rviz)

### Topics ###
Publications: /coeffs [Coeff.msg]
Subscribtions: /waypoint_list [nav_msgs/Path]