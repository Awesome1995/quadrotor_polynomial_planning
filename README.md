# README #

### splines ###
Receives a waypoint list and either generates splines or linear segments.

### Maintainer ###
* Brett Lopez (btlopez@mit.edu)
* Pete Florence (peteflo@csail.mit.edu)

### Usage ###
roslaunch splines splines.launch 

roslaunch splines splines.launch vis:=false (will not send path to rviz)

### Topics ###
If vis:=false

* Publications: /coeffs [Coeff.msg]
* Subscribtions: /waypoint_list [nav_msgs/Path]

else

* Publications: /coeffs [Coeff.msg] /splines_path [nav_msgs/Path]
* Subscribtions: /waypoint_list [nav_msgs/Path] /coeffs [Coeff.msg]