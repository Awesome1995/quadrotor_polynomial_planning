# README #

### splines ###
Receives a waypoint list and either generates splines or linear segments. A master script will evaluate the polynomial and send desired state commands to the vehicle.

### Maintainer ###
* Brett Lopez (btlopez@mit.edu)

### Usage ###
* For polynomial generation:
```
 roslaunch splines linear.launch 
```
```
roslaunch splines linear.launch vis:=false (will not send path to rviz)
```


* For sending desired state commands:
```
 roslaunch splines gandalf.launch v_max:=(desired max velocity) a_max:=(peak acceleration)
```
The larger the acceleration the larger the attitude commands will be. gandalf.launch will also launch the joystick driver.

### Topics ###
linear.launch:

* Publications: /coeffs [Coeff.msg], /splines_path [nav_msgs/Path]
* Subscribtions: /waypoint_list [nav_msgs/Path], /coeffs [Coeff.msg]

gandalf.launch:

* Publications: /odom [Float64.msg], /goal [acl_fw/QuadGoal.msg]
* Subscribtions: /joy [acl_fsw/JoyDef.msg], /coeffs [Coeff.msg]