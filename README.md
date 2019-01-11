# ackermann

View Ackermann model in Rviz
```bash
  roslaunch ackermann_description ackermann_rviz.launch
```
it will diplay also all the controllers present in the model.

# gazebo simulation

Launch the robot in the origin of an empty world, together with the controllers
```bash
  roslaunch ackermann_description ackermann_gazebo.launch
```
# Move the robot

After changed the desired set-point for the robot inside the code in
```bash
  ackermann/ackermann_control/src/controller_node.cpp
```
to move the robot at the final position, simply launch the controller node
```bash
  roslaunch ackermann ackermann_control ackermann_control.launch
```
