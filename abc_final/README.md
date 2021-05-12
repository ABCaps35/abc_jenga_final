# ABC Final Individual Project


## Launching the Environment
To launch this project through simulation, simply run the following launch file using this command:
```
roslaunch abc_final abc_final_full.launch
```

Alternatively, you can run each element of this project separately using these commands:
```
roslaunch abc_final abc_final_sim.launch

roslaunch exmpl_models add_table_and_block.launch

roslaunch abc_final abc_final_transforms.launch

roslaunch abc_final abc_final_services.launch

rostopic pub robot/set_super_enable std_msgs/Bool 1
```
The last command on this list is essential for enabling the robot to take instructions and move. Without this command, the robot will simply not move.

*NOTE: if the simulation starts and the block falls off of the table, right-click it, delete it, and replace it with this command:*
```
roslaunch exmpl_models add_table_and_block.launch
```



## Executables
To execute the functionality of this package, run the following commands:
```
rosrun abc_final abc_coordinator
```
