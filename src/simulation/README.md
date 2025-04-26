#### The instruction for running the simulation

> run the following command is every terminal you open or you can put it in `~/.bashrc` to be activated automatically.
```shell
$ source Desktop/Shiva/catkin_workspace/devel/setup.bash
```

### steps for running 
> Open the each step in different terminals
> 1. run `roscore` \
>   In the linux i have, just open the `ros_core_terminal`.
>
> 2. ```shell
>     $ roslaunch ur5_gripper_moveit_config demo_gazebo.launch
>    ```
> 3. ```shell
>     $ rosrun ur5_gripper_moveit_config robot_controller.py
>       or you can run the code in the VS code
>    ```



## Modify the World

### change the position of the camera

> Go to this path:
> `Desktop/Shiva/catkin_workspace/src/common-sensors/common_sensors/urdf/sensors/kinect.urdf.xacro`
> open the file and change the variables for `cam_px`, `cam_py`, ...

### add objects and save the world
> you can add objects by the `insert` tab or creat new ones by clicking on cube or other shapes in demo_gazebo
> You need to just run `roslaunch ur5_gripper_moveit_config demo_gazebo.launch` and change the environment in gazebo.
> then save the world.
>
> **add the world to the launch file**  :
>> 1. have the world file in `catkin_workspace/src/ur5_simple_pick_and_place/world`  `my_world`
>> 2. change the world name in `gazebo.launch` in `catkin_workspace/src/ur5_gripper_moveit_config/launch` in lines **6** and **10**.

## What it can do now:
> 1. move the robot to pre-defined positions or give the position in `[x, y, z]` format.
> 2. take images by opencv
> 3. goes to the position by giving `x, y, z`
> 4. picks and releases objects 


## What it is supposed to do
> 1. Take image
> 2. apply ai for obtaining the position of the asked object by the user.
> 3. Then, a position in format of [x, y, z] is obtained.
> 4. go and pick it

## To DO 
> 1. How to defined a new pre-defined pose? (I need a better home pose facing the table)  **DONE**
>
> 2. Check if robot can pick the object.   **DONE**\
finding the center pose of the object and convert it to world frame:\
        1. find the min and max HSV color values of the top of the object -> hsv_from_mouse\
        2. draw a bounding box only around that -> stream_top_position\
> 2.1. how to find out the objects' ids **DONE** you define it in the code
> 3. add new objects to the wold **DONE** 
> 4. imporve the world (Not very necessary but better to do) 
> 5. speed up the movement of the robot