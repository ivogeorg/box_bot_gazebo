### `box_bot_gazebo`

#### `box_bot_description`

The repository [`box_bot_description`](https://github.com/ivogeorg/box_bot_description) contains the robot model to populate in Gazebo. 

#### Moments of inertia

##### Formulae for basic solids
```
# "box_inertia"
# params="mass x y z"
# mass: Mass in Kilograms of the link
# x,y,z: Are the size of each side of the box in each of the axes in meters

<inertia  ixx="${mass*(y*y+z*z)/12}" ixy = "0" ixz = "0"
        iyy="${mass*(x*x+z*z)/12}" iyz = "0"
        izz="${mass*(x*x+z*z)/12}"
/>

# "cylinder_inertia" 
# params="mass r l"
# mass: Mass in Kilograms of the link
# r: Radius of the cylinder in meters
# l: Height of the cylinder in meters
# Note: l is parallel to z axis

  <inertia  ixx="${mass*(3*r*r+l*l)/12}" ixy = "0" ixz = "0"
            iyy="${mass*(3*r*r+l*l)/12}" iyz = "0"
            izz="${mass*(r*r)/2}" />

# sphere_inertia"
# params="mass r"
# mass: Mass in Kilograms of the link
# r: Radius of the sphere in meters
<inertia  
      ixx="${2*mass*r*r/5}" ixy = "0" ixz = "0"
      iyy="${2*mass*r*r/5}" iyz = "0"
      izz="${2*mass*r*r/5}"
/>
```
##### Calculator

`source /home/simulations/ros2_sims_ws/install/setup.bash`
`ros2 run spawn_robot_tools_pkg inertia_wizzard`

#### Basic physical properties

| Symbol | Definition | Explanation
| --- | --- | --- |
| **mu1** | The static friction coefficient. | It is how much friction there is until the object starts moving in simple terms.|
| **mu2** | The dynamic friction coefficient. | It is how much friction there is when the object moves in simple terms. |

These values are calculated through friction tests with elements with the same mass as the links you set these values to. Of course, you should also remember the materials they are made of and so on. However, in reality, it sets them with the values that make the robot behave correctly, not necessarily the real ones.

| Symbol | Definition | Explanation
| --- | --- | --- |
| **kp** | This coefficient sets the static contact stiffness. | This determines whether the linked material is closer to marble (rigid, bigger values) or more like rubber (soft material, lower values). |
| **kd** | This coefficient sets the dynamic contact stiffness. | This determines whether the linked material is closer to marble (rigid, bigger values) or more like rubber (soft material, lower values). It is essentially how much it deforms over a long period, exerting its pressure. |

##### Only for elements that touch the ground

```
  <gazebo reference="left_wheel">
    <kp>1000000000000000000000000000.0</kp>
    <kd>1000000000000000000000000000.0</kd>
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <kp>1000000000000000000000000000.0</kp>
    <kd>1000000000000000000000000000.0</kd>
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="front_pitch_link">
    <kp>1000000000000000000000000000.0</kp>
    <kd>1000000000000000000000000000.0</kd>
    <mu1>0.5</mu1>
    <mu2>0.5</mu2>
    <material>Gazebo/Purple</material>
  </gazebo>

  <gazebo reference="back_pitch_link">
    <kp>1000000000000000000000000000.0</kp>
    <kd>1000000000000000000000000000.0</kd>
    <mu1>0.5</mu1>
    <mu2>0.5</mu2>
    <material>Gazebo/Yellow</material>
  </gazebo>
```

#### Launching Gazebo world and spawning Box Bot

1. In terminal, launch the empty world in Gazebo:
   1. `cd ~/ros2_ws`
   2. `colcon build`
   3. `source install/setup.bash`
   4. `ros2 launch box_bot_gazebo start_world.launch.py
2. In another terminal, spawn the robot and publish it's static transforms (Rviz also started but without the [`joint_state_publisher_gui`](https://github.com/ivogeorg/box_bot_description/blob/main/README.md#starting-rviz2-and-joint_state_publisher_gui), all links but the `base_link` and `chassis` are missing):
   1. `cd ~/ros2_ws`
   2. _(optional)_ `colcon build`
   3. `source install/setup.bash`
   4. `ros2 launch box_bot_gazebo spawn_robot_ros2_physical.launch.xml`  

![Box Bot](assets/box_bot.png)  

#### Publishing joint states

##### Launching a joint state publisher node

For example, see previous heading.

##### Adding a Gazebo ROS joint state publisher plugin

```
<gazebo>
    <plugin name="box_bot_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
        <remapping>~/out:=joint_states</remapping>
    </ros>
    <update_rate>30</update_rate>
    <joint_name>joint_left_wheel</joint_name>
    <joint_name>joint_right_wheel</joint_name>

    <joint_name>front_yaw_joint</joint_name>
    <joint_name>back_yaw_joint</joint_name>
    <joint_name>front_roll_joint</joint_name>
    <joint_name>back_roll_joint</joint_name>
    <joint_name>front_pitch_joint</joint_name>
    <joint_name>back_pitch_joint</joint_name>

    </plugin>
</gazebo>
```

##### Topic `/joint_states`

Now Rviz2 shows all the links of the robot:  
![Box bot with joint states](assets/box_bot_with_joint_states.png)  

#### Moving the robot

1. Add joint state publisher and differential drive plugins.
2. Launch Gazebo and spawn the robot with /robot_description and /joint_states topics.
   1. Start Gazebo with empty world  
      `cd ~/ros2_ws`  
      `colcon build`  
      `source install/setup.bash`  
      `ros2 launch box_bot_gazebo start_world.launch.py`  
   2. Spawn the robot and publish to topics  
      `cd ~/ros2_ws`  
      `source install/setup.bash`  
      `ros2 launch box_bot_gazebo spawn_robot_ros2_control.launch.xml`       
3. Teleoperate with keyboard.
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

![Box bot moving in Rviz2](assets/box_bot_moving_in_rviz2.png)  
![Box bot moving in Gazebo](assets/box_bot_moving_in_gazebo.png)  

#### ROS2 Control in Gazebo

[`ros2_control`](https://control.ros.org/rolling/index.html) is a powerful package for robot control in ROS 2.

A small example of using it is shown with a scanner which can be moving "in and out of" the robot body.  

![Box bot with scanner (Rviz2)](assets/box_bot_scanner_in_rviz2.png)  

![Box bot with scanner (Gazebo)](assets/box_bot_scanner_in_gazebo.png)  
