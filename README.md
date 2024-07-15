### box_bot_gazebo

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

