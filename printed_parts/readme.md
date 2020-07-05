This folder contains the `.STL` files for the 3D printed parts. Most of the parts are derived from the parts of the [Vorpal](https://vorpalrobotics.com/wiki/index.php/Vorpal_Robotics) hexapod; for link to original `.STL` files, see [here](https://vorpalrobotics.com/wiki/index.php/Vorpal_The_Hexapod_Assembly_Instructions).

[<img align="right" src="https://github.com/teuler/hexbotling/blob/master/pictures/base.png" alt="Drawing" width="400"/>](https://github.com/teuler/hexbotling/blob/master/pictures/base.png)
### Body
* `base_v5.stl`  
  Body of the robot. Changes from the original:
  * Improved holders for the hip servos inside the body (orange)
  * Axes to hold ball-bearings for hip servos (green)
  * Shafts (gray) for a 3rd set of servo cables, because in the 3DOF version three servos per leg are needed 
* `lid_v1.stl`  
  "lid" for body with space for battery and mounts for electronics caddy (new)
* `electronic_caddy_v2.stl`  
  Caddy for hexbot electronics board (new)

[<img align="right" src="https://github.com/teuler/hexbotling/blob/master/pictures/3dof_leg_2.png" alt="Drawing" width="400"/>](https://github.com/teuler/hexbotling/blob/master/pictures/3dof_leg_2.png) 
### Legs
* `hinge_v1.stl`, `hinge_v1_alternative.stl`   
  Leg (coxa). One coxa requires two hinges.
  * Space for ball-bearing
  * Hole to fix two hinges with an M3 screw.
  * Alternative features a smaller hole to fix horn of mini servos with M2.0 screws (e.g. [SH-0255MG](https://www.savox-shop.com/de/savoex-sh-0255mg-servo.html))
* `femur_v1.stl`  
  Leg (femur). Additional segment for 3DOF legs (new)
* `tibea_v1.stl`, `tibea_v1_alternative.stl`  
  Leg (tibea). Changes from the original:
  * Less angled than original (for 3DOF legs)
  * Axis to hold ball-bearing
  * Holes to fix the servo with screws
  * Alternative version for clones of the [MG90S mini servo](https://www.towerpro.com.tw/product/mg90s-3/)
* `foot.stl`  
  Foot, to be printed from flexibel filament (e.g. TPU)

### Other parts
* `switch_adapter.stl`  
  Adapter for power switch (original)
* `LED_holder.stl`  
  Fits a 3-mm LED into one of the holes of the base
