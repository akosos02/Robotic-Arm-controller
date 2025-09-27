# 2-Link Planar Robotic Arm Control

This project provides a complete Python framework for controlling a **2-link planar robotic arm** with an additional linear actuator (x-axis). It includes kinematics calculations, motion planning, servo control, and pick-and-place functionality using a vacuum gripper. Optional visualization tools are included for debugging and testing in simulation mode.

---

## Features

### 1. Kinematics
- **Forward Kinematics (`directGeometryCalculation`)**  
  Computes the end-effector position in 3D space given the joint angles and link lengths. Supports the YZ plane with a linear x-axis motion.  

- **Inverse Kinematics (`inversGeometryCalculation`)**  
  Calculates the joint angles needed to reach a specific end-effector position using the **circle intersection method**. Handles multiple intersection points and chooses the solution minimizing joint movement.

### 2. Motion Planning
- **Linear Cartesian Motion (`GetArmLinearPositions`)**  
  Generates intermediate positions along a straight line between a start and end position to allow smooth motion.

- **Joint-Space Motion (`MoveArmToPositionGraduallyJoint`)**  
  Interpolates between starting and target joint angles to move the arm gradually. Supports synchronized or independent joint movement.

- **Movement Execution (`movealongpos` and `movealongangle`)**  
  Functions to execute smooth linear or joint-space movements by sending angles to the servo motors.

### 3. Servo Control
- **Angle Conversion (`convertToServoAngles`)**  
  Converts calculated joint angles to physical servo angles, applies limits, and safely sets the servo positions.  
- Prevents servo overheating by setting angles to `None` when idle.

### 4. Pick-and-Place Operations
- **Pick Object (`moveArmToPickObject`)**  
  Moves the arm to pick an object detected by sensors, using both joint-space and linear motion. Controls vacuum gripper for safe object pickup.

- **Drop Object (`moveArmToDropObject`)**  
  Moves the arm to drop an object at a predefined location while avoiding collisions, controlling vacuum release and smooth motion.

### 5. Visualization (Test Mode)
- If `test = True`, the code plots:
  - Linkage circles for inverse kinematics
  - Intersection points
  - Arm directions for q0 and q1  
- Useful for debugging and verifying kinematics solutions without physical hardware.

---

## Global Variables
- `LastAngleq0`, `LastAngleq1` — Stores last used joint angles to minimize movement.
- `xposition` — Stores linear actuator position.
- `linkLength` — Stores robot link lengths.
- `p1` — Stores the chosen intersection point during inverse kinematics.

---

## Dependencies
- `numpy` — For numerical calculations.
- `matplotlib` — For visualization (only in test mode).
- `math` — For trigonometric calculations.
- `time` — For delays between servo movements.

---

## Usage

1. **Initialization**  
   Set up servo objects (`servo1`, `servo2`, `servo3`) and vacuum (`Vaccum`) in a `Setup.py` file.  

2. **Move Arm**  
   - Use `movealongpos` to move along a Cartesian path.  
   - Use `movealongangle` to move along a joint-space path.  

3. **Pick-and-Place**  
   - Detect objects and choose an index.  
   - Call `moveArmToPickObject(detectionobjects, chosen_idx)` to pick the object.  
   - Call `moveArmToDropObject()` to drop the object safely.  

4. **Test Mode Visualization**  
   - Set `test = True` to see plots of arm geometry and intersection points.  

---

## Example

```python
from Setup import servo1, servo2, servo3, Vaccum
from ArmControl import moveArmToPickObject, moveArmToDropObject, linkLength

# Pick the first detected object
moveArmToPickObject(detectionobjects, 0)

# Drop the object at the predefined location
moveArmToDropObject()
```
<p align="center">
  <img src="https://github.com/user-attachments/assets/b5715d79-9ba2-40af-a2ca-0f5d0784a35e">
</p>
<p align="center">
  Real-life example of my code operating on the robotic arm
</p>
