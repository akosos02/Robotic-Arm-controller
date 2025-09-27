import numpy as np
import matplotlib.pyplot as plt
from math import atan2
from time import sleep

if 'test' not in globals():
    test = False
elif test: 
    fig, ax = plt.subplots()
    print("Running Setup.py at test mode")
    

global LastAngleq0
global LastAngleq1
LastAngleq0 = np.radians(90)  #: Initial angle for servo1
LastAngleq1 = np.radians(-40)  #: Initial angle for servo2


global xposition
xposition = 0  #: Global variable to store the x position of the end effector


global p1
p1 = [0, 0]  #: Global variable to store the first intersection point


def translation(i: str, d: float):
    #: Create a 4x4 identity matrix
    a = np.eye(4)
    #: Fill in the translation matrix elements based on the input direction
    if i == "x":
        a[0, 3] = d
    elif i == "y":
        a[1, 3] = d
    elif i == "z":
        a[2, 3] = d
    else:
        #: If the input is invalid, print an error message and return an identity matrix
        print("Error in translation")
    return a


#: Homogeneous transformation matrices for rotation
def rotation(i: str, s: float):
    #: Convert degrees to radians and negate for right-hand rule
    s = s
    if i == "x":
        #: Create a 4x4 identity matrix
        a = np.eye(4)
        #: Fill in the rotation matrix elements
        a[1, 1] = np.cos(s)
        a[1, 2] = -np.sin(s)
        a[2, 1] = np.sin(s)
        a[2, 2] = np.cos(s)
        return a
    if i == "y":
        #: Create a 4x4 identity matrix
        a = np.eye(4)
        #: Fill in the rotation matrix elements
        a[0, 0] = np.cos(s)
        a[0, 2] = np.sin(s)
        a[2, 0] = -np.sin(s)
        a[2, 2] = np.cos(s)
        return a
    if i == "z":
        #: Create a 4x4 identity matrix
        a = np.eye(4)
        #: Fill in the rotation matrix elements
        a[0, 0] = np.cos(s)
        a[0, 1] = -np.sin(s)
        a[1, 0] = np.sin(s)
        a[1, 1] = np.cos(s)
        return a
    #: If the input is invalid, print an error message and return an identity matrix
    print("error")
    return np.eye(4)


#: Direct geometry calculation for a planar 2-link arm in the YZ plane
def directGeometryCalculation(a, q):
    #: Create a homogeneous transformation matrix for the robot arm
    #: based on the link lengths (a) and joint angles (q)
    #: The transformation matrix is constructed by chaining translations and rotations
    #: around the z-axis and x-axis, following the Denavit-Hartenberg convention
    transform = (
        rotation("z", 0)
        @ translation("z", 0)
        @ translation("x", a[0])
        @ rotation("x", q[0])
        @ rotation("z", 0)
        @ translation("z", a[1])
        @ translation("x", 0)
        @ rotation("x", q[1] - q[0])
        @ rotation("z", 0)
        @ translation("z", a[2])
        @ translation("x", 0)
        @ rotation("x", 0)
    )
    t = transform[:3, 3]
    t = np.array([xposition, t[2],-t[1]])  #: Extract the translation part of the transformation matrix
    return t


#: Inverse geometry calculation for a planar 2-link arm in the YZ plane
def inversGeometryCalculation(a, xv):
    #: Calculate the angles for the robot arm based on the end-effector position (xv) and the link lengths (a)
    #: using the intersection of circles method
    #: Get the values from the input
    x0, y0, z0 = [0, 0, 0]
    x2, y2, z2 = xv
    #: Get the intersection points of the circles
    p = circle_intersections(0, 0, a[1], xv[1], xv[2], a[2])
    #: only take the first intersection point
    global LastAngleq0, LastAngleq1
    if not p:
        print("Error: No intersection points found, possition is not reachable")
        return [LastAngleq0, LastAngleq1, 0]

    #: Calculate tha angles based on the intersection point from the horizontal line
    last_q0 = LastAngleq0
    q0horizontal = angle_from_horizontal(y0, z0, p[0][0], p[0][1])
    q0horizontal2 = angle_from_horizontal(y0, z0, p[1][0], p[1][1])
    global p1
    
    #: Choose the intersection point that is closest to the last q0 angle for minimal movement
    p1 = p[0] if abs(q0horizontal - last_q0) < abs(q0horizontal2 - last_q0) else p[1]
    p2 = p[1] if p1 is p[0] else p[0]
    #: Calculate the robot angles
    q0 = angle_from_horizontal(y0, z0, p1[0], p1[1], degrees=False)
    q1 = angle_from_horizontal(p1[0], p1[1], y2, z2, degrees=False)
    global xposition
    q2 = (xposition) / ((60)/np.radians(270)) 
    #: Return the angles
    if test:
        if q0 is not None and q1 is not None:
            print(f"q0: {np.rad2deg(q0):.2f}° q1: {np.rad2deg(q1):.2f}°")
        #: Plot the circles and points for visualization if in test mode
        plotCirclesPoints(y0, z0, a[1], y2, z2, a[2], p[0], p[1], q0, q1, a)
    return [q0, q1 ,q2]


def angle_from_horizontal(x0, y0, x1, y1, *, degrees=False):
    #: Find the distance between the points
    dx = x1 - x0
    dy = y1 - y0
    #: Calculate the angle in radians
    theta = atan2(dy, dx)
    #: Convert to degrees if requested
    return np.degrees(theta) if degrees else theta


def circle_intersections(x0, y0, r0, x1, y1, r1):
    #: Find the distance between the centers of the circles
    dx = x1 - x0
    dy = y1 - y0
    d = np.hypot(dx, dy)
    #: Check if solution exist
    if d > r0 + r1:
        return None  #: Circles are too far apart
    if d < abs(r0 - r1):
        return None  #: One circle is inside the other
    if d == 0 and r0 == r1:
        return None  #: Infinite number of intersection points
    #: Compute a
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    #: Compute point P2
    x2 = x0 + a * dx / d
    y2 = y0 + a * dy / d
    #: Compute h
    h = np.sqrt(r0**2 - a**2)
    #: Compute the offset (perpendicular vector)
    rx = -dy * (h / d)
    ry = dx * (h / d)
    #: Compute intersection points
    xi1 = x2 + rx
    yi1 = y2 + ry
    xi2 = x2 - rx
    yi2 = y2 - ry
    return (xi1, yi1), (xi2, yi2)

def plotCirclesPoints(y0, z0, r1, y2, z2, r2, intersection1, intersection2, q0, q1, a):
    circle1 = plt.Circle(
        (y0, z0), r1, color="blue", fill=False, linestyle="--", label="Circle 1"
    )
    ax.add_artist(circle1)
    #: Plot the second circle
    circle2 = plt.Circle(
        (y2, z2), r2, color="red", fill=False, linestyle="--", label="Circle 2"
    )
    ax.add_artist(circle2)
    #: Plot the initial point (y0, z0)
    ax.plot(y0, z0, "ro", label="Initial Position (y0, z0)")
    #: Plot the desired point (y2, z2)
    ax.plot(y2, z2, "go", label="Desired Position (y2, z2)")
    #: Plot the intersection points
    ax.plot(intersection1[0], intersection1[1], "bo", label="Intersection 1")
    ax.plot(intersection2[0], intersection2[1], "bo", label="Intersection 2")
    #: Plot the line in the direction of q0
    #: q0 direction: (sin(q0), cos(q0)) represents the unit vector direction
    q0_direction = np.array(
        [a[1] * np.cos(q0), a[1] * np.sin(q0)]
    )  #: Direction vector for q0

    ax.quiver(
        y0,
        z0,
        q0_direction[0],
        q0_direction[1],
        angles="xy",
        scale_units="xy",
        scale=1,
        color="green",
        label=f"q0 direction",
    )
    global p1
    intersection = p1 if p1 is not None else intersection1

    #: Plot the line in the direction of q1 (from intersection)

    q1_direction = np.array(
        [a[2] * np.cos(q1), a[2] * np.sin(q1)]
    )  #: Direction vector for q1

    ax.quiver(
        intersection[0],
        intersection[1],
        q1_direction[0],
        q1_direction[1],
        angles="xy",
        scale_units="xy",
        scale=1,
        color="orange",
        label=f"q1 direction",
    )

    #: Set axis limits for better visibility
    ax.set_xlim(-r1 - r2, +r1 + r2)
    ax.set_ylim(-r1 - r2, +r1 + r2)
    #: Set labels and title
    ax.set_xlabel("Y-axis")
    ax.set_ylabel("Z-axis")
    ax.set_title("2D Plot of Circles, Angles q0 and q1")
    #: Add legend
    ax.legend()
    #: Show the plot
    plt.grid(True)


def CalculateDistance(positionStart, positionEnd):
    #: Calculate the distance between the current and desired positions
    distance = np.linalg.norm(positionEnd[1:3] - positionStart[1:3])
    return distance


def CalculateAngleDifference(angleStart, angleEnd):
    #: Calculate the difference between the current and desired angles
    if angleStart[0] is None or angleStart[1] is None or angleEnd[0] is None or angleEnd[1] is None:
        print("Error: One of the angles is None, returning (0, 0)")
        return 0, 0   
    q0diff = np.abs(angleStart[0] - angleEnd[0])
    q1diff = np.abs(angleStart[1] - angleEnd[1])
    return q0diff, q1diff


def GetArmLinearPositions(PositionStart, PositionEnd, maxDistance=1.0):
    #: Move the arm to a desired position gradually in a linear fashion'
    global xposition
    xposition = PositionEnd[0]  #: Update the global x position variable
    distance = CalculateDistance(PositionStart, PositionEnd)
    if distance < maxDistance:
        return [
            PositionEnd
        ]  #: If the distance is less than the maximum distance, return the end position

    Positions = np.linspace(
        PositionStart, PositionEnd, num=int(distance / maxDistance) + 1
    )  #: Generate a linear path from start to end position
    return Positions  #: Return the list of positions to move through


def MoveArmToPositionGraduallyJoint(angleStart, angleEnd, maxAngle=0.08726, sync: bool = True):
    angledifferenceq0, angledifferenceq1 = CalculateAngleDifference(angleEnd, angleStart)
    if angledifferenceq0 == 0 or angledifferenceq1 == 0:
        print("Error: Angles are the same, returning empty list")
        return [angleEnd]
    if not sync:

        #: If the angles are not to be moved in sync, calculate the angles separately
        anglesq0 = np.linspace(
            angleStart[0], angleEnd[0], num=int(angledifferenceq0 / maxAngle) + 1
        )  #: Generate a linear path for q0

        anglesq1 = np.linspace(
            angleStart[1], angleEnd[1], num=int(angledifferenceq1 / maxAngle) + 1
        )  #: Generate a linear path for q1

        num_steps = max(len(anglesq0), len(anglesq1))
        anglesq0 = list(anglesq0) + [anglesq0[-1]] * (num_steps - len(anglesq0))
        anglesq1 = list(anglesq1) + [anglesq1[-1]] * (num_steps - len(anglesq1))
        angles = [
            [round(x, 2), round(y, 2), angleEnd[2]] for x, y in zip(anglesq0, anglesq1)
        ]  #: Combine the angles into a list of angles

    else:
        #: If the angles are to be moved in sync, calculate the maximum angle difference then use that to determine the number of steps
        divider = max(
            angledifferenceq0, angledifferenceq1
        )  #: Use the maximum angle difference to determine the number of steps
        anglesq0 = np.linspace(
            angleStart[0], angleEnd[0], num=int(divider / maxAngle) + 1
        )
        anglesq1 = np.linspace(
            angleStart[1], angleEnd[1], num=int(divider / maxAngle) + 1
        )
        angles = [
            [round(q0, 2), round(q1, 2)] for q0, q1 in zip(anglesq0, anglesq1)
        ]  #: Combine the angles into a list of angles
    return angles  #: Return the list of angles to move through

def movealongpos(positions, linkLength, delay=0.1):
    #: calculate the angles for each position and move the arm to those positions
    angles = []
    for pos in positions:
        #: Calculate the angles for each position using inverse geometry calculation
        angles.append(inversGeometryCalculation(linkLength, pos))
    for ang in angles:
        #: Move the arm to each angle
        if ang[0] is not None and ang[1] is not None and ang[2] is not None:
            convertToServoAngles(np.rad2deg(ang[0]), np.rad2deg(ang[1]), np.rad2deg(ang[2]), sleepTime=delay)
    #: save the last angles as the last position for using later
    global LastAngleq0
    LastAngleq0 = angles[-1][0]
    global LastAngleq1
    LastAngleq1 = angles[-1][1]

def movealongangle(angles, delay=0.1):
    #: Move the arm to a desired position gradually in a joint space
    for ang in angles:
        if ang[0] is not None and ang[1] is not None and ang[2] is not None:
            convertToServoAngles(np.rad2deg(ang[0]), np.rad2deg(ang[1]), np.rad2deg(ang[2]), sleepTime=delay)
        sleep(delay)
    #: save the last angles as the last position for using later
    global LastAngleq0
    LastAngleq0 = angles[-1][0]
    global LastAngleq1
    LastAngleq1 = angles[-1][1]


def clamp(n, min, max):
    #: Clamp a value n between a minimum and maximum value
    if n < min:
        return min
    elif n > max:
        return max
    else:
        return n

def convertToServoAngles(q0,q1,q2,sleepTime=0.01):
    #: Convert angles in degrees to servo angles and move the servos to those angles
    global servo1, servo2, servo3
    #: Check if servos are initialized
    if 'servo1' not in globals() or 'servo2' not in globals() or 'servo3' not in globals():
        #: If servos are not initialized, try to import them from Setup.py
        try:
            from Setup import servo1 as s1, servo2 as s2, servo3 as s3
            servo1, servo2, servo3 = s1, s2, s3
            print("Servos imported from Setup.py.")
        except Exception as e:
            print("Error: Servos are not initialized and could not be imported. Please call startall() from Setup.py before moving the arm.")
            print("Import error:", e)
            return False
        
    #: Covert angles to real servo angles
    q0 = np.round(90-q0, decimals=2, out=None)
    q1 = np.round(60+q1, decimals=2, out=None)
    q2 = np.round(q2, decimals=2, out=None)
    
    #: Clamp the angles to the servo limits
    if not 0 <= q0 <= 90:
        q0 = clamp(q0, 0, 90)
    if not 0 <= q1 <= 140:
        q1 = clamp(q1, 0, 50)
    if not 0 <= q2 <= 270:
        q2 = clamp(q2, 0, 270)
        
    #: Set the servo angles
    servo1.angle = q0
    servo2.angle = q1
    servo3.angle = q2
    
    #: Wait for the servos to reach the desired position
    sleep(sleepTime)  




def moveArmToPickObject(detectionobjects,chosen_idx):
    #: Move the arm to pick an object based on the detection objects
    #: Get the distance and x coordinate from the first detection object
    dist = detectionobjects[chosen_idx].distance2_mm
    x = detectionobjects[chosen_idx].y_cord+detectionobjects[chosen_idx].size/2
    
    #: grab the link length from the global variable
    global linkLength
    
    #: Calculate the end position based on the distance and x coordinate
    endposition = np.array([x, dist+15, 10])
    #: Move the arm above the object joint-wise
    movealongangle(MoveArmToPositionGraduallyJoint([LastAngleq0, LastAngleq1], inversGeometryCalculation(linkLength, endposition), maxAngle=0.08726, sync=False), 0.02)

    #: Move the arm above the objebect linearly
    endposition = np.array([x, dist+15, -10])
    movealongpos(GetArmLinearPositions(directGeometryCalculation(linkLength, [LastAngleq0, LastAngleq1]), endposition, maxDistance=0.2),linkLength,0.01)
    
    #: Activate the vacuum to pick the object
    Vaccum.value = 1.0
    
    #: Move the arm to the final position to pick the object
    endposition = np.array([x, dist+15, -30-(600-dist)/100])
    movealongpos(GetArmLinearPositions(directGeometryCalculation(linkLength, [LastAngleq0, LastAngleq1]), endposition, maxDistance=0.05),linkLength,0.01)
    
    #: Wait for the vacuum to create a seal
    sleep(1) 
    
    #: reduce the vacuum power to avoid to much force on the object
    Vaccum.value = 0.3
    
    #: Move the arm above the object
    endposition = np.array([x, dist+15, 50])
    movealongpos(GetArmLinearPositions(directGeometryCalculation(linkLength, [LastAngleq0, LastAngleq1]), endposition, maxDistance=0.2),linkLength,0.01)
    
    #: reset the servo angles to None to avoid overheating the servos
    servo1.angle = None
    servo2.angle = None
    servo3.angle = None
    return

def moveArmToDropObject():
    #: Move the arm to drop the object
    #: Move the arm above to avoid collisions
    movealongangle(MoveArmToPositionGraduallyJoint([LastAngleq0, LastAngleq1], np.radians([90,-10,270]), maxAngle=0.08726, sync=False), 0.02)
    #: Wait for the arm to reach the position
    sleep(1)
    #: move the arm higher to reach the drop position
    movealongangle(MoveArmToPositionGraduallyJoint([LastAngleq0, LastAngleq1], np.radians([90,60,270]), maxAngle=0.08726, sync=False), 0.02)
    #: turn off the vacuum to drop the object
    Vaccum.value = 0.0
    #: move above the drop position
    movealongangle(MoveArmToPositionGraduallyJoint([LastAngleq0, LastAngleq1], np.radians([90,60,0]), maxAngle=0.08726, sync=False), 0.02)
    #: Wait for the suction to release the object
    sleep(8)
    #: move the arm back to the original position
    movealongangle(MoveArmToPositionGraduallyJoint([LastAngleq0, LastAngleq1], np.radians([90,60,270]), maxAngle=0.08726, sync=False), 0.02)
    sleep(1)
    movealongangle(MoveArmToPositionGraduallyJoint([LastAngleq0, LastAngleq1], np.radians([90,-10,270]), maxAngle=0.08726, sync=False), 0.02)
    sleep(1)
    convertToServoAngles(np.rad2deg(LastAngleq0), np.rad2deg(LastAngleq1), 0, sleepTime=3 )
    
    #: reset the servo angles to None to avoid overheating the servos
    servo1.angle = None
    servo2.angle = None
    servo3.angle = None
    return

if test: 
    plt.show()

