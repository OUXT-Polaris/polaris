# POLARIS
"Policy and Object description Language for Automated Robot Intelligence with Semantic Behavior"

![ROS2-Eloquent](https://github.com/OUXT-Polaris/polaris/workflows/ROS2-Eloquent/badge.svg)

## What it is
DSL for behavior planning and object description in robotics.

## Purpose
Create waypoint from described map and policy

### Sample Code
1. define variable and four arithmetic operations
```
let a = (1.0 * 3.0) / 4;
```
2. Construct pose from quaternion and point
```
let q = quaternion(0,0,0,1);
let a = pose(point(1,2,3),q);
```
3. Calculate roration from two quaternions
```
let a = rpy(0,0,0)*quaternion(0.1,0,0.0,1);
```
rpy function construct quaternion type from RPY value

### Built-In types
```
int : integer type
double : double type
string : string type
bool : boolean type
quaternion : quaternion type
point : point type
pose : pose type
entity : entity(field objects) type
array type : integer/double/string/quaternion/point/pose/boolean/entity array support
```