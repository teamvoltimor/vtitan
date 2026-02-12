# Ackermann Steering Model

This document describes the Ackermann steering geometry used by the simulated robot,
how it differs from differential drive, and the implications for navigation and control.

---

## Robot Specifications

| Parameter          | Value                |
|--------------------|----------------------|
| Wheelbase (L)      | 0.17 m               |
| Track width (T)    | 0.105 m              |
| Wheel radius       | 0.0216 m             |
| Max steering angle | 0.5236 rad (~30 deg) |

---

## Differential Drive vs. Ackermann Comparison

| Aspect              | Differential Drive                        | Ackermann Steering                              |
|---------------------|-------------------------------------------|--------------------------------------------------|
| Control input       | Left/right wheel velocities               | Forward speed + steering angle                   |
| Turning behavior    | Turns by varying wheel speeds             | Turns by angling the front wheels                |
| Minimum turn radius | Zero (can pivot in place)                 | L / tan(max_steering_angle) = ~0.29 m            |
| Pivot capability    | Yes -- can rotate about its own center    | No -- requires forward or reverse motion to turn |
| Real-world match    | Common in small indoor robots             | Cars, trucks, Ackermann-steered RC vehicles      |

---

## Control Message Semantics

The robot subscribes to `geometry_msgs/Twist` messages. However, the interpretation
differs from the standard differential-drive convention:

```
geometry_msgs/Twist
  linear.x   ->  Forward speed (m/s)
  angular.z   ->  Steering angle (rad), NOT angular velocity
```

**Key distinction:** In a differential-drive robot, `angular.z` represents the desired
angular velocity (rad/s) about the vertical axis. In this Ackermann model, `angular.z`
is reinterpreted as the **steering angle** of the front wheels in radians. A positive
value steers left; a negative value steers right.

The steering angle is clamped to the range `[-0.5236, +0.5236]` rad (approximately
-30 to +30 degrees).

---

## Ackermann Geometry

```
            Steering Angle (delta)
                  |
                  v
          +-------*-------+
         /|   Front Axle  |\
   d_i  / |               | \  d_o
       /  |               |  \
      /   |               |   \
     /    |               |    \
    /     |               |     \
   /      |               |      \
  /       |               |       \
 /        |       L       |        \
/         |  (wheelbase)  |         \
          |   0.17 m      |
          |               |
          +-------+-------+
              Rear Axle
          |       T       |
          |  (track width)|
          |   0.105 m     |
          |               |
          |               |
                  |
                  |  R (turn radius, measured to rear axle center)
                  |
                  |
                  * ICR (Instantaneous Center of Rotation)


  d_i  = inner wheel steering angle (larger)
  d_o  = outer wheel steering angle (smaller)
  L    = wheelbase  = 0.17 m
  T    = track width = 0.105 m
  R    = turn radius to rear axle center
  ICR  = instantaneous center of rotation, located along
         the line extending from the rear axle
```

### Ackermann Constraint

For true rolling (no tire slip), the inner and outer wheel angles satisfy:

```
  cot(d_o) - cot(d_i) = T / L
```

Both front wheels must aim their axes toward the same instantaneous center of
rotation (ICR), which lies on the line extending from the rear axle.

---

## Turn Radius Calculation

Given a commanded steering angle `delta`:

```
  R = L / tan(delta)
```

Where:
- `R` is the turn radius measured from the ICR to the center of the rear axle.
- `L` is the wheelbase (0.17 m).
- `delta` is the steering angle in radians.

### Examples

| Steering angle (rad) | Steering angle (deg) | Turn radius (m) |
|----------------------:|---------------------:|-----------------:|
|                0.0500 |                 2.87 |            3.400 |
|                0.1000 |                 5.73 |            1.693 |
|                0.2000 |                11.46 |            0.833 |
|                0.3000 |                17.19 |            0.551 |
|                0.4000 |                22.92 |            0.404 |
|                0.5236 |                30.00 |            0.294 |

**Minimum turn radius** at maximum steering angle:

```
  R_min = 0.17 / tan(0.5236) = 0.17 / 0.5774 = ~0.29 m
```

---

## Navigation Adaptation Notes

Switching from differential drive to Ackermann steering requires several changes
in the navigation logic.

### 1. No Pivot Turns

Differential-drive robots can rotate in place by spinning the wheels in opposite
directions. An Ackermann-steered robot **cannot pivot**. It must maintain forward
(or reverse) speed in order to change heading. Any navigation code that previously
issued a pure rotation command (linear.x = 0, angular.z != 0) must be revised.

### 2. Steering Angle Clamping

The `angular.z` field in the Twist message is clamped to the maximum steering angle:

```
  -0.5236 rad  <=  angular.z  <=  +0.5236 rad
```

Any value outside this range is truncated. Navigation nodes must account for
this limit when computing desired heading corrections.

### 3. Minimum Turn Radius Constraint

Because the minimum turn radius is approximately 0.29 m, the robot cannot
execute sharp turns in tight spaces. Path planners and reactive controllers
must respect this kinematic constraint:

```
  R_min = 0.17 / tan(0.5236) = ~0.29 m
```

### 4. Escape Maneuver: Reverse + Steering

When the robot is stuck or needs to recover from a dead end, it can no longer
pivot to reorient. Instead, the escape maneuver uses **reverse motion combined
with a steering angle** to arc backward and reorient, analogous to a three-point
turn in a car.

### 5. Predictive Turning Magnitudes Reduced

In differential-drive navigation, large angular velocities (e.g., 1.0--2.0 rad/s)
are common for sharp turns. With Ackermann steering, the `angular.z` field
represents a steering angle, not an angular velocity. Steering angles are
inherently smaller in magnitude (capped at ~0.52 rad). Predictive turning
commands must be scaled down accordingly.

### 6. Side Corrections Reduced

Lateral correction gains have been reduced from **0.2** to **0.1** to account
for the less aggressive turning response of Ackermann steering. Larger correction
values caused oscillatory behavior because the steering geometry introduces a
delayed, arc-based response rather than the instantaneous yaw change available
with differential drive.

---

## Summary

The Ackermann steering model provides a more realistic kinematic simulation that
matches real-world car-like vehicles. The primary trade-offs compared to
differential drive are the loss of pivot-turn capability and the introduction of
a minimum turn radius. Navigation algorithms must be adapted to respect these
constraints, use reverse-arc escape maneuvers, and apply reduced correction gains.
