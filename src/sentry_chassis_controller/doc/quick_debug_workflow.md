# Quick FK Debug for RViz Issues

## The Problem
- ✅ Gazebo robot moves with keyboard control
- ✅ Forward kinematics publishes something to `/odom`
- ❌ RViz robot displays NO movement
- ❌ Odom shows all zero values

## Rapid Diagnostic (5-minute test)

### Step 1: Check What's Live
```bash
source /home/idris/final_ws/devel/setup.bash
rosnode list | grep -E "(keyboard|forward|sentry)"
rostopic hz /cmd_vel
rostopic hz /joint_states
rostopic hz /odom
```

### Step 2: Live Motion Check
```bash
# Terminal 1: Start your test launch
roslaunch sentry_chassis_controller sentry_with_odom.launch

# Terminal 2: Monitor joint states LIVE
rostopic echo /joint_states -n 5
defaults on first msg, then press keys quickly!
```

## What Varies? Check These:

### 🔎 **Expected Output**:
**Joint values **CHANGES**  **➜  velocities calculated ➜ odom position changes
** OR
**Joint values **ZERO**  ➜  no velocities  ➜  zero odom

Use  `rostopic echo /joint_states position[0] position[1] position[2] position[3]` while moving robot

### Case 1: Wheels Actually Moving
```
# Shows as wheel position changes:
position: [ 0.123, -0.125,  0.126, -0.124,  0.130, ...]
vs starting at [0.000, 0.000, 0.000, 0.000, ...]
```
→ Excellent! Issue is in velocity calculation math

### Case 2: Wheels NOT Moving
```
# Wheel positions stay zero:
position: [0.000, 0.000, 0.000, 0.000] always
```
→ Control issue: cmd_vel → controller → Gazebo → joint states broken

## Most Common Issues Explained

### **Problem A: Low Position Changes**
If values change but very small (±0.01): Wheel joint scale issue. Ensure:
```
# Wheel scaling:
wheel_radius = .05  # Verify this matches
wheel_base = .36    # Robot dimensions
wheel_track = .36   # Should match joint
```

### **Problem B: Wrong Joint Names**
If `rostopic echo /joint_states` shows different names:
```
# Expected: [left_front_wheel_joint, right_front_wheel_joint, ...]
# Actual:  [drive_lf, front_left_motor, fl_wheel_joint ]
```
→ Update joint names in [`forward_kinematics.cpp:25-31`](/home/idris/final_ws/src/sentry_chassis_controller/src/forward_kinematics.cpp#L25)

### **Problem C: Pivot vs Wheel Confusion**
If only pivot joints move, wheel positions stay:
```
positions showing  .    .
changed values⬆ but wheel positions ↔ zero
pivot joints are turning but wheel angular values not updating
```

## Fast Fix Chances

Most likely **ONE of these**:

1. **Joint names mismatch** in localization code
2. **Scale issue**: Gazebo wheels move fine but tiny Δ/Δt
3. **Controller not publishing GAZEBO joint velocities/positions to ROS**

Test this basic verification checklist and the key output from joint states will tell us exactly where to focus!