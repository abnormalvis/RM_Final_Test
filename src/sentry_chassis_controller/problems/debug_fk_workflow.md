# Forward Kinematics Debug Workflow

## Current Problem
- ✅ Gazebo robot moves with keyboard control
- ✅ Keyboard control publishes /cmd_vel
- ✅ Forward kinematics publishes /odom (but all zero values)
- ❌ RViz robot doesn't move (position stays at 0,0,0)

## Step 1: Run Diagnostic Script

```bash
# Terminal 1 - Start ROS (if not already)
roscore

# Terminal 2 - Launch debug version
source /home/idris/final_ws/devel/setup.bash
roslaunch sentry_chassis_controller sentry_with_odom.launch

# Terminal 3 - Run the debug monitor
source /home/idris/final_ws/devel/setup.bash
rosrun sentry_chassis_controller debug_fk_simple.py
```

## Step 2: Move Robot While Debug Script Runs

Use the keyboard control that works:
- Try **forward/reverse movement** (watch cmd_vel output)
- Try **rotation** (different wheels)

## What to Look For

### 🔍 **Expected Good Output**:
```
🎯 CMD_VEL detected: lin(0.50,0.00) ang(0.00)
⚙️ left_front_wheel_joint    POS=0.123   Δ=0.125     dt=0.100s ➜ VEL=1.250 rad/s
⚙️ right_front_wheel_joint  POS=-0.123  Δ=-0.125    dt=0.100s ➜ VEL=-1.250 rad/s
🚀 ODOM POSITION CHANGE detected after 2.3s!
   new pos: (0.0500, 0.0000, 0.0000)
```

### ❌ **Problem Indicators**:
```
❌ Still no odom movement after 15s
🔄 None of wheels are moving? All vel≈0
Or: All wheel joints have POS=0.000 forever
```

## Step 3: Analyze Results

### **Case A: Wheels Not Moving (Most Likely)**
If debug shows wheels not moving or position not changing:
1. **Check joint naming** - our expected names might be wrong
2. **Check controller output** - does the PID controller publish velocity to Gazebo?

### **Case B: Wheels Move But Odom Zero**
If wheels have velocity but odom stays 0:
1. **Check forward kinematics matrix calculation**
2. **Check the math inversion** - it might be getting zero solution

### **Case C: Wheels Move, Odom Changes Perfectly**
If everything updates correctly but RViz shows nothing:
1. **Check RViz TF display settings**
2. **Check fixed frame in RViz (should be odom)**
3. **Check modulation display in Odometry plugin**

## Quick Manual Check

```bash
# While debug script runs, check what commands produce movement:
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 2.0}}'
rostopic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 1.0}}'

rostopic hz /odom          # Check odom publishing rate
rostopic hz /joint_states  # Check joint state rate
```

## Where to Find Logs

Complete debug output by:
```bash
# Run with logging
erg &
rosrun sentry_chassis_controller debug_fk_simple.py > /tmp/debug.log 2>&1
# Check /tmp/debug.log for patterns
```

## Next Steps After Debug

Based on the output, we'll know which stage is broken:
1. **cmd_vel -> controller** problem
2. **controller -> joint states** problem
3. **joint states -> velocity calculation** problem
4. **velocity -> odometry matrix** problem
5. **odometry -> RViz display** problem

Let's start with this diagnostic first!