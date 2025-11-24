# Debug Forward Kinematics Instructions

Now that you have the debug-enhanced version, here's how to test it:

## Step 1: Launch the System
```bash
# Source your workspace
source /home/idris/final_ws/devel/setup.bash

# Launch the simulation
roslaunch sentry_chassis_controller sentry_with_odom.launch
```

## Step 2: Check Debug Output
You should now see detailed logs from the forward_kinematics node every 5 seconds showing:

1. **Joint States Information**: All joint names published in `/joint_states`
2. **Joint Matching**: Which expected wheel joints were found
3. **Velocity Data**: Wheel velocities and pivot angles for each wheel
4. **Matrix Calculation**: The A matrix and b vector values
5. **Final Calculation**: The calculated vx, vy, wz velocities

## Step 3: What to Look For

### Issue 1: Joint Name Mismatch
**Problem**: Joint names don't match expected names
**Symptoms**:
- "Found 0 wheel joints out of 4"
- Joint names in message don't include expected ones

**Solution**:
- The debug log will show actual joint names from your robot
- Update the expected joint names in the code to match your robot's actual joint names

### Issue 2: All Wheel Velocities Are Zero
**Problem**: Joint states are available but all velocities are 0
**Symptoms**:
- "Wheel 0: vel=0.000 rad/s, v_along=0.000 m/s"
- "Solved velocities: vx=0.000, vy=0.000, wz=0.000"

**Solution**:
- This means the robot isn't moving or not receiving velocity commands
- Try using the keyboard teleop control to move the robot
- Check if controllers are publishing velocity

### Issue 3: Position Data Only (No Velocity)
**Problem**: Joint states only provide position, not velocity
**Symptoms**:
- "Velocity size: 0, Position size: [some number]"
- Wheel velocities estimated from position differences are very small

**Solution**:
- Make sure your controllers are publishing velocity data
- Or try to move the robot to generate position changes

## Step 4: What Your Robot Joint Names Might Be

Based on typical robot configurations, your actual joint names might be:
- `"wheel_fl_joint"`, `"wheel_fr_joint"` , `"wheel_rl_joint"`, `"wheel_rr_joint"`
- `"wheel_front_left_joint"` etc.
- Or something completely different

The debug output will show you exactly what joint names your robot is using.

## Step 5: Drive the Robot

Use keyboard control to generate movement:
```bash
# If using the included keyboard control, it should start automatically
# Otherwise start manual control
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

Then watch the debug output show non-zero velocities and changing poses.

## Step 6: Check TF and Odometry

Once movement is detected, verify:
```bash
# Check odom topic
rostopic echo /odom | head -20

# Check TF
tf_echo odom base_link
```

Report back what you see in the debug output! This will help us identify the exact issue.