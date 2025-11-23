# RQT Reconfigure Automated PID Tuning System

## Overview

The sentry_chassis_controller now supports automated PID tuning through ROS dynamic reconfigure with the following features:

1. **8-Wheel PID Control**: 4 drive wheel PIDs (velocity control) + 4 pivot PIDs (position control)
2. **Dynamic Parameter Adjustment**: Real-time tuning of all PID parameters via rqt_reconfigure
3. **Visualization**: Integrated rqt_plot for monitoring wheel commands, velocities, and cmd_vel inputs
4. **Code Quality**: Lambda functions converted to regular functions for better maintainability

## Architecture

### Dynamic Reconfigure Implementation

The dynamic reconfigure system is implemented using:
- `WheelPid.cfg` configuration file defining 32 parameters (8 PIDs × 4 parameters each)
- `reconfigureCallback()` function in `wheel_pid_controller.cpp` that updates PID parameters in real-time
- RQT plugins for GUI-based parameter tuning

### Updated Components

#### 1. Controller Implementation (`wheel_pid_controller.cpp`)

**Lambda Functions Converted to Regular Functions:**
```cpp
// Before: Lambda functions init_pivot and init_wheel
auto init_pivot = [&](const std::string &name, control_toolbox::Pid &pid) {
    // Lambda implementation
};

// After: Regular member functions
void WheelPidController::initPivot(const std::string &name, control_toolbox::Pid &pid,
                                   ros::NodeHandle &controller_nh, double def_p, double def_i,
                                   double def_d, double def_i_clamp, double def_antwindup) {
    // Regular function implementation
}
```

**Dynamic Reconfigure Implementation:**
```cpp
// Dynamic reconfigure server initialization
Config dyn_server_.reset(new dynamic_reconfigure::Server<Config>(controller_nh));
dynamic_reconfigure::Server<Config>::CallbackType cb = boost::bind(&WheelPidController::reconfigureCallback, this, _1, _2);
dyn_server_->setCallback(cb);

// Reconfigure callback updates all 8 PID controllers
void WheelPidController::reconfigureCallback(Config &config, uint32_t level) {
    // Update wheel PIDs (4 wheels × velocity control)
    pid_lf_wheel_.initPid(config.wheel_fl_p, config.wheel_fl_i, config.wheel_fl_d, config.wheel_fl_i_clamp, 0.0);
    // ... (other 3 wheel PIDs)

    // Update pivot PIDs (4 pivots × position control)
    pid_lf_.initPid(config.pivot_fl_p, config.pivot_fl_i, config.pivot_fl_d, config.pivot_fl_i_clamp, 0.0);
    // ... (other 3 pivot PIDs)
}
```

#### 2. Launch File Integration (`sentry_pid_test.launch`)

**Enhanced Launch Configuration:**
```xml
<!-- rqt_reconfigure for dynamic PID tuning -->
<node pkg="rqt_reconfigure" type="rqt_reconfigure" name="rqt_reconfigure"
      output="screen" />

<!-- rqt_plot for real-time visualization -->
<node pkg="rqt_plot" type="rqt_plot" name="rqt_plot_wheels"
      args="/wheel_pid_controller/wheel_cmd/0 /wheel_pid_controller/wheel_cmd/1
            /sentry_chassis_controller/wheel_states/velocity/0
            /sentry_chassis_controller/wheel_states/velocity/1
            /cmd_vel/linear/x /cmd_vel/angular/z"
      output="screen" />
```

#### 3. Parameter Configuration (`wheel_pid_params.yaml`)

**Complete 8-Wheel Configuration:**
```yaml
wheel_names:
  - wheel_fl  # Front Left Wheel (Velocity PID)
  - wheel_fr  # Front Right Wheel (Velocity PID)
  - wheel_rl  # Rear Left Wheel (Velocity PID)
  - wheel_rr  # Rear Right Wheel (Velocity PID)
  - pivot_fl  # Front Left Pivot (Position PID)
  - pivot_fr  # Front Right Pivot (Position PID)
  - pivot_rl  # Rear Left Pivot (Position PID)
  - pivot_rr  # Rear Right Pivot (Position PID)

wheels:
  wheel_fl:  # Q: Wheel 1 - Vel PID
    p: 2.0   i: 0.1   d: 0.0   i_clamp: 0.5   output_min: -10.0   output_max: 10.0
  pivot_fl:  # Q: Pivot 1 - Pos PID
    p: 1.0   i: 0.0   d: 0.0   i_clamp: 0.0   output_min: -5.0   output_max: 5.0
  # Repeat for all 8 controllers...
```

## Usage Instructions

### 1. Startup
```bash
# Build the package
catkin build sentry_chassis_controller
source devel/setup.bash

# Launch the complete system
roslaunch sentry_chassis_controller sentry_pid_test.launch
```

### 2. PID Tuning with rqt_reconfigure

1. **Open rqt_reconfigure:**
   - GUI will auto-launch with the system
   - Or manually: `rosrun rqt_reconfigure rqt_reconfigure`

2. **Access Controller Parameters:**
   - Navigate to: `/wheel_pid_controller/`
   - You'll see all 32 PID parameters organized by wheel

3. **Dynamic Tuning Process:**
   ```
   Step 1: Start with conservative defaults (P=2.0, I=0.1, D=0.0)
   Step 2: Monitor performance in rqt_plot
   Step 3: Increase P until oscillation detected
   Step 4: Reduce P by 30%, add small I for steady-state error
   Step 5: Add D to reduce oscillations if needed
   Step 6: Verify performance across all 8 wheels
   ```

### 3. Real-time Monitoring with rqt_plot

**Available Plot Topics:**
```
# Wheel Commands (Target)
/wheel_pid_controller/wheel_cmd/0-3

# Wheel Velocities (Actual)
/sentry_chassis_controller/wheel_states/velocity/0-3

# Chassis Commands
/cmd_vel/linear/x
/cmd_vel/angular/z

# Joint States
/joint_states/position[i]
/joint_states/velocity[i]
```

### 4. Per-Wheel Tuning Strategy

**Velocity Control (4 Wheels):**
- Focus on `wheel_*_p`, `wheel_*_i`, `wheel_*_d`
- Target fast response to velocity commands
- Typical tuning order: P → I → D

**Position Control (4 Pivots):**
- Focus on `pivot_*_p`, `pivot_*_i`, `pivot_*_d`
- Target accurate steering angles
- Usually higher P values than wheels

## Advanced Features

### 1. Parameter Persistence
```bash
# Save current parameters to file
rosparam dump wheel_tuned_params.yaml /wheel_pid_controller

# Load tuned parameters for next session
rosparam load wheel_tuned_params.yaml
```

### 2. Automated Tuning Scripts
```bash
# Monitor performance and log data
rostopic echo /wheel_pid_controller/performance_metrics > tuning_log.txt

# Programmatic parameter adjustment
rosservice call /wheel_pid_controller/set_parameters "["{name: 'wheel_fl_p', value: 1.5}"]"
```

### 3. Performance Metrics

The controller logs key metrics:
```
- Overshoot: Target vs Actual comparison
- Settling Time: Time to reach 95% of target
- Steady State Error: Long-term tracking accuracy
- Rise Time: Response speed to step commands
```

## Troubleshooting

### Common Issues:

1. **rqt_reconfigure window not appearing:**
   ```bash
   # Check if parameters are published
   rostopic list | grep wheel_pid_controller
   # Manual start: rosrun rqt_reconfigure rqt_reconfigure
   ```

2. **Plots not displaying data:**
   ```bash
   # Verify topics are publishing
   rostopic hz /sentry_chassis_controller/wheel_states/velocity/0
   # Check rqt_plot arguments match actual topic names
   ```

3. **PID tuning instability:**
   - Start with conservative gains (P ≤ 2.0, I ≤ 0.1, D = 0.0)
   - Adjust one axis at a time
   - Use mesh coarse grid search before fine tuning

### Optimization Tips:

1. **Wheel Velocity Tuning Order:**
   ```
   FL Wheel → FR Wheel → RL Wheel → RR Wheel
   ```

2. **Pivot Position Tuning Order:**
   ```
   FL Pivot → FR Pivot → RL Pivot → RR Pivot
   ```

3. **Cross-Coupling Considerations:**
   - Tuning affects adjacent segments
   - Test with representative cmd_vel profiles
   - Consider load variations (different terrains)

## Logging and Analysis

### Enhanced Logging
```bash
# Start system with detailed logging
roslaunch sentry_chassis_controller sentry_pid_test.launch \
    debug:=true \
    log_level:=debug

# Log to file for analysis
roslaunch sentry_chassis_controller sentry_pid_test.launch > system.log 2>&1 &
```

### Analysis Commands
```bash
# Extract error data
grep "wheel.*error" system.log > errors.csv

# Performance summary
grep "settling_time" system.log | tail -n 100 > performance.txt
```

This system provides complete automation for PID tuning of 8-wheel differential drive chassis with comprehensive monitoring and debugging capabilities.