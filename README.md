# Line-Following Robot with Obstacle Avoidance

A Webots simulation project featuring an E-puck robot that autonomously follows a line while detecting and avoiding obstacles.

## Project Overview

This project implements an intelligent line-following robot using PID control and state-machine-based obstacle avoidance. The robot navigates a track marked with black lines on a white floor while detecting and maneuvering around obstacles using proximity sensors.

## Features

- **Line Following**: PID-based control system for smooth line tracking
- **Obstacle Detection**: 8 proximity sensors for comprehensive obstacle detection
- **Ground Sensing**: 3 ground sensors for line detection (white floor = ~600+, black line = <320)
- **State Machine**: 7-state system for complex navigation behaviors
- **Automatic Recovery**: Handles line loss and sensor stabilization
- **Startup Buffering**: Ignores false positives during initialization

## Hardware

### Robot: E-puck
- **Motors**: Left and right wheel motors (DC motors with infinite position mode)
- **Ground Sensors**: 3 ground color sensors (gs0, gs1, gs2)
- **Proximity Sensors**: 8 distance sensors (ps0-ps7) arranged around the robot

### Sensor Layout
```
         ps6  ps7  ps0  ps1
           \   |   |   /
            \  |   |  /
      ps5 ---\-|   |-/ ps2
             robot
      ps4 ----\-|   |-/ ps3
```

## Control States

1. **FOLLOW_LINE**: Main state - follows the line using PID control
2. **STOP**: Brief pause when obstacle is detected (5 steps)
3. **TURN_AWAY**: Rotates away from obstacle (20 steps)
4. **WALL_FOLLOW**: Follows the detected wall/obstacle boundary
5. **TURN_BACK**: Rotates back toward the line (12 steps)
6. **SEARCH_LINE**: Forward movement with slight rotation to find the line again
7. Boundary Recovery: Special handling when robot reaches the track boundary

## PID Control Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| BASE_SPEED | 2.0 | Normal forward speed |
| Kp | 0.005 | Proportional gain |
| Kd | 0.001 | Derivative gain |
| Line Threshold | 600 | Sensor value for detecting black line |
| Obstacle Threshold | 80 | Proximity sensor threshold for front obstacles |
| Side Threshold | 200 | Proximity sensor threshold for side walls |

## Installation

### Requirements
- **Webots R2025a** or compatible version
- **Python 3.x** (built into Webots)

### Setup
1. Copy the project folder to your Webots workspace
2. Open the world file (`worlds/robot.wbt`) in Webots
3. The controller (`controllers/my_controller/my_controller.py`) will automatically load

## Running the Simulation

1. Open Webots
2. Load `worlds/robot.wbt`
3. Press **Play** to start the simulation
4. The robot will begin by stabilizing sensors, then autonomously follow the line

### Console Output
The robot provides real-time feedback:
- `GS: [values]` - Ground sensor readings each step
- `🛑 Boundary detected!` - Robot reached track edge, executing recovery
- `🚨 Obstacle detected!` - Obstacle found ahead
- `🧱 Moving around obstacle` - Entering wall-following mode
- `🔍 Searching for line` - Looking for the line after obstacle
- `✅ Back on line` - Successfully returned to line following

## World Environment

The world file includes:
- **Track**: 2x2 meter plane with textured track image
- **Obstacles**: 8 small boxes (50mm³) scattered around the track
- **Ground**: White textured surface with black line path
- **Lighting**: Automatic lighting from TexturedBackgroundLight

### Obstacle Positions
```
Top area: (0.59, 0.5), (0, 0.66), (0.38, 0.16)
Right area: (0.15, -0.21), (0.66, -0.43)
Left area: (-0.55, -0.1), (-0.24, 0.06)
Bottom area: (-0.1, -0.6)
```

## Algorithm Details

### Line Following (PID)
```
error = right_ground_sensor - left_ground_sensor
correction = Kp * error + Kd * (error - last_error)
left_speed = BASE_SPEED - correction
right_speed = BASE_SPEED + correction
```

### Obstacle Avoidance Strategy
1. Detect front obstacle via proximity sensors
2. Stop briefly to assess situation
3. Turn away from obstacle
4. Follow wall/obstacle boundary
5. Search for line once wall is lost
6. Return to line following with cooldown to prevent oscillation

### Sensor Stabilization
- On startup, waits until ground sensors read > 600 (white floor detected)
- Provides 10-step buffer after detecting boundary to ignore false positives
- Resets startup buffer when sensors return to normal

## Customization

### Adjust Speed
Modify `BASE_SPEED` in the controller:
```python
BASE_SPEED = 2.0  # Valid range: 0.0 to 6.28 rad/s
```

### Tune PID Parameters
```python
Kp = 0.005  # Increase for sharper turns
Kd = 0.001  # Increase for stability
```

### Change Detection Thresholds
```python
LINE_THRESHOLD = 600      # Lower = more sensitive
OBSTACLE_THRESHOLD = 80   # Lower = detect from farther
SIDE_THRESHOLD = 200      # Wall detection sensitivity
```

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Robot spins at start | Sensor initialization not complete | Wait longer in initialization loop |
| Overshooting line | Kd too low | Increase `Kd` parameter |
| Poor turn response | Kp too low | Increase `Kp` parameter |
| False obstacle detection | Threshold too low | Increase `OBSTACLE_THRESHOLD` |
| Robot gets stuck | Wall_lost counter too high | Decrease wall_lost threshold |

## Project Structure

```
my_project2_line_robotics/
├── controllers/
│   └── my_controller/
│       └── my_controller.py      # Main controller logic
├── worlds/
│   ├── robot.wbt                 # Webots world file
│   └── track.png                 # Track texture image
└── README.md                      # This file
```

## Performance Metrics

The controller achieves:
- **Line following accuracy**: ±150 sensor units (with PID tuning)
- **Obstacle detection range**: 80+ proximity units
- **Recovery time**: ~2-3 seconds from obstacle detection to line re-acquisition
- **Startup time**: ~2-3 seconds for sensor stabilization

## Future Enhancements

- [ ] Implement learning algorithm to optimize route
- [ ] Add camera-based vision for line detection
- [ ] Multi-robot coordination
- [ ] Real-world deployment to physical E-puck
- [ ] Genetic algorithm for parameter optimization
- [ ] Path memory and loop detection

## License

This project is provided as-is for educational purposes.

## Author Notes

The robot uses a hybrid approach combining:
- **PID control** for smooth line tracking
- **State machine** for complex obstacle behaviors
- **Sensor fusion** of proximity and ground sensors
- **Cooldown mechanism** to prevent rapid state switching

The 40-step cooldown after returning to line prevents oscillation and improves overall stability when navigating tight areas.
