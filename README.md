# Maze Solver with Floodfill Algorithm

## Overview

This project implements an autonomous maze-solving robot using the floodfill algorithm. The robot uses ultrasonic sensors to detect walls and navigate through a maze to find the optimal path to a target location. The implementation features a modular design with separate files for the algorithm and hardware control.

## Hardware Requirements

### Microcontroller
- Arduino-compatible board (Uno, Nano, etc.)

### Sensors
- 3 Ultrasonic sensors (HC-SR04 or similar)
  - Front sensor: Trigger A1, Echo A0
  - Left sensor: Trigger A2, Echo A3
  - Right sensor: Trigger A4, Echo A5

### Motors
- 4 DC motors with motor drivers (L298N or similar)
  - Front Left Motor: Enable pin 3, Input pins 12 & 13
  - Front Right Motor: Enable pin 11, Input pins 9 & 10
  - Rear Left Motor: Enable pin 5, Input pins 2 & 4
  - Rear Right Motor: Enable pin 6, Input pins 7 & 8

### Power Supply
- Appropriate voltage for motors and Arduino
- Ensure adequate current capacity for all motors

## File Structure

### MazeSolver.ino
The main Arduino sketch containing:
- Hardware initialization and control
- Sensor reading functions
- Motor control functions
- Main program loop
- Integration with FloodFill class

### FloodFill.h
Header file defining:
- FloodFill class interface
- Direction constants (NORTH, EAST, SOUTH, WEST)
- Maze size configuration
- Public method declarations

### FloodFill.cpp
Implementation file containing:
- Complete floodfill algorithm logic
- Maze representation and wall detection
- Breadth-first search for optimal pathfinding
- Robot position tracking
- Debug and utility functions

## Algorithm Description

### Floodfill Algorithm
The floodfill algorithm is a pathfinding technique that:
1. Creates a grid representation of the maze
2. Assigns values to each cell based on distance to target
3. Uses sensor data to detect and map walls
4. Recalculates optimal paths as new walls are discovered
5. Guides the robot along the shortest known path

### Key Features
- **Adaptive Learning**: Maps the maze as it explores
- **Optimal Pathfinding**: Always chooses the best known route
- **Wall Detection**: Uses ultrasonic sensors to identify obstacles
- **Position Tracking**: Maintains accurate robot location and orientation

## Configuration

### Maze Parameters
- **Maze Size**: 16x16 grid (configurable via MAZE_SIZE constant)
- **Target Location**: Center of maze (7, 7) by default
- **Wall Detection Threshold**: 15cm (adjustable)

### Movement Timing
- **Turn Duration**: 500ms for 90-degree turns
- **Cell Movement**: 1000ms to traverse one cell
- **Sensor Delay**: 500ms between movement cycles

### Motor Speeds
- **Forward Movement**: Differential speeds (255/85) for straight motion
- **Turning**: Full speed (255) for quick directional changes

## Usage Instructions

### Setup
1. Connect all hardware according to pin assignments
2. Upload the code to your Arduino
3. Place robot at maze starting position (0, 0)
4. Ensure robot faces North (positive Y direction)
5. Open Serial Monitor (9600 baud) for debug output

### Operation
1. Robot initializes floodfill algorithm
2. Begins maze exploration from starting position
3. Uses sensors to detect walls and update maze map
4. Calculates optimal path using floodfill values
5. Moves toward target following best available route
6. Continues until target is reached

### Debug Output
The Serial Monitor displays:
- Current robot position and orientation
- Sensor distance readings
- Movement decisions and directions
- Flood values for current position
- Error messages if navigation fails

## Customization Options

### Maze Configuration
```cpp
// Change maze size
#define MAZE_SIZE 12  // For 12x12 maze

// Modify target location
mazeSolver.setTarget(5, 5);  // Set target to (5, 5)

// Adjust wall detection sensitivity
mazeSolver.setWallThreshold(20);  // 20cm threshold
```

### Movement Tuning
```cpp
// Adjust turn timing
while (millis() - startTime < 600) { // 600ms turns

// Modify forward movement speed
FL_MOTOR.setSpeed(200, -1);  // Reduce speed to 200

// Change cell traversal time
while (millis() - startTime < 800) { // 800ms per cell
```

### Hardware Adaptation
- Modify pin assignments in variable declarations
- Adjust motor directions by changing speed parameters
- Calibrate sensor positions and orientations

## Troubleshooting

### Common Issues

**Robot doesn't move**
- Check motor connections and power supply
- Verify pin assignments match hardware
- Ensure adequate current for all motors

**Inaccurate wall detection**
- Calibrate ultrasonic sensor positions
- Adjust wall detection threshold
- Check for sensor interference or obstacles

**Navigation errors**
- Verify starting position and orientation
- Check maze boundary setup
- Ensure timing values match robot mechanics

**Serial output issues**
- Confirm 9600 baud rate setting
- Check USB connection and drivers
- Verify Serial.begin() in setup function

### Debug Features

The FloodFill class includes debug methods:
```cpp
mazeSolver.printMaze();        // Display current maze state
mazeSolver.printFloodValues(); // Show calculated distances
mazeSolver.printWalls();       // List detected walls
```

## Technical Specifications

### Algorithm Complexity
- **Time Complexity**: O(n²) for maze recalculation
- **Space Complexity**: O(n²) for maze representation
- **Update Frequency**: Real-time wall detection and path recalculation

### Performance Characteristics
- **Exploration Efficiency**: Optimal path convergence
- **Memory Usage**: Scalable with maze size
- **Response Time**: Near-instantaneous path updates

### Accuracy Requirements
- **Position Tracking**: Critical for algorithm success
- **Wall Detection**: Must be consistent and reliable
- **Movement Precision**: Affects cell-to-cell navigation

## Future Enhancements

### Potential Improvements
- **Speed Optimization**: Faster solving after initial exploration
- **Multiple Targets**: Support for sequential objectives
- **Dynamic Obstacles**: Handle moving walls or objects
- **Path Visualization**: Real-time maze display
- **Sensor Fusion**: Multiple sensor types for better accuracy

### Advanced Features
- **Return Navigation**: Optimal path back to start
- **Maze Validation**: Detect unsolvable configurations
- **Performance Metrics**: Timing and efficiency measurements
- **Remote Monitoring**: Wireless status updates
<!-- 
## License

This project is provided as-is for educational and research purposes. Feel free to modify and distribute according to your needs.

## Support

For technical questions or issues:
1. Check troubleshooting section above
2. Verify hardware connections and configurations
3. Review Serial Monitor output for error messages
4. Test individual components separately if needed

## Version History

- **Version 1.0**: Initial implementation with basic floodfill algorithm
- **Version 1.1**: Modular design with separate algorithm files
- **Version 1.2**: Enhanced debug features and configuration options -->
