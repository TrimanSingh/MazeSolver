#ifndef FLOODFILL_H
#define FLOODFILL_H

// #include <Arduino.h>

// Floodfill algorithm constants
#define MAZE_SIZE 16  // 16x16 maze
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

class FloodFill {
private:
  // Maze representation
  int floodValues[MAZE_SIZE][MAZE_SIZE];
  bool walls[MAZE_SIZE][MAZE_SIZE][4]; // [x][y][direction] - true if wall exists
  bool visited[MAZE_SIZE][MAZE_SIZE];
  
  // Robot position and orientation
  int robotX;
  int robotY;
  int robotDirection; // 0=North, 1=East, 2=South, 3=West
  
  // Target coordinates (center of maze)
  int targetX;
  int targetY;
  
  // Wall detection threshold
  int wallThreshold;

public:
  // Constructor
  FloodFill();
  
  // Initialization
  void initialize();
  void setTarget(int x, int y);
  void setWallThreshold(int threshold);
  void setRobotPosition(int x, int y, int direction);
  
  // Getters
  int getRobotX() { return robotX; }
  int getRobotY() { return robotY; }
  int getRobotDirection() { return robotDirection; }
  int getTargetX() { return targetX; }
  int getTargetY() { return targetY; }
  int getFloodValue(int x, int y);
  bool hasWall(int x, int y, int direction);
  
  // Wall detection and mapping
  void updateWalls(int frontDist, int leftDist, int rightDist);
  
  // Core floodfill algorithm
  void calculateFloodValues();
  int getOptimalDirection();
  
  // Movement and navigation
  void updateRobotPosition(int newDirection);
  bool hasReachedTarget();
  
  // Utility functions
  void getNextCell(int direction, int* x, int* y);
  bool isValidCell(int x, int y);
  bool isAccessible(int x, int y, int direction);
  
  // Debug functions
  void printMaze();
  void printFloodValues();
  void printWalls();
};

#endif
