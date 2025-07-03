#include "FloodFill.h"
#include <cmath>

FloodFill::FloodFill() {
  robotX = 0;
  robotY = 0;
  robotDirection = NORTH;
  targetX = MAZE_SIZE/2 - 1;
  targetY = MAZE_SIZE/2 - 1;
  wallThreshold = 15; // Default wall detection threshold in cm
}

void FloodFill::initialize() {
  // Initialize flood values - distance from target
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      floodValues[x][y] = abs(x - targetX) + abs(y - targetY); // Manhattan distance
      visited[x][y] = false;
      // Initialize walls - assume no walls initially
      for (int dir = 0; dir < 4; dir++) {
        walls[x][y][dir] = false;
      }
    }
  }
  
  // Set boundary walls
  for (int i = 0; i < MAZE_SIZE; i++) {
    walls[0][i][WEST] = true;        // Left boundary
    walls[MAZE_SIZE-1][i][EAST] = true;  // Right boundary
    walls[i][0][SOUTH] = true;       // Bottom boundary
    walls[i][MAZE_SIZE-1][NORTH] = true; // Top boundary
  }
  
  // Mark starting position as visited
  visited[robotX][robotY] = true;
}

void FloodFill::setTarget(int x, int y) {
  if (isValidCell(x, y)) {
    targetX = x;
    targetY = y;
  }
}

void FloodFill::setWallThreshold(int threshold) {
  wallThreshold = threshold;
}

void FloodFill::setRobotPosition(int x, int y, int direction) {
  if (isValidCell(x, y) && direction >= 0 && direction <= 3) {
    robotX = x;
    robotY = y;
    robotDirection = direction;
    visited[x][y] = true;
  }
}

int FloodFill::getFloodValue(int x, int y) {
  if (isValidCell(x, y)) {
    return floodValues[x][y];
  }
  return 255; // Invalid cell
}

bool FloodFill::hasWall(int x, int y, int direction) {
  if (isValidCell(x, y) && direction >= 0 && direction <= 3) {
    return walls[x][y][direction];
  }
  return true; // Assume wall if invalid
}

void FloodFill::updateWalls(int frontDist, int leftDist, int rightDist) {
  // Update walls based on current robot direction and sensor readings
  if (frontDist < wallThreshold) {
    walls[robotX][robotY][robotDirection] = true;
    // Also set the corresponding wall on the adjacent cell
    int nextX = robotX;
    int nextY = robotY;
    getNextCell(robotDirection, &nextX, &nextY);
    if (isValidCell(nextX, nextY)) {
      walls[nextX][nextY][(robotDirection + 2) % 4] = true;
    }
  }
  
  int leftDir = (robotDirection + 3) % 4; // Turn left
  if (leftDist < wallThreshold) {
    walls[robotX][robotY][leftDir] = true;
    int nextX = robotX;
    int nextY = robotY;
    getNextCell(leftDir, &nextX, &nextY);
    if (isValidCell(nextX, nextY)) {
      walls[nextX][nextY][(leftDir + 2) % 4] = true;
    }
  }
  
  int rightDir = (robotDirection + 1) % 4; // Turn right
  if (rightDist < wallThreshold) {
    walls[robotX][robotY][rightDir] = true;
    int nextX = robotX;
    int nextY = robotY;
    getNextCell(rightDir, &nextX, &nextY);
    if (isValidCell(nextX, nextY)) {
      walls[nextX][nextY][(rightDir + 2) % 4] = true;
    }
  }
}

void FloodFill::calculateFloodValues() {
  // Reset flood values to a high number
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      floodValues[x][y] = 255;
    }
  }
  
  // Set target cell to 0
  floodValues[targetX][targetY] = 0;
  
  // Queue for BFS
  int queue[MAZE_SIZE * MAZE_SIZE][2];
  int front = 0, rear = 0;
  
  // Add target to queue
  queue[rear][0] = targetX;
  queue[rear][1] = targetY;
  rear++;
  
  // BFS to calculate flood values
  while (front < rear) {
    int currentX = queue[front][0];
    int currentY = queue[front][1];
    front++;
    
    int currentValue = floodValues[currentX][currentY];
    
    // Check all four directions
    for (int dir = 0; dir < 4; dir++) {
      if (isAccessible(currentX, currentY, dir)) {
        int nextX = currentX, nextY = currentY;
        getNextCell(dir, &nextX, &nextY);
        
        if (floodValues[nextX][nextY] > currentValue + 1) {
          floodValues[nextX][nextY] = currentValue + 1;
          queue[rear][0] = nextX;
          queue[rear][1] = nextY;
          rear++;
        }
      }
    }
  }
}

int FloodFill::getOptimalDirection() {
  int minValue = 255;
  int bestDirection = -1;
  
  // Check all four directions and find the one with minimum flood value
  for (int dir = 0; dir < 4; dir++) {
    if (isAccessible(robotX, robotY, dir)) {
      int nextX = robotX, nextY = robotY;
      getNextCell(dir, &nextX, &nextY);
      
      if (floodValues[nextX][nextY] < minValue) {
        minValue = floodValues[nextX][nextY];
        bestDirection = dir;
      }
    }
  }
  
  return bestDirection;
}

void FloodFill::updateRobotPosition(int newDirection) {
  robotDirection = newDirection;
  getNextCell(robotDirection, &robotX, &robotY);
  if (isValidCell(robotX, robotY)) {
    visited[robotX][robotY] = true;
  }
}

bool FloodFill::hasReachedTarget() {
  return (robotX == targetX && robotY == targetY);
}

void FloodFill::getNextCell(int direction, int* x, int* y) {
  switch(direction) {
    case NORTH: (*y)++; break;
    case EAST:  (*x)++; break;
    case SOUTH: (*y)--; break;
    case WEST:  (*x)--; break;
  }
}

bool FloodFill::isValidCell(int x, int y) {
  return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

bool FloodFill::isAccessible(int x, int y, int direction) {
  if (!isValidCell(x, y)) return false;
  if (walls[x][y][direction]) return false;
  
  int nextX = x, nextY = y;
  getNextCell(direction, &nextX, &nextY);
  return isValidCell(nextX, nextY);
}

void FloodFill::printMaze() {
  Serial.println("Current Maze State:");
  Serial.print("Robot at (");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.print(") facing ");
  
  switch(robotDirection) {
    case NORTH: Serial.println("NORTH"); break;
    case EAST:  Serial.println("EAST"); break;
    case SOUTH: Serial.println("SOUTH"); break;
    case WEST:  Serial.println("WEST"); break;
  }
}

void FloodFill::printFloodValues() {
  Serial.println("Flood Values:");
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (floodValues[x][y] < 10) Serial.print("  ");
      else if (floodValues[x][y] < 100) Serial.print(" ");
      Serial.print(floodValues[x][y]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void FloodFill::printWalls() {
  Serial.println("Wall Information:");
  Serial.print("Walls at current position (");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.print("): ");
  
  if (walls[robotX][robotY][NORTH]) Serial.print("N ");
  if (walls[robotX][robotY][EAST]) Serial.print("E ");
  if (walls[robotX][robotY][SOUTH]) Serial.print("S ");
  if (walls[robotX][robotY][WEST]) Serial.print("W ");
  Serial.println();
}
