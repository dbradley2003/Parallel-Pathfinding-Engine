#ifndef MAZE_H
#define MAZE_H

#include <atomic>
#include <string>
#include <vector>

#include "../structures/Position.h"
#include "Choice.h"
#include "Direction.h"

#define DEBUG_PRINT 0

class Maze {
public:
  Maze();
  Maze(const Maze &) = delete;
  Maze &operator=(const Maze &) = delete;
  ~Maze();

  Maze(int _width, int _height);
  void Load(const std::string &fileName);

  ListDirection getMoves(Position pos);
  bool canMove(Position pos, Direction dir);
  bool checkSolution(std::vector<Direction> &soln);

  Position getStart();
  Position getEnd();
  ;

  void setEast(Position pos);
  void setSouth(Position pos);
  unsigned int getCell(Position pos);
  void setCell(Position pos, unsigned int val);

  // data:
  std::atomic_uint *poMazeData;
  int height;
  int width;
  int solvable;
};

#endif
