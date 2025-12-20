#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#include <vector>
#include <cassert>

#include "../maze/Maze.h"

class MazeSolver
{
public:
	MazeSolver(Maze* maze)
	{
		pMaze = maze;
		assert(this->pMaze);
	}

	MazeSolver() = delete;
	MazeSolver(const MazeSolver&) = delete;
	MazeSolver& operator = (const MazeSolver&) = delete;
	~MazeSolver() = default;

	virtual std::vector<Direction>* Solve() = 0;

protected:
	// data
	Maze* pMaze;

};

#endif
