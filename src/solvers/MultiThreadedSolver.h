#ifndef MULTI_THREADED_SOLVER_H
#define MULTI_THREADED_SOLVER_H

#include <vector>
#include <future>
#include <iostream>

#include "MazeSolver.h"

#include "../maze/Direction.h"
#include "../concurrency/ThreadPool.h"

class MultiThreadedSolver : MazeSolver
{
public:
	MultiThreadedSolver(Maze* maze)
		: MazeSolver(maze)
	{
		assert(pMaze);
	}

	MultiThreadedSolver() = delete;
	MultiThreadedSolver(const MultiThreadedSolver&) = delete;
	MultiThreadedSolver& operator=(const MultiThreadedSolver&) = delete;
	~MultiThreadedSolver() = default;

	std::vector<Direction>* Solve() override
	{
		std::promise<std::vector<Direction>*> prom;
		std::future<std::vector<Direction>*> fut = prom.get_future();
		ThreadPool threadPool(this->pMaze, std::move(prom));

        try {
		    std::vector<Direction>* finalPath = fut.get();
		    return finalPath;
        }  
        catch(const std::future_error& e) {
            std::cerr << "Caught a future_error: " << e.what() << "\n";
            return nullptr;
        }
	}
};

#endif

