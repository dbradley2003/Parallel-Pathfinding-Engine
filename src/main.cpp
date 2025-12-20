#include <iostream>
#include <vector>
#include <exception>
#include <filesystem>
#include "maze/Maze.h"

#include "solvers/MazeSolverDFS.h"
#include "solvers/MazeSolverBFS.h"
#include "solvers/MultiThreadedSolver.h"

#include "utils/Timer.h"

#define INPUT_NAME_SIZE 64



int main(int argc, char* argv[])
{
	char inFileName[INPUT_NAME_SIZE] = "mazes/Maze100Kx100K.data";
	Timer aTimer;
	Timer bTimer;
	Timer cTimer;

	std::cout << "\n";
	std::cout << "\n" << "Maze: start" << "(" << inFileName << ") ------------\n";

	Maze* pMaze;

	// MazeSolverBFS: Start
	std::cout << "\n Maze: MazeSolverBFS\n";

	pMaze = new Maze();
	pMaze->Load(inFileName);

	aTimer.start(); // DFS timer start
	MazeSolverBFS solverBFS(pMaze);
	std::vector<Direction>* pSolutionBFS = solverBFS.Solve();
	aTimer.stop(); // DFS timer end
    

    if(pSolutionBFS == nullptr)
    {
        std::cout << "Solution is nullptr" << std::endl;
        return -1;
    }

	pMaze->checkSolution(*pSolutionBFS);

	delete pSolutionBFS;
	delete pMaze;
	//---------------------------- (End)



	// MazeSolverDFS: Start
	std::cout << "\n Maze: MazeSolverDFS\n";

	pMaze = new Maze();
	pMaze->Load(inFileName);

	bTimer.start(); // DFS timer start
	MazeSolverDFS solverDFS(pMaze);
	std::vector<Direction>* pSolutionDFS = solverDFS.Solve();
	bTimer.stop(); // DFS timer end

	pMaze->checkSolution(*pSolutionDFS);

	delete pSolutionDFS;
	delete pMaze;
	//---------------------------- (End)




	// MazeSolverMT: Start
	std::cout << "\n Maze: MazeSolverMT\n";

	pMaze = new Maze();
	pMaze->Load(inFileName);

	cTimer.start(); // MT timer start
	MultiThreadedSolver solverMT(pMaze);
	std::vector<Direction>* pSolutionMT = solverMT.Solve();
	cTimer.stop(); // MT timer end

	pMaze->checkSolution(*pSolutionMT);

	delete pSolutionMT;
	delete pMaze;
	//---------------------------- (End)

	// ----------Results----------
	double bfsSeconds = aTimer.elapsedSeconds();
	double dfsSeconds = bTimer.elapsedSeconds();
	double mtSeconds = cTimer.elapsedSeconds();

	double bfsMilliseconds = aTimer.elapsedSeconds();
	double dfsMilliseconds = bTimer.elapsedSeconds();
	double mtMilliseconds = cTimer.elapsedSeconds();

	std::cout << "\n";

	std::cout << "Results(" << inFileName << "):" << std::endl;
	std::cout << "BFS Time: " << bfsSeconds << "s" << std::endl;
	std::cout << "DFS Time: " << dfsSeconds << "s" << std::endl;
	std::cout << "MT  Time: " << mtSeconds << "s" << std::endl;

}
