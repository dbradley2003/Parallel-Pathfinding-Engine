#pragma once

#include <vector>
#include "SkippingMazeSolver.h"
#include "../maze/Maze.h"

class MazeSolverDFS : public SkippingMazeSolver
{
public:
	MazeSolverDFS(Maze* maze)
		: SkippingMazeSolver(maze)
	{
		assert(pMaze);
	}

	MazeSolverDFS() = delete;
	MazeSolverDFS(const MazeSolverDFS&) = delete;
	MazeSolverDFS& operator=(const MazeSolverDFS&) = delete;
	~MazeSolverDFS() = default;

	std::vector<Direction>* Solve() override
	{
		std::vector<Choice> pChoiceStack;
		pChoiceStack.reserve(VECTOR_RESERVE_SIZE);  // Optimized allocations...

		Choice ch;
		try
		{
			pChoiceStack.push_back(firstChoice(pMaze->getStart()));

			while (!(pChoiceStack.size() == 0))
			{
				ch = pChoiceStack.back();
				if (ch.isDeadend())
				{
					// backtrack.
					pChoiceStack.pop_back();

					if (!(pChoiceStack.size() == 0))
					{
						pChoiceStack.back().pChoices.pop_front();
					}

					continue;
				}

				pChoiceStack.push_back(follow(ch.at, ch.pChoices.front()));

			}
			// No solution found.
			return 0;
		}
		catch (SolutionFoundSkip e)
		{

			std::vector<Choice>::iterator iter = pChoiceStack.begin();
			std::vector<Direction>* pFullPath = new std::vector<Direction>();
			pFullPath->reserve(VECTOR_RESERVE_SIZE);  // Optimized allocations...

			// Get full solution path.
			Position curr = pMaze->getStart();
			Direction go_to = Direction::Uninitialized;
			Direction came_from = Direction::Uninitialized;

			while (!(curr == pMaze->getEnd()))
			{
				ListDirection pMoves = pMaze->getMoves(curr);

				if (Direction::Uninitialized != came_from)
				{
					pMoves.remove(came_from);
				}

				if (pMoves.size() == 1)
				{
					go_to = pMoves.front();
				}
				else if (pMoves.size() > 1)
				{
					go_to = iter++->pChoices.front();
				}
				else if (pMoves.size() == 0)
				{
					printf("Error in solution--leads to deadend.");
					assert(false);
				}

				pFullPath->push_back(go_to);
				curr = curr.move(go_to);
				came_from = reverseDir(go_to);
			}
			return pFullPath;
		}
	}
};
