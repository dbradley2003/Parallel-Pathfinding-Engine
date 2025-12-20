#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <iostream>
#include <vector>
#include <exception>
#include <atomic>
#include <algorithm>
#include <random>
#include <thread>
#include <future>

#include "JoinThreads.h"

#include "../structures/Frontier.h"
#include "../structures/Message.h"
#include "../structures/Task.h"

#define NO_REQUEST 0xFFFFFFFF
#define NO_RESPONSE ((Message*)1)

//#define MAX_THREADS 64

class ThreadPool
{
public:
	static const unsigned int A_FLAG = 0x8;
	static const unsigned int B_FLAG = 0x4;

	static const unsigned int DIR_FROM_NORTH = 0;
	static const unsigned int DIR_FROM_SOUTH = 2;
	static const unsigned int DIR_FROM_EAST = 1;
	static const unsigned int DIR_FROM_WEST = 3;

	static const unsigned int START_PARENT_SHIFT = 28;
	static const unsigned int END_PARENT_SHIFT = 26;

	static const unsigned int SIZE_THRESHOLD = 3; // N * ( 128 * 16 (size of Task) ) bytes

	Maze* pMaze;
	std::atomic<bool> done;

	char padding[3];
	std::atomic<int> intersectPos = -1;

	std::promise<std::vector<Direction>*> prom;

	std::vector<Frontier> frontiers;

	bool s[MAX_THREADS];
	std::atomic<unsigned int> r[MAX_THREADS];
    Message *t[MAX_THREADS];
	std::vector<std::thread> threads;
	JoinThreads joiner;

	inline static thread_local unsigned myIndex; // thread local index

	Direction getParentDirection(Position at, unsigned shift)
	{
		unsigned int cell = pMaze->getCell(at);

		unsigned int parentDirection = (cell >> shift) & 0x3;

		Direction go_to = Direction::Uninitialized;

		switch (parentDirection)
		{
		case DIR_FROM_NORTH:
			go_to = Direction::North;
			break;
		case DIR_FROM_SOUTH:
			go_to = Direction::South;
			break;
		case DIR_FROM_EAST:
			go_to = Direction::East;
			break;
		case DIR_FROM_WEST:
			go_to = Direction::West;
			break;
		default:
			break;
		}
		return go_to;
	}

	Choice follow(Position at, Direction dir, unsigned int mFlag)
	{
		ListDirection Choices{};
		Direction go_to = dir;
		Direction came_from = reverseDir(dir);
		at = at.move(go_to);

		do
		{
			if (done.load())
			{
				break;
			}

			int intersectIndex = (at.row * pMaze->width) + at.col;
			int expected = -1;

			if (mFlag == A_FLAG)
			{
				MarkParentDirection(came_from, at, START_PARENT_SHIFT);

				if ((this->pMaze->poMazeData[at.row * this->pMaze->width + at.col].fetch_or(A_FLAG) & B_FLAG) == B_FLAG)
				{
					this->done = true;
					if (intersectPos.compare_exchange_strong(expected, intersectIndex))
					{
						throw SolutionFoundSkip(at, reverseDir(go_to));
					}
					else
					{
						break;
					}
				}

				if (at == pMaze->getEnd())
				{
					this->done = true;
					if (intersectPos.compare_exchange_strong(expected, intersectIndex))
					{
						throw SolutionFoundSkip(at, reverseDir(go_to));
					}
					else
					{
						break;
					}
				}
			}

			if (mFlag == B_FLAG)
			{
				MarkParentDirection(came_from, at, END_PARENT_SHIFT);

				if ((this->pMaze->poMazeData[at.row * this->pMaze->width + at.col].fetch_or(B_FLAG) & A_FLAG) == A_FLAG)
				{
					this->done = true;
					if (intersectPos.compare_exchange_strong(expected, intersectIndex))
					{
						throw SolutionFoundSkip(at, reverseDir(go_to));
					}
					else
					{
						break;
					}
				}

				if (at == pMaze->getStart())
				{
					this->done = true;
					if (intersectPos.compare_exchange_strong(expected, intersectIndex))
					{
						throw SolutionFoundSkip(at, reverseDir(go_to));
					}
					else
					{
						break;
					}
				}
			}

			Choices = pMaze->getMoves(at);
			Choices.remove(came_from);

			if (Choices.size() == 1)
			{
				go_to = Choices.begin();
				at = at.move(go_to);
				came_from = reverseDir(go_to);
			}

		} while (Choices.size() == 1);

		Choice pRet(at, came_from, Choices);
		return pRet;
	}

	bool hasIncomingRequest()
	{
		unsigned j = r[myIndex];

		if (j == NO_REQUEST)
		{
			return false;
		}

		return true;
	}

	void reply(std::function<void(Frontier& other)> callback)
	{
		unsigned int j = r[myIndex];

		if (j == NO_REQUEST)
		{
			return;
		}

		callback(frontiers[j]);

		Message* msg = new Message();
		msg->type = Message::MessageType::Success;

		t[j] = msg;
		r[myIndex] = NO_REQUEST;
	}

	void rejectRequest()
	{
		unsigned int j = r[myIndex];
        
		if (j == NO_REQUEST)
		{
			return;
		}
        
		t[j] = nullptr;
		r[myIndex] = NO_REQUEST;
	}

	void acquire()
	{
		unsigned int i = myIndex;

		if (threads.empty())
		{
			return;
		}
        
		thread_local std::random_device rd;
		thread_local std::mt19937 gen(rd());

		unsigned int maxIndex = (unsigned int) threads.size() - 1;
		std::uniform_int_distribution<unsigned int> dist(0, maxIndex);

		r[i] = i; //dummy value
        
		while (true)
		{
            if(done)
            {
                break;
            }

			t[i] = nullptr;

			unsigned int k = dist(gen);

			unsigned int expected = NO_REQUEST;

			if (s[k] && (r[k].compare_exchange_weak(expected, i)))
			{
				while (t[i] == nullptr && !done)
				{
				}

				if (done)
				{
					return;
				}

				if (t[i] != nullptr)
				{
                   
                    Message* msg = t[i];
                    delete msg;
                    t[i] = nullptr;                    
                    break;
                }
			}
            
		}
		r[i] = NO_REQUEST;
	}

	void updateStatus()
	{
		bool b = (frontiers[myIndex].size() > SIZE_THRESHOLD);

		if (s[myIndex] != b)
		{
			s[myIndex] = b;
		}
	}

	void rebuildPath()
	{
		int lenA = 0;
		int lenB = 1;

		Direction next = Direction::Uninitialized;
		int finalIdx = intersectPos.load();
		int finalRow = finalIdx / pMaze->width;
		int finalCol = finalIdx % pMaze->width;

		Position intersectPosition{ finalRow,finalCol };
		
		Position pStart = intersectPosition;
		Position pEnd = intersectPosition;

		while (!(pStart == this->pMaze->getStart()))
		{
			next = getParentDirection(pStart, START_PARENT_SHIFT);
			pStart = pStart.move(next);
			lenA++;
		}

		while (!(pEnd == this->pMaze->getEnd()))
		{
			next = getParentDirection(pEnd, END_PARENT_SHIFT);
			pEnd = pEnd.move(next);
			lenB++;
		}

		int length = (lenA + lenB) - 1;
		std::vector<Direction>* finalPath = new std::vector<Direction>(length);
		
		Position pCurr;
		pCurr = intersectPosition;

		int index = lenA - 1;
		
		while (index >= 0)
		{
			next = getParentDirection(pCurr, START_PARENT_SHIFT);
			pCurr = pCurr.move(next);
			(*finalPath)[index] = reverseDir(next);
			
			index--;
		}

		pCurr = intersectPosition;
		index = lenA;

		while (index < finalPath->size())
		{
			next = getParentDirection(pCurr, END_PARENT_SHIFT);
			pCurr = pCurr.move(next);
			(*finalPath)[index] = next;
			
			index++;
		}
		this->prom.set_value(finalPath);
	}

	void workerThread(unsigned mIndex)
	{
		myIndex = mIndex;
		Frontier& frontier = this->frontiers[myIndex];

		Choice curr{};
		Task tsk{};
        
		try {
			while (!done)
			{
				if (frontier.isEmpty())
				{
					acquire();
				}
				else
				{
					if (hasIncomingRequest())
					{
						size_t sz = frontier.size();


						if (sz > SIZE_THRESHOLD)
						{
							reply([&frontier](Frontier& other) {
								frontier.split(other);
								});
						}
						else
                        {
							rejectRequest();
						}
					}

					if (!(frontier.tryPop(tsk)))
					{
						continue;
					}

					unsigned int myFlag = tsk.flag;

					curr = follow(tsk.at, tsk.dir, myFlag);

					if (curr.pChoices.size() == 0)
					{
						continue;
					}


					Direction myDir = curr.pChoices.pop_front();
					ListDirection mvs = curr.pChoices;

					while (mvs.size() > 0)
					{
						Direction dir = mvs.pop_front();
						Position nextPos = curr.at.move(dir);
						frontier.push(Task{ curr.at, dir, myFlag });
					}


					frontier.push(Task{ curr.at,myDir, myFlag });
					updateStatus();
				}
			}
		}
		catch (SolutionFoundSkip e)
		{
			rebuildPath();
		}
	}

	void MarkParentDirection(Direction dir, Position pos, unsigned shift)
	{

		unsigned int oldValue = pMaze->poMazeData[pos.row * pMaze->width + pos.col].load();
		unsigned int newValue = oldValue;

		switch (dir)
		{
		case Direction::South:
			newValue |= (DIR_FROM_SOUTH << shift);
			break;
		case Direction::North:
			newValue |= (DIR_FROM_NORTH << shift);
			break;
		case Direction::East:
			newValue |= (DIR_FROM_EAST << shift);
			break;
		case Direction::West:
			newValue |= (DIR_FROM_WEST << shift);
			break;
		case Direction::Uninitialized:
			break;
		default:
			break;
		}

		this->pMaze->poMazeData[pos.row * pMaze->width + pos.col] = newValue;
	}

	ThreadPool(Maze* pMaze, std::promise<std::vector<Direction>*> _prom)
		:pMaze(pMaze),
		done(false),
		prom(std::move(_prom)),
		joiner(threads)
	{

		unsigned const threadCount = std::thread::hardware_concurrency();
        
		try {
			frontiers.reserve(threadCount);

			Position startPos = this->pMaze->getStart();
			ListDirection startMoves = this->pMaze->getMoves(startPos);

			Position endPos = this->pMaze->getEnd();
			ListDirection endMoves = this->pMaze->getMoves(endPos);

            for(int i{}; i < MAX_THREADS; ++i)
            {
                r[i].store(NO_REQUEST);
                t[i] = nullptr;
                s[i] = false;
            }
             
			for (unsigned i = 0; i < threadCount; ++i)
			{
				frontiers.emplace_back();

				if (startMoves.size() > 0)
				{
					frontiers[i].push(Task{ startPos,startMoves.pop_front(), A_FLAG });
					continue;
				}

				if (endMoves.size() > 0)
				{
					frontiers[i].push(Task{ endPos,endMoves.pop_front(),B_FLAG });
					continue;
				}
			}

			for (unsigned i = 0; i < threadCount; ++i)
			{
				threads.push_back(std::thread(&ThreadPool::workerThread, this, i));
			}
		}
		catch (std::exception e)
		{
			std::cout << "Error occurred creating thread\n";

			done = true;
		}
	}
    
	ThreadPool(const ThreadPool&) = delete;
    
    ThreadPool& operator=(const ThreadPool&) = delete;
	
    ~ThreadPool()
	{
		done = true;
	}
};

#endif
