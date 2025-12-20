#ifndef JOIN_THREADS_H
#define JOIN_THREADS_H

#include <iostream>

class JoinThreads
{
public:
	std::vector<std::thread>& threads;
	explicit JoinThreads(std::vector<std::thread>& _threads)
		:
		threads(_threads)
	{
	}

	~JoinThreads()
	{ 
		for (unsigned long i = 0; i < threads.size();++i)
		{
			if (threads[i].joinable())
			{
				threads[i].join();
			}
		}
	}
};

#endif
