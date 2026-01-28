#ifndef CHUNK_H
#define CHUNK_H

#include <array>
#include "Task.h"
#include "../constants.h"
struct Chunk
{
	Chunk* next = nullptr;
	Chunk* prev = nullptr;
	unsigned int top = 0;
    std::array<Task,CHUNK_CAPACITY> data;
	char padding[4];


	bool isEmpty()
	{
		return top == 0;
	}

	bool isFull()
	{
		return top == CHUNK_CAPACITY;
	}

	void push(Task t)
	{
		data[top++] = t;
	}

	size_t size()
	{
		return top;
	}

	Task pop()
	{
		return data[top--];
	}

	~Chunk() = default;
};

#endif
