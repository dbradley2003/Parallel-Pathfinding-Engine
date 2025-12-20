#ifndef CHUNK_H
#define CHUNK_H

#include "Task.h"

struct Chunk
{
	static const unsigned int CHUNK_CAPACITY = 127;

	Chunk* next = nullptr;
	Chunk* prev = nullptr;
	unsigned int top = 0;
	Task data[CHUNK_CAPACITY];
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
