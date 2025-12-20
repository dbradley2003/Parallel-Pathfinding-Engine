#ifndef FRONTIER_H
#define FRONTIER_H

#include "Chunk.h"

class Frontier
{
	Chunk* head;
	Chunk* tail;

	unsigned int totalChunks;
	char pad[4];

	void pop_chunk()
	{
		Chunk* oldHead = head;

		head = oldHead->prev;

		if (oldHead == tail)
		{
			tail = head;
		}

		delete oldHead;
		totalChunks--;
	}

	void push_chunk(Chunk* newChunk)
	{

		if (head == nullptr)
		{
			head = newChunk;
			tail = head;
			return;
		}

		Chunk* oldHead = head;
		newChunk->prev = oldHead;
		oldHead->next = newChunk;
		newChunk->next = nullptr;
		head = newChunk;
	}

	Chunk* get_tail()
	{
		return tail;
	}

public:
	void split(Frontier& other)
	{
		if (totalChunks < 2)
		{
			return;
		}

		Chunk* curr = other.head;

		while (curr)
		{
			Chunk* prev = curr->prev;
			delete curr;
			curr = prev;
		}

		unsigned int keepAmount = totalChunks - (totalChunks / 2);
		unsigned int giveAmount = totalChunks / 2;

		Chunk* newThiefTail = this->tail;

		for (unsigned int i = 1; i < giveAmount; i++)
		{
			if (newThiefTail->next == nullptr)
			{
				break;
			}
			newThiefTail = newThiefTail->next;
		}

		Chunk* newOwnerTail = newThiefTail->next;

		other.head = newThiefTail;
		other.tail = this->tail;
		other.totalChunks = giveAmount;

		this->tail = newOwnerTail;

		if (this->tail != nullptr)
		{
			this->tail->prev = nullptr;
		}
		else
		{
			this->head = nullptr;
		}

		this->totalChunks = keepAmount;

	}

	void push(Task t)
	{
		if (head && !head->isFull())
		{
			head->data[head->top++] = t;
			return;
		}

		Chunk* newChunk = new Chunk();
		newChunk->data[newChunk->top++] = t;
		push_chunk(newChunk);
		totalChunks++;
	}


	bool tryPop(Task& t)
	{

		if (totalChunks < 3)
		{

			if (head == nullptr)
			{
				return false;
			}

			if (head->isEmpty())
			{
				if (head == tail)
				{
					return false;
				}

				pop_chunk();
				t = head->data[--head->top];
				return true;
			}

			t = head->data[--head->top];
			return true;
		}


		if (!head->isEmpty())
		{
			t = head->data[--head->top];
			return true;
		}

		pop_chunk();
		t = head->data[--head->top];
		return true;
	}

	bool isEmpty()
	{
		if (head == nullptr)
		{
			return true;
		}
		return (head == tail) && (head->top == 0);
	}

	size_t size() {
		return totalChunks;
	}

	Frontier()
	{
		head = new Chunk();
		tail = head;
		totalChunks = 1;
	}

	~Frontier()
	{

		Chunk* curr = head;
		while (curr)
		{
			Chunk* prev = curr->prev;
			delete curr;
			curr = prev;
		}
	}

	Frontier(Frontier&& other) noexcept
	{
		head = other.head;
		tail = other.tail;
		totalChunks = other.totalChunks;

		other.head = nullptr;
		other.tail = nullptr;
		other.totalChunks = 0;
	}

	Frontier& operator=(Frontier&& other) noexcept
	{
		if (this != &other)
		{
			Chunk* curr = head;
			while (curr)
			{
				Chunk* prev = curr->prev;
				delete curr;
				curr = prev;
			}
			head = other.head;
			tail = other.tail;
			totalChunks = other.totalChunks;

			other.head = nullptr;
			other.tail = nullptr;
			other.totalChunks = 0;
		}
        return *this;
	}
};

#endif
