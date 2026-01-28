#ifndef FRONTIER_H
#define FRONTIER_H

#include <optional>
#include "Chunk.h"
#include "../constants.h"

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
            tail = head;

		delete oldHead;
		totalChunks--;
	}

	void push_chunk(Chunk* newChunk)
	{
		if (!head)
		{
			head = newChunk;
			tail = head;
		}
        else
        {
		    Chunk* old = head;
		    newChunk->prev = old;
		    old->next = newChunk;
		    newChunk->next = nullptr;
		    head = newChunk;
        }
	}

public:
	void split(Frontier& other)
	{
		if (this->totalChunks < SPLIT_MIN_CHUNKS)
			return;

		unsigned int numKeepChunks = totalChunks - (totalChunks / 2);
		unsigned int numGiveChunks = totalChunks / 2;

		Chunk* pOtherNewHead = this->tail;
		for (unsigned int i = 1; i < numGiveChunks; i++)
		{
			if (pOtherNewHead->next == nullptr)
				break;

			pOtherNewHead = pOtherNewHead->next;
		}

		other.head = pOtherNewHead;
		other.tail = this->tail;
		other.totalChunks = numGiveChunks;
        
        // assign node before other's head as this frontier's tail
		this->tail = pOtherNewHead->next;

		if (this->tail != nullptr)
		{
			this->tail->prev = nullptr;
		}
		else
		{
			this->head = nullptr;
		}
		this->totalChunks = numKeepChunks;
	}

	void push(Task t)
	{
		if (head != nullptr && !head->isFull())
		{
			head->data[head->top++] = std::move(t);
		}
        else
        {
		    Chunk* newChunk = new Chunk();
		    newChunk->data[newChunk->top++] = std::move(t);
		    this->push_chunk(newChunk);
		    totalChunks++;
        }
	}

    std::optional<Task> tryPop()
	{
        if (this->isEmpty()) 
        { 
            return std::optional<Task>(std::nullopt);
        }
        else 
        {
            if (head->isEmpty()) 
                pop_chunk();

			return std::optional<Task>(std::move(head->data[--head->top])); 
        }
	}

	bool isEmpty()
	{
		if (head == nullptr)
			return true;

		return (head == tail && head->isEmpty()); 
	}

	size_t size() 
    {
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
		while (curr != nullptr)
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
