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

		if (oldHead == tail) tail = head;

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
		if (totalChunks < SPLIT_MIN_CHUNKS)
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
		if (head != nullptr && !head->isFull())
		{
			head->data[head->top++] = t;
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
            if (this->isEmpty()) return std::optional<Task>(std::nullopt);
            else 
            {
                if (head->isEmpty()) { pop_chunk(); }
			    return std::optional<Task>(std::move(head->data[--head->top])); 
            }
	}

	bool isEmpty()
	{
		if (head == nullptr)
		{
			return true;
		}

		return (head == tail && head->isEmpty()); 
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
