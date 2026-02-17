#ifndef CHUNK_H
#define CHUNK_H

#include <array>

#include "../constants.h"
#include "Task.h"

struct Chunk {
  void push(const Task t) { data[top++] = t; }
  Task pop() { return data[top--]; }
  ~Chunk() = default;

  [[nodiscard]] bool isEmpty() const { return top == 0; }
  [[nodiscard]] bool isFull() const { return top == CHUNK_CAPACITY; }
  [[nodiscard]] size_t size() const { return top; }

  // data:
  Chunk *next{nullptr};
  Chunk *prev{nullptr};
  unsigned int top{0};
  std::array<Task, CHUNK_CAPACITY> data;
};

#endif
