#pragma once

#include "../structures/Position.h"

struct Task {
  Task(const Position at, const Direction dir, const unsigned int flag)
      : at(at), dir(dir), flag(flag) {}

  Task() = default;

  Position at{0, 0};
  Direction dir{Direction::Uninitialized};
  unsigned int flag{0};
};
