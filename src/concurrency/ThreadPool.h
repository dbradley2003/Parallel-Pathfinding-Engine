#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <algorithm>
#include <array>
#include <atomic>
#include <exception>
#include <future>
#include <iostream>
#include <new>
#include <optional>
#include <random>
#include <thread>
#include <vector>

#include "../constants.h"
#include "../structures/Frontier.h"
#include "../structures/Message.h"
#include "../structures/Task.h"
#include "JoinThreads.h"

#define NO_REQUEST 0xFFFFFFFF

using namespace std;

template <typename T> class ThreadSafeUniquePointer {
public:
  ThreadSafeUniquePointer() = default;
  explicit ThreadSafeUniquePointer(T *p) : p_(p) {}
  ThreadSafeUniquePointer(const ThreadSafeUniquePointer &) = delete;
  ThreadSafeUniquePointer operator=(const ThreadSafeUniquePointer &) = delete;
  ~ThreadSafeUniquePointer() { delete p_.load(); }
  void publish(T *p) noexcept { p_.store(p, memory_order_release); }
  const T *get() const noexcept { return p_.load(memory_order_acquire); }
  const T &operator*() const noexcept { return *this->get(); }
  ThreadSafeUniquePointer& operator=(T *p) noexcept {
    delete p_.load(memory_order_relaxed);
    this->publish(p);
    return *this;
  }

  // ThreadSafeUniquePointer &operator=(T *p) noexcept {
  //   this->publish(p);
  //   return *this;
  // }

private:
  std::atomic<T *> p_{nullptr};
};

template class ThreadSafeUniquePointer<Message>;

struct alignas(std::hardware_destructive_interference_size * 2) PaddedMessage {
  ThreadSafeUniquePointer<Message> slot;
};

struct alignas(std::hardware_destructive_interference_size * 2) PaddedContainer {
  std::atomic<unsigned int> val{NO_REQUEST};
};

struct alignas(std::hardware_destructive_interference_size * 2) PaddedStatus {
  std::atomic<bool> status{false};
};

class ThreadPool {
public:
  Maze *pMaze;
  std::atomic<bool> done;
  std::atomic<int> intersectPos = -1;
  std::atomic<unsigned int> numThreads{0};

  std::promise<std::vector<Direction> *> prom;

  std::vector<Frontier> gFrontiers;
  std::array<PaddedStatus, MAX_THREADS> gStatuses;
  std::array<PaddedContainer, MAX_THREADS> gRequests;
  std::array<PaddedMessage, MAX_THREADS> gMailboxes;

  std::vector<std::thread> threads;
  JoinThreads joiner;
  inline static thread_local unsigned myIndex;

  Direction getParentDirection(const Position at, const unsigned shift) const {
    const unsigned int cell = pMaze->getCell(at);
    const unsigned int parentDirection = (cell >> shift) & 0x3;
    auto go_to = Direction::Uninitialized;

    switch (parentDirection) {
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

  Choice follow(Position at, const Direction dir, const unsigned int mFlag) {
    ListDirection Choices{};
    Direction go_to = dir;
    Direction came_from = reverseDir(dir);
    at = at.move(go_to);

    do {
      if (done.load())
        break;
      int intersectIndex = (at.row * pMaze->width) + at.col;
      int expected = -1;

      if (mFlag == A_FLAG) {
        MarkParentDirection(came_from, at, START_PARENT_SHIFT);
        if ((this->pMaze->poMazeData[at.row * this->pMaze->width + at.col]
                 .fetch_or(A_FLAG) &
             B_FLAG) == B_FLAG) {
          this->done = true;
          if (intersectPos.compare_exchange_strong(expected, intersectIndex)) {
            throw SolutionFoundSkip(at, reverseDir(go_to));
          }
          break;
        }

        if (at == pMaze->getEnd()) {
          this->done = true;
          if (intersectPos.compare_exchange_strong(expected, intersectIndex)) {
            throw SolutionFoundSkip(at, reverseDir(go_to));
          }
          break;
        }
      }

      if (mFlag == B_FLAG) {
        MarkParentDirection(came_from, at, END_PARENT_SHIFT);
        if ((this->pMaze->poMazeData[at.row * this->pMaze->width + at.col]
                 .fetch_or(B_FLAG) &
             A_FLAG) == A_FLAG) {
          this->done = true;
          if (intersectPos.compare_exchange_strong(expected, intersectIndex)) {
            throw SolutionFoundSkip(at, reverseDir(go_to));
          }
          break;
        }

        if (at == pMaze->getStart()) {
          this->done = true;
          if (intersectPos.compare_exchange_strong(expected, intersectIndex)) {
            throw SolutionFoundSkip(at, reverseDir(go_to));
          }
          break;
        }
      }

      Choices = pMaze->getMoves(at);
      Choices.remove(came_from);

      if (Choices.size() == 1) {
        go_to = Choices.begin();
        at = at.move(go_to);
        came_from = reverseDir(go_to);
      }
    } while (Choices.size() == 1);

    Choice pRet(at, came_from, Choices);
    return pRet;
  }

  bool hasIncomingRequest() const {
    return gRequests[myIndex].val.load(std::memory_order_acquire) != NO_REQUEST;
  }

  void send(const unsigned int threadID,
            const Message::MessageType messageType) {
    gMailboxes[threadID].slot = new Message(messageType);
  }

  Message::MessageType check(const unsigned threadID) const {
    if (gMailboxes[threadID].slot.get()) {
      return gMailboxes[threadID].slot.get()->type_;
    }
    return Message::MessageType::Empty;
  }

  void reply(const std::function<void(Frontier &other)> &callback) {
    const unsigned int j = gRequests[myIndex].val.load(std::memory_order_acquire);
    if (j == NO_REQUEST)
      return;

    callback(gFrontiers[j]);
    this->send(j, Message::MessageType::Success);
    gRequests[myIndex].val.store(NO_REQUEST, std::memory_order_release);
  }

  void rejectRequest() {
    const unsigned int j = gRequests[myIndex].val.load(std::memory_order_acquire);

    if (j == NO_REQUEST)
      return;

    this->send(j, Message::MessageType::RejectRequest);
    gRequests[myIndex].val.store(NO_REQUEST, std::memory_order_relaxed);
  }

  void acquire() {
    const unsigned int mIndex = myIndex;
    assert(!threads.empty());

    thread_local std::random_device rd;
    thread_local std::mt19937 gen(rd());
    const auto maxIndex = numThreads.load(memory_order_relaxed) - 1;
    std::uniform_int_distribution<unsigned int> dist(0, maxIndex);

    gRequests[mIndex].val.store(mIndex);
    unsigned int expected = NO_REQUEST;

    while (!done) {
      const unsigned int k = dist(gen);

      // thread[k] not available, skip...
      if (!gStatuses[k].status.load(memory_order_acquire)) {
        continue;
      }

      // r[k]: another thread already made request to thread[k], skip...
      if (!gRequests[k].val.compare_exchange_strong(
              expected, mIndex, memory_order_release, memory_order_relaxed)) {
        continue;
      }

      // request has been made, waiting for response
      constexpr unsigned LIMIT = 4;
      unsigned j = 0;
      while (check(mIndex) == Message::MessageType::Empty && !done) {
        if (j > LIMIT) {
          for (int i = 0; i < LIMIT; ++i) {
            std::this_thread::yield();
          }
          //std::this_thread::sleep_for(std::chrono::nanoseconds(1));
          //asm volatile("yield" ::: "memory");
          j = 0;
        }
        ++j;
      }
      if (done.load())
        return;

      if (check(mIndex) == Message::MessageType::Success) {
        // remove notification from this thread's slot
        gMailboxes[mIndex].slot = nullptr;
        break;
      }
      gMailboxes[mIndex].slot = nullptr;
    }

    gRequests[mIndex].val.store(NO_REQUEST);
  }

  void updateStatus() {
    bool b = (gFrontiers[myIndex].size() > MAX_NUM_CHUNKS);

    // ignore write if possible to avoid false sharing
    if (gStatuses[myIndex].status.load(memory_order_relaxed) != b) {
      gStatuses[myIndex].status.store(b, memory_order_release);
    }
  }

  void rebuildPath() {
    int lenA = 0;
    int lenB = 1;

    Direction next;
    const int finalIdx = intersectPos.load();
    const int finalRow = finalIdx / pMaze->width;
    const int finalCol = finalIdx % pMaze->width;
    const Position intersectPosition{finalRow, finalCol};

    Position pStart = intersectPosition;
    Position pEnd = intersectPosition;

    while (!(pStart == this->pMaze->getStart())) {
      next = getParentDirection(pStart, START_PARENT_SHIFT);
      pStart = pStart.move(next);
      lenA++;
    }

    while (!(pEnd == this->pMaze->getEnd())) {
      next = getParentDirection(pEnd, END_PARENT_SHIFT);
      pEnd = pEnd.move(next);
      lenB++;
    }

    const int length = (lenA + lenB) - 1;
    auto *finalPath = new std::vector<Direction>(length);

    Position pCurr = intersectPosition;
    int index = lenA - 1;

    while (index >= 0) {
      next = getParentDirection(pCurr, START_PARENT_SHIFT);
      pCurr = pCurr.move(next);
      (*finalPath)[index] = reverseDir(next);
      index--;
    }

    pCurr = intersectPosition;
    index = lenA;
    while (index < finalPath->size()) {
      next = getParentDirection(pCurr, END_PARENT_SHIFT);
      pCurr = pCurr.move(next);
      (*finalPath)[index] = next;
      index++;
    }
    this->prom.set_value(finalPath);
  }

  void workerThread(const unsigned mIndex) {
    myIndex = mIndex;
    Frontier &frontier = this->gFrontiers[myIndex];
    Choice curr{};
    try {
      while (!done) {
        if (frontier.isEmpty()) {
          acquire();
        } else {
          if (hasIncomingRequest()) {
            if (frontier.size() > MAX_NUM_CHUNKS) {
              reply([&frontier](Frontier &other) { frontier.split(other); });
            } else {
              rejectRequest();
            }
          }

          std::optional<Task> currentTask = frontier.tryPop();
          if (currentTask.has_value()) {
            const unsigned int collisionFlag = currentTask.value().flag;
            curr = follow(currentTask.value().at, currentTask.value().dir,
                          collisionFlag);

            while (curr.pChoices.size() > 0) {
              frontier.push(
                  Task{curr.at, curr.pChoices.pop_front(), collisionFlag});
            }

            updateStatus();
          }
        }
      }
    } catch (SolutionFoundSkip &e) {
      rebuildPath();
    }
  }

  void MarkParentDirection(const Direction dir, const Position pos,
                           const unsigned shift) const {
    const unsigned int oldValue =
        pMaze->poMazeData[pos.row * pMaze->width + pos.col];
    unsigned int newValue = oldValue;

    switch (dir) {
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
    default:
      break;
    }
    this->pMaze->poMazeData[pos.row * pMaze->width + pos.col] = newValue;
  }

  ThreadPool(Maze *pMaze, std::promise<std::vector<Direction> *> _prom)
      : pMaze(pMaze), done(false), prom(std::move(_prom)), joiner(threads) {
    unsigned const threadCount = std::thread::hardware_concurrency();
    numThreads.store(threadCount, memory_order_relaxed);
    try {
      gFrontiers.reserve(threadCount);
      const Position startPos = this->pMaze->getStart();
      const Position endPos = this->pMaze->getEnd();
      ListDirection startMoves = this->pMaze->getMoves(startPos);
      ListDirection endMoves = this->pMaze->getMoves(endPos);
      for (unsigned i = 0; i < threadCount; ++i) {
        gFrontiers.emplace_back();

        if (startMoves.size() > 0) {
          gFrontiers[i].push(Task{startPos, startMoves.pop_front(), A_FLAG});
        }

        else if (endMoves.size() > 0) {
          gFrontiers[i].push(Task{endPos, endMoves.pop_front(), B_FLAG});
        }
      }

      for (unsigned i = 0; i < threadCount; ++i) {
        threads.emplace_back(&ThreadPool::workerThread, this, i);
      }
    } catch (std::exception &e) {
      std::cout << "Error occurred creating thread\n";
      done = true;
    }
  }

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;
  ~ThreadPool() { done = true; }
};

#endif
