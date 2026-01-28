#pragma once

namespace {
    static constexpr unsigned int A_FLAG = 0x8;
    static constexpr unsigned int B_FLAG = 0x4;
    static constexpr unsigned int DIR_FROM_NORTH = 0;
    static constexpr unsigned int DIR_FROM_SOUTH = 2;
    static constexpr unsigned int DIR_FROM_EAST = 1;
    static constexpr unsigned int DIR_FROM_WEST = 3;
    static constexpr unsigned int START_PARENT_SHIFT = 28;
    static constexpr unsigned int END_PARENT_SHIFT = 26;
    static constexpr unsigned int MAX_NUM_CHUNKS = 3; 
    static constexpr unsigned int SPLIT_MIN_CHUNKS = 2;
    static constexpr unsigned int CHUNK_CAPACITY = 127; 
}
