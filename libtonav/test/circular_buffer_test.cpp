//
// Created by Tomas Krejci on 10/28/17.
//

#include "circular_buffer.hpp"

#include <gtest/gtest.h>

using namespace tonav;

TEST(CircularBufferTest, CanBeCreated) {
    CircularBuffer<int> b(5);
}

TEST(CircularBufferTest, CanAddItemsUpToFullCapacity) {
    CircularBuffer<int> b(5);
    b.pushBack(0);
    b.pushBack(1);
    b.pushBack(2);
    b.pushBack(3);
    b.pushBack(4);
}

TEST(CircularBufferTest, CanAddMoreThenMaxCapacityElements) {
    CircularBuffer<int> b(5);
    for (int i = 0; i < 10; ++i) {
        b.pushBack(i);
        b.popFront();
    }
}

TEST(CircularBufferTest, BufferSize) {
    CircularBuffer<int> b(5);
    EXPECT_EQ(0, b.size());
    b.pushBack(0);
    EXPECT_EQ(1, b.size());
    b.pushBack(1);
    EXPECT_EQ(2, b.size());
    b.pushBack(2);
    EXPECT_EQ(3, b.size());
    b.pushBack(3);
    EXPECT_EQ(4, b.size());
    b.pushBack(4);
    EXPECT_EQ(5, b.size());
}

TEST(CircularBufferTest, BufferSizeOverMaxCapacity) {
    CircularBuffer<int> b(5);
    for (int i = 0; i < 10; ++i) {
        b.pushBack(i);
        EXPECT_EQ(1, b.size());
        b.popFront();
        EXPECT_EQ(0, b.size());
    }
}

TEST(CircularBufferTest, CanAccessFrontAndBack) {
    CircularBuffer<int> b(5);
    for (int i = 0; i < 5; ++i) {
        b.pushBack(i);
    }
    b.popFront();
    b.pushBack(5);
    EXPECT_EQ(1, b.front());
    EXPECT_EQ(5, b.back());
}

TEST(CircularBufferTest, OneBeforeEndIsBackElement) {
    CircularBuffer<int> b(5);
    for (int i = 0; i < 5; ++i) {
        b.pushBack(i);
    }
    auto it = std::prev(std::end(b));
    EXPECT_EQ(4, *it);
}

TEST(CircularBufferTest, CanIterateForwardsUsingIterator) {
    CircularBuffer<int> b(5);
    for (int i = 0; i < 5; ++i) {
        b.pushBack(i);
    }
    int i = 0;
    for (auto it = std::begin(b); it != std::end(b); ++it) {
        EXPECT_EQ(i, *it);
        i += 1;
    }
}

TEST(CircularBufferTest, CanIterateBackwardsUsingIterator) {
    CircularBuffer<int> b(5);
    for (int i = 0; i < 5; ++i) {
        b.pushBack(i);
    }
    auto it = std::prev(std::end(b));
    for (int i = 4; i >= 0; --i) {
        EXPECT_EQ(i, *it);
        --it;
    }
}

TEST(CircularBufferTest, CanAccessElementsUsingSquareBracketOperator) {
    CircularBuffer<int> b(5);
    b.pushBack(0);
    b.popFront();
    for (int i = 1; i < 6; ++i) {
        b.pushBack(i);
    }
    for (int i = 1; i < 6; ++i) {
        EXPECT_EQ(i, b[i-1]);
    }
}
