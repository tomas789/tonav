#include <gtest/gtest.h>

#include "ring_buffer.h"

TEST(RingBufferTest, CorrectMaxSize) {
    RingBuffer<int> c(5);
    EXPECT_EQ(5, c.max_size());
}

TEST(RingBufferTest, EmptyContainer) {
    RingBuffer<int> c(5);
    EXPECT_TRUE(c.empty());
}

TEST(RingBufferTest, InsertSingleElementTestNonEmpty) {
    RingBuffer<int> c(5);
    c.push_back(1);
    EXPECT_FALSE(c.empty());
}

TEST(RingBufferTest, InsertSingleElementTestSizeOne) {
    RingBuffer<int> c(5);
    c.push_back(1);
    EXPECT_EQ(1, c.size());
}

TEST(RingBufferTest, InsertSingleElementTestEqual) {
    RingBuffer<int> c(5);
    c.push_back(1);
    EXPECT_EQ(1, c[0]);
}

TEST(RingBufferTest, InsertOverLimitThrows) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    EXPECT_THROW({ c.push_back(6); }, std::runtime_error);
}

TEST(RingBufferTest, PopFrontShrinksSize) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    EXPECT_EQ(5, c.size());
    c.pop_front();
    EXPECT_EQ(4, c.size());
}

TEST(RingBufferTest, InsertOverLimitWithEmptyNoThrow) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    EXPECT_NO_THROW({
        for (int i = 0; i < 5; ++i) {
            c.pop_front();
            c.push_back(i + 5);
        }
    });
}

TEST(RingBufferTest, CopyConstructCorrectSize) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    
    RingBuffer<int> c2(c);
    
    EXPECT_EQ(c.size(), c2.size());
}

TEST(RingBufferTest, CopyConstructElementsEqual) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    
    RingBuffer<int> c2(c);
    
    for (int i = 0; i < c2.size(); ++i) {
        EXPECT_EQ(c[i], c2[i]);
    }
}

TEST(RingBufferTest, MoveConstructCorrectSize) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    
    RingBuffer<int> c2 = std::move(c);
    
    EXPECT_EQ(5, c2.size());
}

TEST(RingBufferTest, MoveConstructElementsEqual) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    
    RingBuffer<int> c2 = std::move(c);
    
    for (int i = 0; i < c2.size(); ++i) {
        EXPECT_EQ(i, c2[i]);
    }
}

TEST(RingBufferTest, IterateElementsWithIterator) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    
    int i = 0;
    for (auto it = std::begin(c); it != std::end(c); ++it) {
        EXPECT_EQ(i, *it);
        i += 1;
    }
}

TEST(RingBufferTest, EndIsLesserThenEverything) {
    RingBuffer<int> c(5);
    for (int i = 0; i < 5; ++i) {
        c.push_back(i);
    }
    
    for (auto it = std::begin(c); it != std::end(c); ++it) {
        EXPECT_LT(it, std::end(c));
    }
}