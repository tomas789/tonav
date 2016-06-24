#ifndef TONAV_RING_BUFFER_H
#define TONAV_RING_BUFFER_H

#include <iterator>
#include <type_traits>
#include <stdexcept>

template <class T>
class RingBuffer;

template <class T, bool Const>
class RingBufferIterator {
public:
    friend class RingBuffer<T>;
    typedef std::random_access_iterator_tag iterator_category;
    
    using container_type = typename std::conditional<Const, const RingBuffer<T>, RingBuffer<T>>::type;
    using difference_type = typename std::iterator<std::random_access_iterator_tag, T>::difference_type;
    using size_type = typename container_type::size_type;
    using iterator = RingBufferIterator<T, Const>;
    using value_type = typename container_type::value_type;
    using reference = typename container_type::reference;
    using pointer = typename container_type::pointer;
    
    RingBufferIterator(const RingBufferIterator<T, Const>& other)
    : c_(other.c_), pos_(other.pos_) { }
    RingBufferIterator(RingBufferIterator<T, Const>&& other)
    : c_(other.c_), pos_(other.pos_) { }
    
    iterator& operator= (const iterator& other) {
        c_ = other.c_;
        pos_ = other.pos_;
        return *this;
    }
    
    iterator& operator= (iterator&& other) {
        c_ = other.c_;
        pos_ = other.pos_;
        return *this;
    }
    
    iterator& operator+= (difference_type i) {
        pos_ += i;
        return *this;
    }
    
    iterator& operator-= (difference_type i) {
        pos_ -= i;
        return *this;
    }
    
    reference operator* () const {
        return c_->operator[] (pos_);
    }
    
    pointer operator-> () const {
        return &(c_->operator[] (pos_));
    }
    
    reference operator[] (std::size_t i) const {
        return *(*this + i);
    }
    
    iterator& operator++ () {
        pos_ += 1;
        return *this;
    }
    
    iterator& operator-- () {
        pos_ -= 1;
        return *this;
    }
    
    iterator operator++ (int) const {
        iterator it(c_, pos_);
        pos_ += 1;
        return it;
    }
    
    iterator operator-- (int) const {
        iterator it(c_, pos_);
        pos_ -= 1;
        return it;
    }
    
    difference_type operator- (const iterator& other) {
        if (c_ != other.c_) {
            return std::runtime_error("Subtracting different iterators");
        }
        return pos_ - other.pos_;
    }
    
    iterator operator+ (difference_type i) const {
        return iterator(c_, pos_ + i);
    }
    
    iterator operator- (difference_type i) const {
        return iterator(c_, pos_ - i);
    }
    
    bool operator== (const iterator& other) const {
        return c_ == other.c_ && pos_ == other.pos_;
    }
    
    bool operator!= (const iterator& other) const {
        return c_ != other.c_ || pos_ != other.pos_;
    }
    
    bool operator< (const iterator& other) const {
        return pos_ < other.pos_;
    }
    
    bool operator> (const iterator& other) const {
        return pos_ > other.pos_;
    }
    
    bool operator<= (const iterator& other) const {
        return pos_ <= other.pos_;
    }
    
    bool operator>= (const iterator& other) const {
        return pos_ >= other.pos_;
    }
    
    
private:
    RingBufferIterator(container_type* c, size_type pos) {
        c_ = c;
        pos_ = pos;
    }
    
    container_type* c_;
    size_type pos_;
};

template <class T>
class RingBuffer {
public:
    using value_type = T;
    using reference = value_type&;
    using const_reference = const value_type&;
    using move_reference = value_type&&;
    using pointer = value_type*;
    using iterator = RingBufferIterator<T, false>;
    using size_type = std::size_t;
    using const_iterator = RingBufferIterator<T, true>;
    
    template <class K, bool C> friend class RingBufferIterator;
    
    RingBuffer(std::size_t max_size) {
        max_size_ = max_size;
        c_ = new char[sizeof(value_type)*max_size_];
        begin_ = 0;
        end_ = 0;
    }
    RingBuffer(const RingBuffer<T>& other) {
        c_ = nullptr;
        *this = other;
    }
    RingBuffer(RingBuffer<T>&& other) {
        c_ = nullptr;
        *this = std::move(other);
    }
    
    RingBuffer& operator= (const RingBuffer<T>& other) {
        max_size_ = other.max_size_;
        begin_ = other.begin_;
        end_ = other.end_;
        release_all_items();
        delete[] static_cast<pointer>(c_);
        c_ = new char[sizeof(value_type)*max_size_];
        for (size_type i = begin_; i < end_; ++i) {
            (*this)[i] = other[i];
        }
        return *this;
    }
    RingBuffer& operator= (RingBuffer<T>&& other) {
        max_size_ = other.max_size_;
        begin_ = other.begin_;
        end_ = other.end_;
        release_all_items();
        delete[] static_cast<pointer>(c_);
        c_ = other.c_;
        other.c_ = nullptr;
        return *this;
    }
    
    reference operator[] (std::size_t i) {
        return *(static_cast<pointer>(c_) + (i % max_size_));
    }
    const_reference operator[] (std::size_t i) const {
        return *(static_cast<pointer>(c_) + (i % max_size_));
    }
    
    iterator begin() {
        return iterator(this, begin_);
    }
    iterator end() {
        return iterator(this, end_);
    }
    
    const_iterator begin() const {
        return const_iterator(this, begin_);
    }
    const_iterator end() const {
        return const_iterator(this, end_);
    }
    
    const_iterator cbegin() const {
        return const_iterator(this, begin_);
    }
    const_iterator cend() const {
        return const_iterator(this, end_);
    }
    
    void push_back(const_reference value) {
        if (size() == max_size()) {
            throw std::runtime_error("Buffer is full");
        }
        end_ += 1;
        (*this)[end_ - 1] = value;
    }
    void push_back(move_reference value) {
        if (size() == max_size()) {
            throw std::runtime_error("Buffer is full");
        }
        end_ += 1;
        (*this)[end_ - 1] = std::move(value);
    }
    
    void pop_front() {
        (static_cast<pointer>(c_) + (begin_ % max_size_))->~value_type();
        begin_ += 1;
    }
    
    reference front() {
        assert(size() > 0);
        return operator[] (begin_);
    }
    reference back() {
        assert(size() > 0);
        return operator[] (end_ - 1);
    }
    const_reference front() const {
        assert(size() > 0);
        return operator[] (begin_);
    }
    const_reference back() const {
        assert(size() > 0);
        return operator[] (end_ - 1);
    }
    
    size_type max_size() const {
        return max_size_;
    }
    size_type size() const {
        return end_ - begin_;
    }
    bool empty() const {
        return begin_ == end_;
    }
    
    ~RingBuffer() {
        release_all_items();
        delete[] static_cast<pointer>(c_);
    }
private:
    std::size_t max_size_;
    void* c_;
    size_type begin_;
    size_type end_;
    
    void release_all_items() {
        pointer c_t_ = static_cast<pointer>(c_);
        for (size_type i = begin_; i < end_; ++i) {
            pointer ptr = c_t_ + (i % max_size_);
            ptr->~value_type();
        }
    }
};

#endif //TONAV_RING_BUFFER_H