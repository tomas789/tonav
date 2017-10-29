#ifndef TONAV_CIRCULAR_BUFFER_HPP
#define TONAV_CIRCULAR_BUFFER_HPP

#include <iterator>
#include <stdexcept>
#include <utility>

namespace tonav {

template <typename T>
class CircularBuffer {
public:
    template <bool IsConst> class Iterator;
    
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = typename std::add_pointer<value_type>::type;
    using reference = T&;
    using const_reference = const T&;
    using iterator = Iterator<false>;
    using const_iterator = Iterator<true>;
    using size_type = std::size_t;
    
    template <bool IsConst>
    class Iterator {
    public:
        using iterator_category = std::bidirectional_iterator_tag;
        
        using container_type = typename std::conditional<IsConst, const CircularBuffer, CircularBuffer>::type;
        using container_pointer_type = typename std::add_pointer<container_type>::type;
        using difference_type = typename container_type::difference_type;
        using value_type = typename container_type::value_type;
        using pointer = typename std::conditional<IsConst, const T*, T*>::type;
        using const_pointer = const T*;
        using reference = typename std::conditional<IsConst, const T&, T&>::type;
        using const_reference = const T&;
        
        Iterator()
            : container_(nullptr), it_(0) { }
        
        Iterator(container_pointer_type container, size_type it)
            : container_(container), it_(it) { }
        
        Iterator(const Iterator<IsConst>& other)
            : container_(other.container_), it_(other.it_) { }
        
        Iterator<IsConst>& operator=(const Iterator<IsConst>& other) {
            container_ = other.container_;
            it_ = other.it_;
            return *this;
        }
        
        bool operator==(const Iterator<IsConst>& other) const {
            return (container_ == other.container_) && (it_ == other.it_);
        }
        
        bool operator!=(const Iterator<IsConst>& other) const {
            return !(*this == other);
        }
    
        pointer operator->() {
            return &(this->operator*());
        }
    
        const_pointer operator->() const {
            return &(this->operator*());
        }
    
        reference operator*() {
            return container_->operator[](it_);
        }
    
        const_reference operator*() const {
            return container_->operator[](it_);
        }
        
        difference_type operator-(const Iterator<IsConst>& rhs) const {
            return it_ - rhs.it_;
        }
        
        Iterator<IsConst> operator+(size_type i) const {
            return Iterator<IsConst>(container_, it_ + i);
        }
        
        Iterator<IsConst> operator-(size_type i) const {
            return Iterator<IsConst>(container_, it_ - i);
        }
        
        Iterator<IsConst>& operator--() {
            --it_;
            return *this;
        }
        
        Iterator<IsConst>& operator++() {
            ++it_;
            return *this;
        }
        
        Iterator<IsConst> operator--(int) {
            Iterator<IsConst> tmp(container_, it_);
            operator--();
            return tmp;
        }
        
        Iterator<IsConst> operator++(int) {
            Iterator<IsConst> tmp(container_, it_);
            operator++();
            return tmp;
        }
    
    private:
        size_type it_;
        container_pointer_type container_;
    };
    
    
    CircularBuffer(int max_size)
        : max_size_(max_size), begin_it_(0), end_it_(0) {
        buffer_ = reinterpret_cast<pointer>(new char[max_size_*sizeof(value_type)]);
    }
    
    CircularBuffer(const CircularBuffer& other) {
        *this = other;
    }
    
    CircularBuffer& operator=(const CircularBuffer& other) {
        max_size_ = other.max_size_;
        buffer_ = reinterpret_cast<pointer>(new char[max_size_*sizeof(value_type)]);
        begin_it_ = other.begin_it_;
        end_it_ = other.end_it_;
        for (std::size_t i = begin_it_; begin_it_ < end_it_; ++end_it_) {
            pointer it = buffer_ + (i % max_size_);
            new(it) value_type(other[i]);
        }
        return *this;
    }
    
    reference operator[](size_type i) {
        return buffer_[(begin_it_ + i) % max_size_];
    }
    
    const_reference operator[](size_type i) const {
        return buffer_[(begin_it_ + i) % max_size_];
    }
    
    void popFront() {
        if (empty()) {
            throw std::runtime_error("Trying to delete item from empty CircularBuffer.");
        }
        buffer_[begin_it_ % max_size_].~value_type();
        begin_it_ += 1;
    }
    
    void pushBack(const_reference pose) {
        if (full()) {
            throw std::runtime_error("CircularBuffer is full. Cannot add another item.");
        }
        pointer it = buffer_ + (end_it_ % max_size_);
        new(it) value_type(pose);
        end_it_ += 1;
    }
    
    iterator begin() noexcept {
        return iterator(this, 0);
    }
    
    iterator end() noexcept {
        return iterator(this, size());
    }
    
    const_iterator begin() const noexcept {
        return const_iterator(this, 0);
    }
    
    const_iterator end() const noexcept {
        return const_iterator(this, size());
    }
    
    reference front() {
        if (empty()) {
            throw std::runtime_error("CircularBuffer is empty. Cannot return front element.");
        }
        return buffer_[begin_it_ % max_size_];
    }
    
    reference back() {
        if (empty()) {
            throw std::runtime_error("CameraPoseBuffer is empty. Cannot return back element.");
        }
        return buffer_[(end_it_ - 1) % max_size_];
    }
    
    const_reference front() const {
        if (empty()) {
            throw std::runtime_error("CircularBuffer is empty. Cannot return front element.");
        }
        return buffer_[begin_it_ % max_size_];
    }
    
    const_reference back() const {
        if (empty()) {
            throw std::runtime_error("CameraPoseBuffer is empty. Cannot return back element.");
        }
        return buffer_[(end_it_ - 1) % max_size_];
    }
    
    size_type size() const {
        return end_it_ - begin_it_;
    }
    
    bool full() const {
        return size() == max_size_;
    }
    
    bool empty() const {
        return end_it_ == begin_it_;
    }
    
    ~CircularBuffer() {
        while (!empty()) {
            popFront();
        }
        delete (char*)buffer_;
    }
private:
    size_type begin_it_;
    size_type end_it_;
    
    size_type max_size_;
    pointer buffer_;
};

}

#endif // TONAV_CIRCULAR_BUFFER_HPP