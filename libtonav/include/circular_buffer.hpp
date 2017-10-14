#ifndef TONAV_CIRCULAR_BUFFER_HPP
#define TONAV_CIRCULAR_BUFFER_HPP

namespace tonav {

template <typename T>
class CircularBuffer {
public:
    template <bool IsConst> class Iterator;
    
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = typename std::add_pointer<value_type>::type;
    using reference = typename std::add_lvalue_reference<value_type>::type;
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
        using pointer = typename container_type::pointer;
        using reference = typename container_type::reference;
        
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
        
        reference operator*() {
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
    
    reference operator[](size_type i) {
        return buffer_[(begin_it_ + i) % max_size_];
    }
    
    const reference operator[](size_type i) const {
        return buffer_[(begin_it_ + i) % max_size_];
    }
    
    void popFront() {
        if (empty()) {
            throw std::runtime_error("Trying to delete item from empty CircularBuffer.");
        }
        begin_it_ += 1;
        buffer_[begin_it_ % max_size_].~value_type();
    }
    
    void pushBack(const reference pose) {
        if (full()) {
            throw std::runtime_error("CircularBuffer is full. Cannot add another item.");
        }
        if (end_it_ > max_size_) {
            buffer_[begin_it_ % max_size_] = pose;
        } else {
            pointer it = buffer_ + end_it_;
            new(it) value_type(pose);
        }
    }
    
    iterator begin() {
        return iterator(this, begin_it_);
    }
    
    iterator end() {
        return iterator(this, end_it_);
    }
    
    const_iterator begin() const {
        return const_iterator(this, begin_it_);
    }
    
    const_iterator end() const {
        return const_iterator(this, end_it_);
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
    
    const reference front() const {
        if (empty()) {
            throw std::runtime_error("CircularBuffer is empty. Cannot return front element.");
        }
        return buffer_[begin_it_ % max_size_];
    }
    
    const reference back() const {
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
        delete buffer_;
    }
private:
    size_type begin_it_;
    size_type end_it_;
    
    size_type max_size_;
    pointer buffer_;
};

}

#endif // TONAV_CIRCULAR_BUFFER_HPP