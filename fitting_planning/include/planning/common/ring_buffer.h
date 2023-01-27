#ifndef PLANNING_COMMON_RING_BUFFER_H_
#define PLANNING_COMMON_RING_BUFFER_H_

#include <list>
/**
 * @namespace common
 */
namespace common {

/**
 * @class RingBuffer
 *
 * @brief Circular buffer class
 */
template<typename T>
class RingBuffer {
public:
    RingBuffer(int max_buffer_size) :
        kMaxBufferSize(max_buffer_size),
        buffer_(new T[max_buffer_size]) {
        clear();
    }

    ~RingBuffer() {
        delete[] buffer_;
    }

    inline int size() const {
        return size_;
    }

    inline int max_size() const {
        return kMaxBufferSize;
    }

    inline bool empty() const {
        return size_ == 0;
    }

    inline bool full() const {
        return size_ == kMaxBufferSize;
    }

    inline T& front() const {
        return buffer_[front_];
    }

    inline T& back() const {
        return buffer_[back_];
    }

    inline void clear() {
        size_ = 0;
        front_ = 0;
        back_ = kMaxBufferSize - 1;
    }

    inline void push(const T& x) {
        back_ = (back_ + 1) % kMaxBufferSize;
        if(size() == kMaxBufferSize) {
            front_ = (front_ + 1) % kMaxBufferSize;
        }
        else {
            size_++;
        }
        back() = x;
    }

    inline void pop() {
        if(size_ > 0) {
            size_--;
            front_ = (front_ + 1) % kMaxBufferSize;
        }
    }

    inline void back_erase(int n) {
        if(n >= size_) {
            clear();
        }
        else {
            size_ -= n;
            back_ = (front_ + size_ - 1) % kMaxBufferSize;
        }
    }

    inline void front_erase(int n) {
        if(n >= size_) {
            clear();
        }
        else {
            size_ -= n;
            front_ = (front_ + n) % kMaxBufferSize;
        }
    }

private:
    template<typename T1> friend class RingBufferIterator;

    inline int front_index() const {
        return front_;
    }

    inline int back_index() const {
        return back_;
    }

    inline const T* buffer() const {
        return buffer_;
    }

    T* buffer_;
    int size_;
    int front_;
    int back_;
    const int kMaxBufferSize;
};

/**
 * @class RingBufferIterator
 *
 * @brief Iterator used for RingBuffer
 */
template<typename T>
class RingBufferIterator {
public:
    RingBufferIterator(const RingBuffer<T>& ring_buffer, bool is_from_front) :
        ring_buffer_(ring_buffer),
        index_(ring_buffer_.front_index()) {
        if(!is_from_front) {
            index_ = ring_buffer_.back_index();
        }
    }

    inline T* move_forward() {
        if(index_ == ring_buffer_.back_index()) {
            return NULL;
        }
        ++index_;
        if(index_ >= ring_buffer_.max_size()) {
            index_ -= ring_buffer_.max_size();
        }
        return &(ring_buffer_.buffer()[index_]);
    }

    inline T* move_backward() {
        if(index_ == ring_buffer_.front_index()) {
            return NULL;
        }
        --index_;
        if(index_ < 0) {
            index_ += ring_buffer_.max_size();
        }
        return &(ring_buffer_.buffer()[index_]);
    }

    inline const T* value() const {
        if(0 == ring_buffer_.size()) {
            return NULL;
        }
        return &(ring_buffer_.buffer()[index_]);
    }

private:
    const RingBuffer<T>& ring_buffer_;
    int index_;
};

}

#endif // PLANNING_COMMON_RING_BUFFER_H_
