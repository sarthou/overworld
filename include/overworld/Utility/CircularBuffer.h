#ifndef OWDS_CIRCULARBUFFER_H
#define OWDS_CIRCULARBUFFER_H

#include <array>

namespace owds {
template <typename T, std::size_t capacity> class CircularBuffer
{
  public:
    inline CircularBuffer() : firstIndex_(0), lastIndex_(0), size_(0) {}

    inline const T& at(std::size_t i) const
    {
        if (i >= size_)
        {
            throw std::out_of_range("Circular buffer out of range error. Asked element at place '" + std::to_string(i) + "' but size is '" +
                                    std::to_string(size_) + "'.");
        }
        return buffer_[(firstIndex_ + i) % capacity];
    }

    inline const T& back() const
    {
        if (size_ == 0)
        {
            throw std::out_of_range("Called 'back' function on an empty buffer");
        }
        return buffer_[(lastIndex_ + capacity - 1) % capacity];
    }

    inline void push_back(const T& element)
    {
        if (size_ == capacity)
        {
            // Buffer full, overwrite the first element
            assert(lastIndex_ == firstIndex_);
            firstIndex_ = (firstIndex_ + 1) % capacity;
        }
        else
        {
            size_++;
        }
        buffer_[lastIndex_] = element;
        lastIndex_ = (lastIndex_ + 1) % capacity;
    }

    inline size_t size() const { return size_; }

    inline void empty()
    {
        lastIndex_ = firstIndex_ = 0;
        size_ = 0;
        (void)buffer_.empty();
    }

  protected:
    std::size_t firstIndex_;
    std::size_t lastIndex_;
    std::size_t size_;
    std::array<T, capacity> buffer_;
};
} // namespace owds
#endif /* OWDS_CIRCULARBUFFER_H */
