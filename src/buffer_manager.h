#ifndef BUFFER_MANAGER_H
#define BUFFER_MANAGER_H

#include <boost/thread/mutex.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>

typedef struct
{
    boost::shared_ptr<std::complex<float>[] > buffer;
    size_t len;                     //< This is the amont of space in use (the actual size is always block_size_)
} BufferElement;


class BufferFactory : boost::noncopyable
{
public:
    explicit BufferFactory(size_t block_size, size_t max_num_blocks) :
        block_size_(block_size),
        max_num_blocks_(max_num_blocks) {}

    BufferElement get_new_element(void)
    {
        boost::mutex::scoped_lock lock(mutex_);

        BufferElement elem;
        const size_t num_floats = block_size_ / sizeof(std::complex<float>);
        elem.buffer = boost::make_shared<std::complex<float>[] >(num_floats,0.0);
        assert(elem.buffer != nullptr);
        elem.len = 0;
        return elem;
    }

private:
    mutable boost::mutex mutex_;
    size_t block_size_;
    size_t max_num_blocks_;
};

#endif // BUFFER_MANAGER_H

