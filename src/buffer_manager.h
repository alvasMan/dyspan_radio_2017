#ifndef BUFFER_MANAGER_H
#define BUFFER_MANAGER_H

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/call_traits.hpp>
#include <queue>

#include <boost/pool/simple_segregated_storage.hpp>

#define MAX_BUFFER_BLOCKS 64

typedef struct
{
    std::complex<float>* buffer;
    //size_t size;
    size_t len;                     //< This is the amont of space in use (the actual size is always block_size_)
} BufferElement;


class BufferFactory : boost::noncopyable
{
public:
    explicit BufferFactory(size_t block_size, size_t max_num_blocks) :
        block_size_(block_size),
        max_num_blocks_(max_num_blocks)
    {
        // assign memory to storage
        const size_t pool_size = block_size_ * max_num_blocks_;
        std::cout << "Memory pool size: " << pool_size / 8 / 1024 << " MB" << std::endl;
        v.resize(pool_size);
        storage.add_block(&v.front(), v.size(), block_size_);
    }

    void get_new_element(BufferElement &elem)
    {
        boost::mutex::scoped_lock lock(mutex_);
        elem.buffer = static_cast<std::complex<float>*>(storage.malloc_n(1, block_size_));
        assert(elem.buffer != nullptr);

        // purge memory
        elem.len = 0;
        memset(elem.buffer, 0, block_size_);
    }

    void release_element(const BufferElement &elem)
    {
        boost::mutex::scoped_lock lock(mutex_);
        storage.free_n(elem.buffer, 1, block_size_);
    }

private:
    mutable boost::mutex mutex_;
    boost::simple_segregated_storage<std::size_t> storage;
    size_t block_size_;
    size_t max_num_blocks_;
    std::vector<uint8_t> v;
};

#endif // BUFFER_MANAGER_H

