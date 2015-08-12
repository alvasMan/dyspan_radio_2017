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




template<class T>
class BlockBuffer : boost::noncopyable
{
public:
    typedef std::queue<T> container_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;
    typedef typename boost::call_traits<value_type>::param_type param_type;

    explicit BlockBuffer(size_t block_size, size_t max_num_blocks) :
        block_size_(block_size),
        max_num_blocks_(max_num_blocks),
        capacity_(10)
    {
        // assign memory to storage
        const size_t pool_size = block_size_ * max_num_blocks_;
        std::cout << "Memory pool size: " << pool_size / 8 / 1024 << " MB" << std::endl;
        v.resize(pool_size);
        storage.add_block(&v.front(), v.size(), block_size_);
    }
    //explicit BlockBuffer() : capacity_(10) {}

#if 0
    void initialize(const size_t buffer_block_size)
    {

    }
#endif

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


    void pushBack(T const& data)
    {
        boost::mutex::scoped_lock lock(mutex_);
        while (container_.size() >= capacity_) {
           notFullCond_.wait(lock);
        }
        container_.push(data);
        lock.unlock();
        notEmptyCond_.notify_one();
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(mutex_);
        return container_.empty();
    }

    bool tryPop(T& value)
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (container_.empty()) {
            return false;
        }
        value = container_.front();
        container_.pop();
        lock.unlock();
        notFullCond_.notify_one();
        return true;
    }

    void popFront(T& value)
    {
        boost::mutex::scoped_lock lock(mutex_);
        while (container_.empty()) {
            notEmptyCond_.wait(lock);
        }
        value = container_.front();
        container_.pop();
        lock.unlock();
        notFullCond_.notify_one();
    }

    size_type size() {
        boost::mutex::scoped_lock lock(mutex_);
        return container_.size();
    }

    size_type capacity() {
        boost::mutex::scoped_lock lock(mutex_);
        return capacity_;
    }

    bool isEmpty() {
        boost::mutex::scoped_lock lock(mutex_);
        return container_.empty();
    }

private:
    container_type container_;
    size_type capacity_;
    mutable boost::mutex mutex_;
    boost::condition_variable notEmptyCond_;
    boost::condition_variable notFullCond_;

    boost::simple_segregated_storage<std::size_t> storage;
    size_t block_size_;
    size_t max_num_blocks_;
    std::vector<uint8_t> v;
};

#endif // BUFFER_MANAGER_H

