#ifndef BUFFER_FACTORY_H
#define BUFFER_FACTORY_H

#include "dyspanradio.h"
#include <boost/thread/mutex.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <vector>

#define NUM_ELEMENTS 2000

struct BufferItem
{
    BufferItem() : data(NUM_ELEMENTS), len(0) {}
    CplxFVec data;
    size_t len;                     //< This is the amont of space in use (the actual size is always block_size_)
};

typedef boost::shared_ptr<BufferItem> ItemPtr;

// The default fast_pool_allocator seems to just work fine
#if 1
typedef boost::fast_pool_allocator<BufferItem> ItemPool;
#else
typedef boost::fast_pool_allocator<BufferElement,
                                    boost::default_user_allocator_new_delete,
                                    boost::details::pool::default_mutex,
                                    2 * NUM_ELEMENTS * sizeof(std::complex<float>), // NextSize
                                    5500000> ItemPool;
#endif


class BufferFactory : boost::noncopyable
{
public:
    explicit BufferFactory() {}

    ItemPtr get_new(void)
    {
        boost::mutex::scoped_lock lock(mutex_);
        return ItemPtr(new (pool_.allocate()) BufferItem(), boost::bind(&ItemPool::destroy, ref(pool_), _1) );
    }

private:
    mutable boost::mutex mutex_;
    ItemPool pool_;
    // not used ..
    size_t block_size_;
    size_t max_num_blocks_;
};

#endif // BUFFER_FACTORY_H

