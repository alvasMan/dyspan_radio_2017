/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2015  Andre Puschmann, Francisco Paisana, Justin Tallon
 *
 * \section LICENSE
 *
 * This file is part of dyspanradio.
 *
 * dyspanradio is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * dyspanradio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#ifndef BUFFER_FACTORY_H
#define BUFFER_FACTORY_H

#include "dyspanradio.h"
#include <boost/thread/mutex.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <vector>

#define MAX_SPP 4096 //< Maximum samples per packet

// This class is inspired by Kurt Neufeld

struct BufferItem
{
    BufferItem() : data(MAX_SPP), len(0) {}
    CplxFVec data;
    int len;                     //< This is the amont of space in use (the actual size is always block_size_)
};

typedef boost::shared_ptr<BufferItem> ItemPtr;

template<typename T>
class BufferFactory : boost::noncopyable
{
public:
    typedef BufferFactory<T> buffer_type;
    typedef std::list<T*> list;

    BufferFactory(size_t initial_size = 10, size_t grow_by = 1)
    {
        grow_by_ = grow_by;
        make(initial_size);
    }

    virtual ~BufferFactory()
    {
        free_pool();
    }

    ItemPtr get_new(void)
    {
        ItemPtr ptr = create();
        assert(ptr != nullptr);
        return ptr;
    }

private:
    void make(size_t n)
    {
        for (unsigned i=0; i < n; i++)
        {
            pool_.push_back(new T());
        }
    }

    ItemPtr create()
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (pool_.empty())
            make(grow_by_);

        T* p = pool_.back();
        pool_.pop_back();

        // magic is here, custom "deleter" puts raw pointer back in pool
        return ItemPtr(p, boost::bind(&buffer_type::release, this, _1) );
    }

    void release(T* p)
    {
        boost::mutex::scoped_lock lock(mutex_);
        pool_.push_back(p);
    }

    void free_pool()
    {
        boost::mutex::scoped_lock lock(mutex_);
        for (typename list::iterator it = pool_.begin(); it != pool_.end(); ++it) {
            delete *it;
        }
        pool_.clear();
    }

    mutable boost::mutex mutex_;
    list pool_;
    size_t grow_by_;

    // not used ..
    size_t block_size_;
    size_t max_num_blocks_;
};

#endif // BUFFER_FACTORY_H

