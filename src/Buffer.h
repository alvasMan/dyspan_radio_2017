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


// This class is used from Iris' buffer class

#ifndef BUFFER_H
#define BUFFER_H

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/call_traits.hpp>
#include <queue>

template<class T>
class Buffer : boost::noncopyable
{
public:
    typedef std::queue<T> container_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;
    typedef typename boost::call_traits<value_type>::param_type param_type;

    explicit Buffer(size_type capacity) : capacity_(capacity) {}
    explicit Buffer() : capacity_(10) {}

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
};


#endif // BUFFER_H
