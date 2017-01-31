/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2015  Francisco Paisana, Andre Puschmann, Justin Tallon
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

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/call_traits.hpp>
#include <boost/bind.hpp>
#include <boost/move/move.hpp>

#ifndef BUFFER_UTILS_HPP
#define BUFFER_UTILS_HPP

namespace buffer_utils
{

template<class T>
class rdataset;

template<class T>
class wdataset;

// THERE IS SOME BUG WITH THIS BUFFER

template<class T>
class bounded_buffer
{
public:

    bounded_buffer(int size) : buf(size), n_written(0), w_idx(-1), r_idx(-1)
    {
    }
    
    bounded_buffer(int size, const T& init_val) : buf(size,init_val), n_written(0), w_idx(-1), r_idx(-1)
    {
    }

    void get_rdataset(rdataset<T>& ret)
    {
        boost::mutex::scoped_lock lock(m_mutex);
        while (n_written == 0)
            m_not_empty.wait(lock);
        r_idx = (r_idx + 1) % buf.size();
        ret.set(this, buf[r_idx]);
    }

    bool try_get_rdataset(rdataset<T>& ret)
    {
        boost::mutex::scoped_lock lock(m_mutex);
        if(n_written == 0)
            return false;
        r_idx = (r_idx + 1) % buf.size();
        ret.set(this, buf[r_idx]);
        return true;
    }
    
    void get_wdataset(wdataset<T>& ret)
    {
        boost::mutex::scoped_lock lock(m_mutex);
        while (n_written == buf.size())
            m_not_full.wait(lock);
        w_idx = (w_idx + 1) % buf.size();
        ret.set(this, buf[w_idx]);
    }

    bool try_get_wdataset(wdataset<T>& ret)
    {
        boost::mutex::scoped_lock lock(m_mutex);
        bool flag = true;
        if (n_written == buf.size())
        {
            std::cout << "O"; // we write in the previous
            n_written--;
            flag = false;
        }
        else
            w_idx = (w_idx + 1) % buf.size();
        ret.set(this, buf[w_idx]);
        //ret()->reserve((ret().capacity() > 16) ? ret.data->capacity() : 16);
        return flag;
    }

    void release_wdataset()
    {
        boost::mutex::scoped_lock lock(m_mutex);
        n_written++;
        m_not_empty.notify_one();
    }

    void release_rdataset()
    {
        boost::mutex::scoped_lock lock(m_mutex);
        n_written--;
        m_not_full.notify_one();
    }

    bool empty()
    {
        return n_written == 0;
    }

private:
    std::vector<T> buf;
    boost::mutex m_mutex;
    boost::condition m_not_full;
    boost::condition m_not_empty;
    int n_written;
    int w_idx;
    int r_idx;
};

template<class T>
class wdataset
{
    friend class bounded_buffer<T>;
public:
    wdataset() : buf(NULL), data(NULL)
    {
    }

    wdataset(bounded_buffer<T>* b, T& t) : buf(b), data(&t)
    {
    }

    inline void release()   // explicit release
    {
        if (buf != NULL)
            buf->release_wdataset();
        buf = NULL;
        data = NULL;
    }

    inline T& operator()()
    {
        return *data;
    }

    ~wdataset()
    {
        if (buf != NULL)
            buf->release_wdataset();
    }
protected:
    inline void set(bounded_buffer<T>* b, T& t) // only called by the buffer
    {
        if(buf != NULL)
            buf->release_rdataset();
        buf = b;
        data = &t;
    }
private:
    wdataset(wdataset<T>& w); // non-copyable
    wdataset<T>& operator=(wdataset<T>& w); // non-copyable
    bounded_buffer<T> *buf;
    T* data;
};

template<class T>
class rdataset
{
    friend class bounded_buffer<T>;
public:

    rdataset() : buf(NULL), data(NULL)
    {
    }

    rdataset(bounded_buffer<T>* b, T& t) : buf(b), data(&t)
    {
    }

    inline T& operator()()
    {
        return *data;
    }
    
    inline void release()   // explicit release
    {
        if (buf != NULL)
            buf->release_rdataset();
        buf = NULL;
        data = NULL;
    }

    ~rdataset()
    {
        if (buf != NULL)
            buf->release_rdataset();
    }
protected:
    void set(bounded_buffer<T>* b, T& t)
    {
        if(buf != NULL)
            buf->release_rdataset();
        buf = b;
        data = &t;
    }
private:
    rdataset(wdataset<T>& w);
    rdataset<T>& operator=(wdataset<T>& w);

    bounded_buffer<T> *buf;
    T* data;
};

}

#endif /* BUFFER_UTILS_HPP */

