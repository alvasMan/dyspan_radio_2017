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


#ifndef STATS_H
#define	STATS_H

#include <algorithm>

struct val_stats {
    double val_sum;
    uint32_t val_count;
    val_stats() {
        reset();
    }
    inline void push(float val) {
        val_sum += val;
        ++val_count;
    }
    inline float get_avg() {
        return val_sum / val_count;
    }
    inline void reset() {
        val_sum = 0;
        val_count = 0;
    }
};

struct rate_stats {
    uint32_t val_sum;
    uint32_t val_count;
    rate_stats() {
        reset();
    }
    inline void reset() {
        val_sum = 0;
        val_count = 0;
    }
    inline void hit() {
        ++val_sum;
        ++val_count;
    }
    inline void miss() {
        ++val_count;
    }
    inline float get_rate() {
        return ((float)val_sum) / val_count;
    }
};

struct exp_stats {
    double val_sum;
    double alpha;
    uint32_t val_count;
    
    exp_stats()
    {
        alpha = 0.05;
        reset();
    }
    inline void push(float val) {
        if(val_count == 0)
            val_sum = val;
        else
            val_sum = (1-alpha)*val_sum + alpha*val;
        ++val_count;
    }
    inline float get_avg() {
        return val_sum;
    }
    inline void reset() {
        val_sum = 0;
        val_count = 0;
    }
};

template <typename T>
class CircularBuffer {
    std::vector<T> buf;
    uint32_t idx;
    uint32_t cur_size;

public:
    CircularBuffer() : idx(0) {}
    CircularBuffer(uint32_t size) {set_size(size);}
    inline void set_size(uint32_t size) {buf.resize(size); cur_size = 0; idx = 0;}
    inline uint32_t size() {
        return cur_size;
    }
    inline void push(const T &val) {
        buf[idx++] = val;
        if(idx >= buf.size()) idx = 0;
        if(cur_size < buf.size()) ++cur_size;
    }
    inline const T& get_val(uint32_t pos) {
        pos = (pos+idx) % size();
        return buf[pos];
    }
};

template <typename T>
class MovingAverage 
{
    std::vector<T> buf;
    T pwr_sum = 0;
    uint32_t idx = 0;
    uint16_t count_refresh = 0;
    uint16_t refresh_period = 5;

public:
    MovingAverage() = default;//delete;
    MovingAverage(uint32_t size) : buf(size,0), refresh_period(3*size) 
    {
    }
    MovingAverage(uint32_t size, uint32_t refresh_dur) : buf(size,0), refresh_period(refresh_dur) 
    {
    }
    inline void set_size(uint32_t size) {buf.resize(size, 0); count_refresh = 0; refresh_period = 3 * buf.size();}
    inline uint32_t size() const {return buf.size();}
    inline T push(const T &val) 
    {
        T ret = buf[idx];
        pwr_sum += val - buf[idx];
        buf[idx++] = val;
        if(idx >= buf.size())
            idx = 0;
        if(count_refresh++ >= refresh_period)
            refresh();
        return ret;
    }
    inline double get_avg() const { return pwr_sum / (double)size(); }
    inline void refresh() 
    {
        pwr_sum = std::accumulate(buf.begin(), buf.end(), 0.0);
        count_refresh = 0;
    }
    inline std::vector<T> data() const
    {
        std::vector<T> d(buf.size());
        for(int i = 0; i < d.size(); ++i)
            d[i] = buf[(idx+i)%buf.size()];
        return d;
    }
};

class MovingWindowMax {
public:
    MovingWindowMax(uint32_t size) {resize(size);}
    inline void resize(uint32_t size) {buf.resize(size, 0); max_val_idx = 0; idx = 0;}
    inline uint32_t size() {return buf.size();}
    inline void push(double val) 
    {
        buf[idx] = val;
        if(buf[max_val_idx] < val)
            max_val_idx = idx;
        if(++idx >= size()) idx = 0;
    }
    inline double max()
    {
        return buf[max_val_idx];
    }
private:
    std::vector<double> buf;
    uint32_t idx;
    uint32_t max_val_idx;
};

#endif	/* STATS_H */
