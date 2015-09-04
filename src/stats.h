/* 
 * File:   utilities.h
 * Author: ctvr
 *
 * Created on August 31, 2015, 5:45 PM
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
class MovingAverage {
    std::vector<T> buf;
    T pwr_sum;
    uint32_t idx;
    uint16_t count_refresh, refresh_period;

public:
    MovingAverage() {pwr_sum = 0; idx = 0;}
    MovingAverage(uint32_t size) {set_size(size);}
    inline void set_size(uint32_t size) {buf.resize(size, 0); pwr_sum = 0; idx = 0; count_refresh = 0; refresh_period = 4 * buf.size();}
    inline uint32_t size() {return buf.size();}
    inline void push(const T &val) {
        pwr_sum += val - buf[idx];
        buf[idx++] = val;
        if(count_refresh++ >= refresh_period)
            refresh();
        if(idx >= size()) idx = 0;
    }
    inline T get_avg() { return pwr_sum / (float)size(); }
    inline T get_refreshed_avg() {
        return std::accumulate(buf.begin(), buf.end(), 0.0) / (float)size();
    }
    inline void refresh() {
        T old_pwr_sum = pwr_sum;
        pwr_sum = std::accumulate(buf.begin(), buf.end(), 0.0);
        T test = std::abs(old_pwr_sum - pwr_sum)/pwr_sum;
        if(test < 0.001) refresh_period += buf.size();
        else if(test > 0.1)  refresh_period = buf.size();
        count_refresh = 0;
    }
};

#endif	/* STATS_H */
