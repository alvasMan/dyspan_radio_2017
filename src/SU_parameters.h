/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SU_parameters.h
 * Author: connect
 *
 * Created on 17 February 2017, 10:03
 */
#include <atomic>

#ifndef SU_PARAMETERS_H
#define SU_PARAMETERS_H

// NOTE: You may add other parameters like tx power if it needs to be communicated
// across threads

class SU_tx_params
{
public:
    SU_tx_params() : channel_idx(-1), has_started_(false) 
    {
        timestamp_ = 0;
    }
    inline void set_channel(short ch)
    {
        channel_idx = ch;
    }
    inline short channel()
    {
        return (has_started_==true) ? channel_idx.load() : -1;
    }
    inline void set_gain(int g)
    {
        gain_ = g;
    }
    inline int gain()
    {
        return gain_.load();
    }
    inline void start()
    {
        has_started_ = true;
    }
    inline void set_clock(time_format t)
    {
        timestamp_ = t;
    }
    inline time_format get_clock()
    {
        return timestamp_.load();
    }
private:
    std::atomic<short> channel_idx; //0-4
    std::atomic<int> gain_; //0-4
    std::atomic<time_format> timestamp_;
    bool has_started_;
};

#endif /* SU_PARAMETERS_H */

