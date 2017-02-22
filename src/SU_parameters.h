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
    SU_tx_params() : channel_idx(-1) {}
    inline void set_channel(short ch)
    {
        channel_idx = ch;
    }
    inline short channel()
    {
        return channel_idx.load();
    }
    inline void set_gain(int g)
    {
        gain_ = g;
    }
    inline int gain()
    {
        return gain_.load();
    }
private:
    std::atomic<short> channel_idx; //0-4
    std::atomic<int> gain_; //0-4
};

#endif /* SU_PARAMETERS_H */

