/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   channel_hopper.hpp
 * Author: connect
 *
 * Created on 22 February 2017, 18:55
 */

#include "context_awareness.h"
#include "SU_parameters.h"
#include <chrono>

#ifndef CHANNEL_HOPPER_HPP
#define CHANNEL_HOPPER_HPP

class SimpleChannelHopper
{
public:
    int current_channel = -1;
    std::chrono::system_clock::time_point tchange;
    int current_scenario_expanded = -1;
    std::vector<int> current_free_channels;
    int max_channel_stay_ms = 500;
    int min_channel_return_ms = 100;
    int last_channel = -1;
    SituationalAwarenessApi* pu_api;
    SU_tx_params* su_api;
    
    SimpleChannelHopper() = default;
    SimpleChannelHopper(SituationalAwarenessApi& scen_api, SU_tx_params& sutx_api)
    {
        setup(scen_api, sutx_api);
    }
    
    void setup(SituationalAwarenessApi& scen_api, SU_tx_params& sutx_api)
    {
        pu_api = &scen_api;
        su_api = &sutx_api;
        set_channel(0);
    }
    
    void set_channel(int ch)
    {
        assert(ch >= 0 and ch < 4);
        if(ch == current_channel)
            return; // do nothing
        else
        {
            last_channel = current_channel;
            current_channel = ch;
            tchange = std::chrono::system_clock::now();
        }
    }
    
    void work();
};

#endif /* CHANNEL_HOPPER_HPP */

