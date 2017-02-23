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
    SituationalAwarenessApi* pu_api;
    
    SimpleChannelHopper() = default;
    SimpleChannelHopper(SituationalAwarenessApi& scen_api)
    {
        setup(scen_api);
    }
    
    void setup(SituationalAwarenessApi& scen_api)
    {
        pu_api = &scen_api;
        set_channel(0);
    }
    
    void set_channel(int ch)
    {
        if(ch == current_channel)
            return; // do nothing
        else
        {
            current_channel = ch;
            tchange = std::chrono::system_clock::now();
        }
    }
    
    void work();
};

#endif /* CHANNEL_HOPPER_HPP */

