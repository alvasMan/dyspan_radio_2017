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
    int current_channel;
    std::chrono::system_clock::time_point tchange;
    int current_scenario_expanded = -1;
    vector<int> current_free_channels;
    int max_channel_stay_ms = 500;
    SituationalAwarenessApi* pu_api;
    
    SimpleChannelHopper(SituationalAwarenessApi& scen_api) : pu_api(&scen_api)
    {
        set_channel(0);
    }
    
    set_channel(int ch)
    {
        if(ch == current_channel)
            return; // do nothing
        else
        {
            current_channel = ch;
            tchange = std::chrono::system_clock::now();
        }
    }
    
    work()
    {
        auto scenario_expanded = pu_api->PU_expanded_scenario_idx();
        
        if(scenario_expanded != current_scenario_expanded)
        {
            current_free_channels = context_utils::find_free_channels(pu_api);
            if(current_free_channels.size()>0)
            {
                // TODO: Set random channel
                set_channel(current_free_channels[0]);
            }
            // if no free channels, do nothing
            
            current_scenario_expanded = scenario_expanded;
        }
        else
        {
            if(current_free_channels.size()>0)
            {
                auto found_ch_it = std::find(current_free_channels.begin(), current_free_channels.end(), current_channel);
                assert(found_ch_it != current_free_channels.end());
                
                // If enough time has passed, change to another free channel
                auto tnow = std::chrono::system_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(tnow-tchange).count() > max_channel_stay_ms)
                {
                    auto new_end_it = std::remove(current_free_channels.begin(), current_free_channels.end(), found_ch_it);
                    assert(current_free_channels.begin() != new_end_it);
                    // TODO: Select random
                    set_channel(current_free_channels[0]);
                }
                // do nothing if not enough time has passed
            }
            // if there is no free channel, do nothing (or change to last detection)
        }
    }
};

#endif /* CHANNEL_HOPPER_HPP */

