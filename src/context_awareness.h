/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ContextAwareSystem.h
 * Author: connect
 *
 * Created on 01 February 2017, 10:04
 */

#include "ScenarioDescriptor.hpp"
#include <mutex>
#include <atomic>
#include <memory>
#include <algorithm>

#ifndef CONTEXTAWARESYSTEM_H
#define CONTEXTAWARESYSTEM_H

// This will be information that is not in the Scenario Description
// For instance, the actual utilized channels
class ScenarioStats
{
public:
    void set_occupied_channels(const std::vector<int8_t>& v)
    {
        std::lock_guard<std::mutex> lk(mut);
//        if(env->scenario_list[scenario_idx.load()].n_visited_channels!=v.size())
//        {
//            std::cout << "ERROR: Number of channels does not match scenario" << std::endl;
//        }
        ch_vec = v;
    }
    std::vector<int8_t> occupied_channels()
    {
        std::lock_guard<std::mutex> lk(mut);
        return ch_vec;
    }
private:
    
    std::vector<int8_t> ch_vec;
    std::mutex mut;
};

class SituationalAwarenessApi
{   
public:
    
    int PU_scenario_idx() const {return scenario_idx.load();}
    void set_PU_scenario(int s) 
    {
        assert(s < env->scenario_list.size() && s >= -1);
        scenario_idx = (int8_t)s;
    }
    const RFEnvironmentData* get_environment()
    { // need to lock?
        return env;
    }
    void set_environment(RFEnvironmentData& e) {env = &e;}
    
    ScenarioStats stats;
private:
    RFEnvironmentData* env;   // this will only change at the beginning
    std::atomic<int8_t> scenario_idx;
};

namespace context_utils
{   
std::unique_ptr<RFEnvironmentData> make_rf_environment();
void launch_mock_scenario_update_thread(SituationalAwarenessApi* scenario_api);

inline bool is_channel_visited(const std::vector<int8_t>& occupied_channels, int8_t ch)
{
    auto it = std::find(occupied_channels.begin(), occupied_channels.end(), ch);
    return it == occupied_channels.end();
}

inline std::vector<int8_t> find_free_channels(const std::vector<int8_t>& occupied_channels, int8_t num_channels = 4)
{
    std::vector<int8_t> possible_channels(num_channels); 
    std::iota(possible_channels.begin(), possible_channels.end(), 0);
    
    auto new_end_it = std::remove_if(possible_channels.begin(), possible_channels.end(), [&](int8_t ch_val)
    {
        return context_utils::is_channel_visited(occupied_channels, ch_val);
    });
    possible_channels.erase(new_end_it, possible_channels.end());
    
    return possible_channels;
}

std::vector<int8_t> find_free_channels(const SituationalAwarenessApi& api);
        
//        auto new_end_it = std::remove_if(possible_channels.begin(), possible_channels.end(), [&](uint8_t val)
//        {
//            return std::find(context.scenario_list.begin(), context.scenario_list.end(), [&](const ScenarioDescriptor& s)
//            {
//                return scenario_utils::is_channel_visited(s,val);
//            }) != context.scenario_list.end();
//        });
};

#endif /* CONTEXTAWARESYSTEM_H */

