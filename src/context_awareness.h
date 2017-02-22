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

class ExpandedScenarioDescriptor
{
public:
    scenario_number_type scenario_idx;
    const ScenarioDescriptor* scenario;
    std::vector<bool> ch_occupied_mask;
};

class ScenariosExpanded
{
public:
    ScenariosExpanded(RFEnvironmentData* rf_env) : rf_environment(rf_env)
    {
        expand_scenarios();
    }
    void expand_scenarios();
    const RFEnvironmentData* rf_environment;
    std::vector<ExpandedScenarioDescriptor> scenarios_expanded_list;
};

// This will be information that is not in the Scenario Description
// For instance, the actual utilized channels
class ScenarioStats
{
public:
    void set_occupied_channels(const std::vector<int>& v)
    {
        std::lock_guard<std::mutex> lk(mut);
//        if(env->scenario_list[scenario_idx.load()].n_visited_channels!=v.size())
//        {
//            std::cout << "ERROR: Number of channels does not match scenario" << std::endl;
//        }
        ch_vec = v;
    }
    std::vector<int> occupied_channels()
    {
        std::lock_guard<std::mutex> lk(mut);
        return ch_vec;
    }
private:
    
    std::vector<int> ch_vec;
    std::mutex mut;
};

class SituationalAwarenessApi
{   
public:
    SituationalAwarenessApi(RFEnvironmentData& e) : environment_data(&e), expanded_scenarios(&e)
    {
    }
    int PU_scenario_idx() const {return scenario_idx.load();}
    int PU_expanded_scenario_idx() const {return expanded_scenario_idx.load();}
    void set_PU_scenario(scenario_number_type s)
    {
        assert(s < environment_data->scenario_list.size() && s >= -1);
        scenario_idx = s;
    }
    
    const RFEnvironmentData* environment_data;   // this will only change at the beginning
    const ScenariosExpanded expanded_scenarios;
    ScenarioStats stats;
private:
    std::atomic<scenario_number_type> scenario_idx;
    std::atomic<scenario_number_type> expanded_scenario_idx;
};

namespace context_utils
{   
std::unique_ptr<RFEnvironmentData> make_rf_environment();
void launch_mock_scenario_update_thread(SituationalAwarenessApi* scenario_api);

inline bool is_channel_visited(const std::vector<int>& occupied_channels, int ch)
{
    auto it = std::find(occupied_channels.begin(), occupied_channels.end(), ch);
    return it == occupied_channels.end();
}

inline std::vector<int> find_free_channels(const std::vector<int>& occupied_channels, int num_channels = 4)
{
    std::vector<int> possible_channels(num_channels); 
    std::iota(possible_channels.begin(), possible_channels.end(), 0);
    
    auto new_end_it = std::remove_if(possible_channels.begin(), possible_channels.end(), [&](int8_t ch_val)
    {
        return context_utils::is_channel_visited(occupied_channels, ch_val);
    });
    possible_channels.erase(new_end_it, possible_channels.end());
    
    return possible_channels;
}

std::vector<int> find_free_channels(const SituationalAwarenessApi& api);

};

#endif /* CONTEXTAWARESYSTEM_H */

