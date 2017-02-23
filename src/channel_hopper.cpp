/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "channel_hopper.hpp"
#include "general_utils.hpp"

void SimpleChannelHopper::work()
{
    auto scenario_expanded = pu_api->PU_expanded_scenario_idx();

    if(scenario_expanded != current_scenario_expanded)
    {
        current_free_channels = context_utils::find_free_channels(*pu_api);
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
            auto t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-tchange).count();
            if(t_elapsed > max_channel_stay_ms)
            {
                std::cout << "Free channels: " << print_range(current_free_channels) << std::endl;
                auto it = current_free_channels.begin();
                while(it == found_ch_it || (*it == last_channel && t_elapsed < min_channel_return_ms))
                    ++it;
                assert(it != found_ch_it);
                // TODO: Select random
                set_channel(*it);
            }
            // do nothing if not enough time has passed
        }
        // if there is no free channel, do nothing (or change to last detection)
    }
}