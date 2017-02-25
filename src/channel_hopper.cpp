/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "channel_hopper.hpp"
#include "general_utils.hpp"

int pick_round_robin(const std::vector<int>& vec)
{
    static int idx = 0;
    idx = idx % vec.size();
    
    int ret = vec[idx];
    idx = (idx+1) % vec.size();
    assert(ret>=0 and ret < 4);
    return ret;
}

template<typename Op>
int pick_round_robin(const std::vector<int>& vec, Op func)
{
    static int idx = 0;
    idx = idx % vec.size();
    int aux = idx;
    int ret;
    do{
        ret = vec[aux];
        aux = (aux+1) % vec.size();
        if(aux==idx)
            break; // Note: This should not happen
    } while(func(ret)==false);
    idx = aux;
    assert(ret>=0 and ret < 4);
    return ret;
}

void SimpleChannelHopper::work()
{
    auto scen_idx = pu_api->PU_expanded_scenario_idx();
    // if the scenario has changed
    bool change_ch = scen_idx != current_scenario_expanded;
    if(change_ch)
        current_free_channels = context_utils::find_free_channels(*pu_api);
    
    // if the scenario has changed but the occupancy masked hasn't, do not change channel
    if(change_ch == true && current_scenario_expanded>=0)
    {
        auto &mask1 = pu_api->expanded_scenarios.scenarios_expanded_list[scen_idx].ch_occupied_mask;
        auto &mask2 = pu_api->expanded_scenarios.scenarios_expanded_list[current_scenario_expanded].ch_occupied_mask;
        //change_ch = !std::equal(mask1.begin(), mask1.end(), mask2.begin());
        
        // only change channel if it stopped being free
        if(current_channel >= 0 && mask1[current_channel] == mask2[current_channel] && mask1[current_channel]==false)
            change_ch = false;
    }
    
    if(change_ch)
    {
        if(current_free_channels.size()>0)
        {
            // TODO: Set random channel
            set_channel(pick_round_robin(current_free_channels));//current_free_channels[0]);
        }
        // if no free channels, do nothing

        current_scenario_expanded = scen_idx;
    }
    else
    {
        if(current_free_channels.size()>0)
        {
//            auto found_ch_it = std::find(current_free_channels.begin(), current_free_channels.end(), current_channel);
//            assert(found_ch_it != current_free_channels.end());

            // If enough time has passed, change to another free channel
            auto t_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-tchange).count();
            if(t_elapsed > max_channel_stay_ms)
            {
                std::cout << "Free channels: " << print_range(current_free_channels) << std::endl;
                int ch;
                if(current_free_channels.size()>2)
                {
                    ch = pick_round_robin(current_free_channels, [&](int ch_idx){
                        return ch_idx != current_channel && (ch_idx != last_channel || t_elapsed > min_channel_return_ms);
                    });
                }
                else
                {
                    ch = pick_round_robin(current_free_channels, [&](int ch_idx){
                        return ch_idx != current_channel;
                    });
                }
//                auto it = current_free_channels.begin();
//                while(it == found_ch_it || (*it == last_channel && t_elapsed < min_channel_return_ms))
//                    ++it;
                //assert(it != found_ch_it);
                // TODO: Select random
                set_channel(ch);//*it);
            }
            // do nothing if not enough time has passed
        }
        else
        {
            int ch;
            time_format tstamp;
            std::tie(ch, tstamp) = pu_api->stats.most_recent_occupied_channel();
            if(tstamp > last_fast_hop_tstamp)
            {
                set_channel(ch);
                last_fast_hop_tstamp = tstamp;
            }
        }
        // if there is no free channel, do nothing (or change to last detection)
    }
}