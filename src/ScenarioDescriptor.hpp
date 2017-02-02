/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   scenario_descriptor.hpp
 * Author: connect
 *
 * Created on 31 January 2017, 11:32
 */

#include <boost/cstdint.hpp>
#include <vector>
#include <iostream>
#include <cassert>
#include <numeric>
#include <memory>

#ifndef SCENARIODESCRIPTOR_HPP
#define SCENARIODESCRIPTOR_HPP

class ScenarioDescriptor
{
public:
    int8_t channel_width;                   ///< channel bandwidth from [1,4]
    int8_t n_visited_channels;              ///< Number of channels the PU hops in
//    int16_t packet_len;                     ///< Packet length in Bytes
    int8_t packet_delay;                     ///< Inter-arrival Time in ms
    bool poisson_flag;
        
//    inline int16_t packet_Bytes() const {return packet_len;}
//    inline void set_packet_Bytes(int16_t val) 
//    {
//        assert(val==800 || val==8000);
//        packet_len = val;
//    }
    
//    std::vector<int8_t> ch_hop_idxs;       ///< Channel hopping indeces
};


// This is a read-only Structure
class RFEnvironmentData
{
public:
    RFEnvironmentData(int n, float ch_bw, const std::vector<float>& l, const std::vector<ScenarioDescriptor>& scenarios) 
        : num_channels(n), channel_bw_MHz(ch_bw), delay_ms_list(l), scenario_list(scenarios) {}
    
//    // only getters. You are not supposed to change this after setting it up
//    int8_t num_channels() const {return Nch;}
//    float channel_bandwidth_MHz() const {return channel_bw_MHz;}
//    float delay_ms(int i) const {return delay_ms_list[i];}
//    
//    const ScenarioDescriptor* scenario(int i) const {return &scenario_list[i];}

    // possible values for the scenarios
    const int8_t num_channels = 4;
    const float channel_bw_MHz = 2.5;
    const std::vector<float> delay_ms_list = {0,5,10};
    const std::vector<ScenarioDescriptor> scenario_list;
    
    //float time_granularity;    
};



//inline float packet_duration(const ScenarioDescriptor& sd)
//{
//    std::cout << "You have to implement this" << std::endl;
//    throw "";
//    return sd.packet_Bytes()/1000;
//}
//
//inline float bandwidth_MHz(const ScenarioDescriptor& sd)
//{
//    return sd.channel_width()*2.5;
//}
//
//inline float bw_MHz_to_channel_width(const RFEnvironmentData& p, float bw)
//{
//    return std::round(bw/p.channel_bandwidth);
//}
//
//inline float inter_arrival_time_sec(const ScenarioDescriptor& sd)
//{
//    return sd.IaT_msec()/1000;
//}


#endif /* SCENARIODESCRIPTOR_HPP */

