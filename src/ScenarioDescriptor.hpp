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

#ifndef SCENARIODESCRIPTOR_HPP
#define SCENARIODESCRIPTOR_HPP

class ScenarioDescriptor
{
public:
    ScenarioDescriptor(uint8_t chw, int16_t pl, const std::vector<uint8_t>& chi, float iat, bool deterministic) :
    ch_hop_idxs(chi), ch_w(chw), packet_len(pl), inter_arrival_time(iat), deterministic_flag(deterministic) {}    
    
    inline uint8_t channel_width() const {return ch_w;}
    inline void set_channel_width(uint8_t val) 
    {
        assert(val > 0 && val <= 4);
        ch_w = val;
    }
    inline uint16_t packet_Bytes() const {return packet_len;}
    inline void set_packet_Bytes(uint16_t val) 
    {
        assert(val==800 || val==8000);
        packet_len = val;
    }
    inline float IaT_msec() const {return inter_arrival_time;}
    inline void set_IaT_msec(float val) 
    {
        assert(val >= 0);
        inter_arrival_time = val;
    }
    inline bool is_IaT_deterministic() const {return deterministic_flag;}
    void set_IaT_deterministic(bool flag = true) { deterministic_flag = flag; }
    
    std::vector<uint8_t> ch_hop_idxs;       ///< Channel hopping indeces
    
private:
    uint8_t ch_w;                           ///< channel bandwidth from [1,4]
    int16_t packet_len;                     ///< Packet length in Bytes
    float inter_arrival_time;               ///< Inter-arrival Time in ms
    bool deterministic_flag;
};

namespace scenario_utils
{

inline bool is_channel_visited(const ScenarioDescriptor& sd, uint8_t ch)
{
    auto it = std::find(sd.ch_hop_idxs.begin(), sd.ch_hop_idxs.end(), ch);
    
    return it == sd.ch_hop_idxs.end();
}

inline float packet_duration(const ScenarioDescriptor& sd)
{
    std::cout << "You have to implement this" << std::endl;
    throw "";
    return sd.packet_Bytes()/1000;
}

inline float bandwidth_MHz(const ScenarioDescriptor& sd)
{
    return sd.channel_width()*2.5;
}

inline float inter_arrival_time_sec(const ScenarioDescriptor& sd)
{
    return sd.IaT_msec()/1000;
}

};


#endif /* SCENARIODESCRIPTOR_HPP */

