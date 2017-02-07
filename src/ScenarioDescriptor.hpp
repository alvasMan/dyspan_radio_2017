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

typedef int scenario_number_type;

class ScenarioDescriptor
{
public:
    int n_occupied_channels;                     ///< Number of channels the PU hops in
    int packet_delay_idx;                        ///< Inter-arrival Time from the list
    bool channel_hopping;
    bool poisson_flag;
};


// This is a read-only Structure
class RFEnvironmentData
{
public:
    RFEnvironmentData(int n, float ch_bw, const std::vector<float>& l, const std::vector<ScenarioDescriptor>& scenarios) 
        : num_channels(n), channel_bw_MHz(ch_bw), delay_ms_list(l), scenario_list(scenarios) {}

    // possible values for the scenarios
    const int num_channels = 4;
    const float channel_bw_MHz = 2.5;
    const std::vector<float> delay_ms_list = {0,5,10};
    const std::vector<ScenarioDescriptor> scenario_list;
    
    //float time_granularity;    
};

#endif /* SCENARIODESCRIPTOR_HPP */

