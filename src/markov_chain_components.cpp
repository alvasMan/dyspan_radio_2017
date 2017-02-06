/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <algorithm>
#include <cassert>
#include "markov_chain_components.h"

using namespace std;
using namespace nlohmann;

PacketObservation extract_observation(const DetectedPacket& prev_packet, const DetectedPacket& p)
{
    return PacketObservation(get<0>(p)-get<0>(prev_packet), 0.5*(get<2>(p) + get<2>(prev_packet)));
}

void ChallengeMarkovChain::register_packet(const DetectedPacket& p)
{
    if(get<0>(prev_packet) >= 0)
    {
        int idx = get_idx(extract_observation(prev_packet, p));
        
        state_probability *= prob_emission[idx]; // probability of state transition is 1
    }
    
    prev_packet = p;
}

void ChallengeMarkovChain::register_no_packet()
{   
    state_probability *= prob_emission[prob_emission.size()-1];
}

ChallengeMarkovChain::ChallengeMarkovChain(const LearningMarkovChain& m) : 
        AbstractChannelMarkovChain(m.val_range,m.quantization_step),
        prob_emission(m.n_observations())
{
    for(int i = 0; i < m.n_observations(); ++i)
        prob_emission[i] = (float)(m.n_hits[i] / (double)m.total_packets);
}

void LearningMarkovChain::register_packet(const DetectedPacket& p)
{
    if(get<0>(prev_packet) >= 0)
    {
        int idx = get_idx(extract_observation(prev_packet, p));
    
        n_hits[idx]++;
        total_packets++;
    }
    
    prev_packet = p;
}

void LearningMarkovChain::register_no_packet()
{
    n_hits[n_hits.size()-1]++;
    total_packets++;
}

void MarkovChainLearner::work(std::vector<DetectedPacket>& q_packets)
{
    // sort by tstamp
    sort(q_packets.begin(), q_packets.end(), [](DetectedPacket& a, DetectedPacket &b){return get<0>(a) < get<0>(b);});
    
    for(auto it = q_packets.begin(); it != q_packets.end(); ++it)
        for(int i = 0; i < Nch; ++i)
            if(i == get<1>(*it))
                mc_vec[i].register_packet(*it);
            else
                mc_vec[i].register_no_packet();
}

//template<typename Range>
//void MarkovChainLearner::work(int ch_idx, const Range& q_packets)
//{
//    for(auto it = q_packets.begin(); it != q_packets.end(); ++it)
//        mc_vec[ch_idx].work(*it);
//}

CombinatorialMarkovChain::CombinatorialMarkovChain(int n_channels, int n_occupied, const std::vector<LearningMarkovChain>& mc_learned) :
    Nch(n_channels), num_occupied(n_occupied)
{
    // NOTE: mc_combinations should have the MC for both non-occupied and occupied states
    if(n_occupied == Nch || n_occupied == 0)
    {
        assert(mc_learned.size() == Nch);
        multi_hypothesis = false;
    }
    else
    {
        assert(mc_learned.size() == 2*Nch);
        multi_hypothesis = true;
    }
    
    // convert to ChallengeMarkovChain
    mc_combinations.reserve(mc_learned.size());
    for(const auto& e: mc_learned)
        mc_combinations.emplace_back(e);
    
    // make all possible combinations of channel occupancy
    std::vector<bool> bitmask(num_occupied, true); // K leading 1's
    bitmask.resize(Nch, false); // N-K trailing 0's
    do
    {
        combinations.push_back(bitmask);
    } while(prev_permutation(bitmask.begin(), bitmask.end()));
}

void CombinatorialMarkovChain::work(std::vector<DetectedPacket>& q_packets)
{
    // sort by tstamp
    sort(q_packets.begin(), q_packets.end(), [](DetectedPacket& a, DetectedPacket &b){return get<0>(a) < get<0>(b);});
    
    for(auto it = q_packets.begin(); it != q_packets.end(); ++it)
    {
        for(int i = 0; i < Nch; ++i)
        {
            if(i == get<1>(*it))
                mc_combinations[i].register_packet(*it);
            else
                mc_combinations[i].register_no_packet();
        }
        if(multi_hypothesis) // IN CASE OF MULTI HYPOTHESIS
        {
            for(int i = 0; i < Nch; ++i)
            {
                if(i == get<1>(*it))
                    mc_combinations[i + Nch].register_packet(*it);
                else
                    mc_combinations[i + Nch].register_no_packet();
            }
        }
    }
}

float combination_state_probability(std::vector<bool>& bitmask, const std::vector<ChallengeMarkovChain>& state)
{
    float prob = 1;
    for(int i = 0; i < bitmask.size(); ++i)
        prob *= bitmask[i]*state[i].state_probability + (!bitmask[i])*state[i+bitmask.size()].state_probability;
    return prob;
}

pair<float,vector<int> > CombinatorialMarkovChain::state_probability()
{
    float max_prob = 0;
    int max_idx = -1;
    for(int i = 0; i < combinations.size(); ++i)
    {
        float prob = combination_state_probability(combinations[i], mc_combinations);
        if(prob > max_prob)
        {
            max_prob = prob;
            max_idx = i;
        }
    }
    
    std::vector<int> ch_occupied(num_occupied);
    for(int j = 0; j < combinations[max_idx].size(); ++j)
        if(combinations[max_idx][j]==true)
            ch_occupied.push_back(j);
    
    return make_pair(max_prob, ch_occupied);
}

namespace markov_utils
{
json to_json(const LearningMarkovChain& m)
{
    json j = {
        {"val_range", { {"min_delay",m.val_range.first.tdelay},
                        {"max_delay",m.val_range.second.tdelay},
                        {"min_snr",m.val_range.first.snr},
                        {"max_snr",m.val_range.second.snr} }},
        {"quantization_step", { {"delay_step", m.quantization_step.tdelay},
                                {"snr_step", m.quantization_step.snr} }},
        {"n_pwr_steps", m.n_pwr_steps},
        {"n_time_steps", m.n_time_steps},
        {"n_hits", json(m.n_hits)},
        {"total_packets", m.total_packets}
    };
    
    return j;
}

LearningMarkovChain from_json(const json& j)
{
    std::pair<PacketObservation,PacketObservation> vrange;
    vrange.first.tdelay = j["val_range"]["min_delay"].get<decltype(vrange.first.tdelay)>();
    vrange.second.tdelay = j["val_range"]["max_delay"].get<decltype(vrange.second.tdelay)>();
    vrange.first.snr = j["val_range"]["min_snr"].get<decltype(vrange.first.snr)>();
    vrange.second.snr = j["val_range"]["max_snr"].get<decltype(vrange.second.snr)>();
    
    PacketObservation qstep;
    qstep.tdelay = j["quantization_step"]["delay_step"].get<decltype(qstep.tdelay)>();
    qstep.snr = j["quantization_step"]["snr_step"].get<decltype(qstep.snr)>();
    
    LearningMarkovChain mc(vrange,qstep);
    
    assert(mc.n_pwr_steps == j["n_pwr_steps"].get<decltype(mc.n_pwr_steps)>());
    assert(mc.n_time_steps = j["n_time_steps"].get<decltype(mc.n_time_steps)>());
    
    mc.n_hits = j["n_hits"].get<decltype(mc.n_hits)>();
    mc.total_packets = j["total_packets"].get<decltype(mc.total_packets)>();
    
    return mc;
}

json to_json(const MarkovChainLearner& l)
{
    json j = {
        {"Nch", l.Nch},
        {"mc_vec", json::array()}
    };
    
    for(const auto& e : l.mc_vec)
        j["mc_vec"].push_back(to_json(e));
}

};