/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "monitor_components.h"
#include <sstream>
#include <set>
#include <algorithm>
#include <complex>

using namespace std;
using namespace nlohmann;

void ForgetfulChannelMonitor::work(const vector<float>& ch_pwrs)
{
    // make kind of a fosphor thing
    for(int i = 0; i < channel_energy.size(); ++i)
        if(ch_pwrs[i]>=0) // not SU-Tx transmitting
            channel_energy[i] = alpha*ch_pwrs[i] + (1-alpha)*channel_energy[i];
}

// returns the indexes sorted by probability of being occupied (descending)
vector<size_t> ForgetfulChannelMonitor::ch_sorted_by_energy()
{
    vector<size_t> idx(channel_energy.size());
    iota(idx.begin(), idx.end(), 0);
    
    sort(idx.begin(), idx.end(),
       [&](size_t i1, size_t i2) {return channel_energy[i1] > channel_energy[i2];});

    return idx;
}

//> Channel Packet Monitor

void ChannelPacketRateMonitor::work(const std::vector<DetectedPacket>& packets)
{
    
    for(const auto& e : packets)
    {
        int ch_idx = get<1>(e);
        if(prev_packet_tstamp[ch_idx] > 0)
        {
            time_format tdelay = get<0>(e) - prev_packet_tstamp[ch_idx];
            tdelay_acc[ch_idx].push(tdelay);
        }
        prev_packet_tstamp[ch_idx] = get<0>(e);
    }
}

json to_json(const delay_stats& x)
{
    return {
        {"count", x.count},
        {"sum", x.sum},
        {"sum2", x.sum2}
    };
}

delay_stats from_json(const json& j)
{
    delay_stats d;
    d.count = j["count"].get<long>();
    d.sum = j["sum"].get<time_format>();
    d.sum2 = j["sum2"].get<decltype(d.sum2)>();
    return d;
}

json ChannelPacketRateMonitor::to_json()
{   
    json j = {
        {"Nch", Nch},
        {"channel_stats", json::array()},
        {"channel_free", json::array()}
    };
    for(int i = 0; i < Nch; ++i)
    {
        j["channel_stats"].push_back(::to_json(tdelay_acc[i]));
        j["channel_free"].push_back(::to_json(tdelay_acc_free[i]));
    }
    return j;
}


void ChannelPacketRateMonitor::from_json(json& j, vector<int> ch_occupancy)
{
    if(j.empty())
        return;
    Nch = j["Nch"].get<decltype(Nch)>();
    
    
    tdelay_acc_free.resize(Nch);
    tdelay_acc.resize(Nch);
    for(int i = 0; i < Nch; ++i)
    {
        tdelay_acc_free[i] = ::from_json(j["channel_free"][i]);
        tdelay_acc[i] = ::from_json(j["channel_stats"][i]);
    }

    if(ch_occupancy.size()!=0) // swap positions according to occupancy
        for(int i = 0; i < Nch; ++i)
            if(ch_occupancy[i]==0)
                swap(tdelay_acc_free[i],tdelay_acc[i]);
}

void ChannelPacketRateMonitor::merge_json(nlohmann::json& j2, std::vector<int> ch_occupancy)
{
    if(Nch<0) // it was empty
    {
        from_json(j2, ch_occupancy);
    }
    else
    {
        ChannelPacketRateMonitor m2;
        m2.from_json(j2, ch_occupancy);
        
        assert(Nch==m2.Nch);
    
        for(int i = 0; i < Nch; ++i)
        {
            tdelay_acc[i] += m2.tdelay_acc[i];
            tdelay_acc_free[i] += m2.tdelay_acc_free[i];
        }
    }
}

#define UNOCCUPIED_DELAY 0.1
#define ALPHA_VAR 0.1

std::vector<ExpandedScenarioDescriptor> ChannelPacketRateTester::possible_expanded_scenarios(const ChannelPacketRateMonitorInterface* m, int forbidden_channel)
{
    const auto& expanded_l = pu_api->expanded_scenarios.scenarios_expanded_list;
    const auto& delay_list = pu_api->environment_data->delay_ms_list;
    float min_err_acum = std::numeric_limits<float>::max();
    vector<int> min_idxs;
    
    for(int i = 0; i < expanded_l.size(); ++i)
    {
        float delay = delay_list[expanded_l[i].scenario->packet_delay_idx]/1000;
        // TODO: some optimizations can be done
        float err_acum = 0;
        for(int n = 0; n < m->Nch; ++n)
        {
            if(n==forbidden_channel)
                continue;
            auto true_delay = (expanded_l[i].ch_occupied_mask[n]) ? delay : UNOCCUPIED_DELAY;
            bool poisson = expanded_l[i].scenario->poisson_flag;
            auto d = m->is_occupied(n) ? m->packet_arrival_period(n) : UNOCCUPIED_DELAY; 
            auto err_mean = std::norm(d - true_delay);
            auto dvar =  m->is_occupied(n) ? m->packet_arrival_period_var(n) : UNOCCUPIED_DELAY;
            if(dvar==delay_stats::NaN())
                err_acum += err_mean;
            else
            {
                auto err_var = (poisson==true) ? std::norm(dvar-true_delay) : std::norm(dvar-0);
                err_acum += (1-ALPHA_VAR)*err_mean + ALPHA_VAR*err_var;
            }
        }
        if(err_acum < min_err_acum)
        {
            min_err_acum = err_acum;
            min_idxs.assign({i});
        }
        else if(err_acum == min_err_acum)
        {
            min_idxs.push_back(i);
        }
        //std::cout << "DEBUG: scenario error distance (" << expanded_l[i].scenario_idx << "," << i << ")=" << err_acum << endl;
    }
    
    assert(min_idxs.size()>0);
    
    vector<ExpandedScenarioDescriptor> possible_scens;
    possible_scens.reserve(min_idxs.size());
    for(auto& e : min_idxs)
        possible_scens.push_back(expanded_l[e]);
    return possible_scens;
}

vector<scenario_number_type> ChannelPacketRateTester::possible_scenario_idxs(const vector<ExpandedScenarioDescriptor>& possible_expanded_scenarios)
{
    // find unique elements
    set<scenario_number_type> set_idxs;
    for(auto& e : possible_expanded_scenarios)
        set_idxs.insert(e.scenario_idx);

    return vector<scenario_number_type>(set_idxs.begin(), set_idxs.end());
}

vector<scenario_number_type> ChannelPacketRateTester::possible_scenario_idxs(const ChannelPacketRateMonitorInterface* m, int forbidden_channel)
{
    return possible_scenario_idxs(possible_expanded_scenarios(m,forbidden_channel));
}

namespace monitor_utils
{
std::string print_packet_rate(const ChannelPacketRateMonitor& p)
{
    stringstream os;
    os << "[";
    for(int i = 0; i < p.Nch-1; ++i)
        if(p.is_occupied(i))
            os << p.packet_arrival_rate(i) << ",\t";
        else
            os << "free,\t";
    if(p.is_occupied(p.Nch-1))
        os << p.packet_arrival_rate(p.Nch-1) << "]";
    else
        os << "free]";
    return os.str();
}
std::string print_packet_period(const ChannelPacketRateMonitor& p)
{
    stringstream os;
    os << "[";
    for(int i = 0; i < p.Nch-1; ++i)
        if(p.is_occupied(i))
            os << p.packet_arrival_period(i) << ",\t";
        else
            os << "free,\t";
    if(p.is_occupied(p.Nch-1))
        os << p.packet_arrival_period(p.Nch-1) << "]";
    else
        os << "free]";
    return os.str();
}
std::string print_packet_delay_variance(const ChannelPacketRateMonitor& p)
{
    stringstream os;
    os << "[";
    float var;
    for(int i = 0; i < p.Nch-1; ++i)
    {
        var = p.packet_arrival_period_var(i);
        if(p.is_occupied(i) && var!=delay_stats::NaN())
            os << p.packet_arrival_period_var(i) << ",\t";
        else
            os << "free,\t";
    }
    var = p.packet_arrival_period_var(p.Nch-1);
    if(p.is_occupied(p.Nch-1) && var!=delay_stats::NaN())
        os << p.packet_arrival_period_var(p.Nch-1) << "]";
    else
        os << "free]";
    return os.str();
}

std::unique_ptr<JsonScenarioMonitor> make_scenario_monitor(const std::string& type)
{
    if(type=="ChannelPacketRateMonitor")
        return unique_ptr<JsonScenarioMonitor>(new ChannelPacketRateMonitor);
    else 
        throw std::runtime_error("I do not recognize such Json monitor name: " + type);
}
};