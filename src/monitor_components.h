/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   monitor_components.h
 * Author: connect
 *
 * Created on 13 February 2017, 14:04
 */
#include "context_awareness.h"
#include "json_utils.h"
#include <vector>
#include "stats.h"

using std::vector;
using std::pair;
using std::string;

#ifndef MONITOR_COMPONENTS_H
#define MONITOR_COMPONENTS_H

typedef double time_format;
typedef std::tuple<time_format,int,float> DetectedPacket;

class ChannelPacketRateMonitorInterface
{
public:
    ChannelPacketRateMonitorInterface() = default;
    ChannelPacketRateMonitorInterface(int n_channels) : Nch(n_channels)
    {
    }
    static float NaN() {return std::numeric_limits<time_format>::max();}
    
    virtual time_format packet_arrival_period(int i) const = 0;
    virtual time_format packet_arrival_period_var(int i) const = 0;
    virtual time_format packet_arrival_rate(int i) const = 0;    
    virtual bool is_occupied(int i) const = 0;
    virtual ~ChannelPacketRateMonitorInterface() {}
    
    int Nch = -1;
};

//********** Implementations ****************//

class ForgetfulChannelMonitor
{
public:
    ForgetfulChannelMonitor() = default;
    ForgetfulChannelMonitor(int n_channels, float alpha_x = 0.1) : 
            Nch(n_channels), 
            alpha(alpha_x), 
            channel_energy(Nch)
    {
    }
    
    void work(const vector<float>& ch_pwrs);
    vector<size_t> ch_sorted_by_energy();
    
    int Nch;
    float alpha;
    
    vector<float> channel_energy;
};

#define TDELAY_MAX 0.2

struct delay_stats
{
    long count = 0;
    time_format sum = 0;
    time_format sum2 = 0;
    
    inline void push(time_format val) 
    {
        sum += val;
        sum2 += pow(val,2);
        count++;
    }
    static time_format NaN() {return std::numeric_limits<time_format>::max();}
    inline time_format mean() const { return (count>0) ? sum/count : NaN(); }
    inline double var() const { return (count>1) ? (sum2-pow(sum,2)/count)/(count-1) : NaN(); }
    inline time_format std() const { return sqrt(var()); }
    inline delay_stats& operator+=(const delay_stats& d) 
    {
        count += d.count;
        sum += d.sum;
        sum2 += d.sum2;
    }
};

class ChannelPacketRateMonitor : public ChannelPacketRateMonitorInterface, public JsonScenarioMonitor
{
public:
    ChannelPacketRateMonitor() = default;
    ChannelPacketRateMonitor(int n_channels) : 
                ChannelPacketRateMonitorInterface(n_channels), 
                tdelay_acc(n_channels), tdelay_acc_free(n_channels),
                prev_packet_tstamp(n_channels,0) 
    {
    }

    // ChannelPacketRateMonitorInterface
    void work(const vector<DetectedPacket>& packets, time_format tstamp);
    inline time_format packet_arrival_period(int i) const final
    {
        return tdelay_acc[i].mean();
    }
    inline time_format packet_arrival_period_var(int i) const final
    {
        return tdelay_acc[i].var();
    }
    inline time_format packet_arrival_rate(int i) const  final
    {
        auto t = tdelay_acc[i].mean();
        return (t != delay_stats::NaN()) ? 1.0/t : 0;
    }
     // NOTE: Coarse estimation of availability of the channel. If Pfa is high, this wont work.
    inline bool is_occupied(int i) const final {return packet_arrival_period(i) < TDELAY_MAX;}
    
    // JsonScenarioMonitorInterface
    string json_key() {return "ChannelPacketRateMonitor";}
    nlohmann::json to_json() final;
    void from_json(nlohmann::json& j, vector<int> ch_occupancy = {}) final;
    void merge_json(nlohmann::json& j2, vector<int> ch_occupancy = {}) final;
    
    vector<delay_stats> tdelay_acc;
    vector<delay_stats> tdelay_acc_free;
    
    vector<time_format> prev_packet_tstamp;
    time_format current_tstamp = -1;
};

class SlidingChannelPacketRateMonitor : public ChannelPacketRateMonitorInterface
{
public:
    SlidingChannelPacketRateMonitor() = default;
    SlidingChannelPacketRateMonitor(int n_channels, int avg_size) : 
                ChannelPacketRateMonitorInterface(n_channels), 
                delays_mavg(n_channels, GrowingMovingAverage<time_format>(avg_size, 10*avg_size)), 
                delays_m2(n_channels, GrowingMovingAverage<time_format>(avg_size, 10*avg_size)),
                prev_packet_tstamp(n_channels,0) 
    {
    }
    void work(const vector<DetectedPacket>& packets, time_format tstamp);
    inline time_format packet_arrival_period(int i) const final
    {
        auto siz = delays_mavg[i].size();
        if(current_tstamp-prev_packet_tstamp[i] > TDELAY_MAX || siz == 0)
            return ChannelPacketRateMonitorInterface::NaN();
        else
            return delays_mavg[i].sum()/siz;
    }
    inline time_format packet_arrival_period_var(int i) const final
    {
        auto siz = delays_mavg[i].size();
        if(current_tstamp-prev_packet_tstamp[i] > TDELAY_MAX || siz<=1)
            return ChannelPacketRateMonitorInterface::NaN();
        else
            return (delays_m2[i].sum()-pow(delays_mavg[i].sum(),2)/siz)/(siz-1);
    }
    inline time_format packet_arrival_rate(int i) const  final
    {
        auto t = packet_arrival_period(i);
        return (t != ChannelPacketRateMonitorInterface::NaN()) ? 1.0/t : 0;
    }
    inline bool is_occupied(int i) const final {return packet_arrival_period(i) < TDELAY_MAX;}
    
    vector<GrowingMovingAverage<time_format>> delays_mavg;
    vector<GrowingMovingAverage<time_format>> delays_m2;
    
    time_format current_tstamp;
    vector<time_format> prev_packet_tstamp;
};

class ChannelPacketRateTester
{
public:
    ChannelPacketRateTester() = default;
    ChannelPacketRateTester(SituationalAwarenessApi* api) : pu_api(api)
    {
    }
    SituationalAwarenessApi* pu_api = NULL;
    
    vector<ExpandedScenarioDescriptor> possible_expanded_scenarios(const ChannelPacketRateMonitorInterface* m, int forbidden_channel = -1);
    vector<scenario_number_type> possible_scenario_idxs(const vector<ExpandedScenarioDescriptor>& possible_expanded_scenarios);
    vector<scenario_number_type> possible_scenario_idxs(const ChannelPacketRateMonitorInterface* m, int forbidden_channel = -1);
};

namespace monitor_utils
{
string print_packet_rate(const ChannelPacketRateMonitor& p);
string print_packet_period(const ChannelPacketRateMonitor& p);
string print_packet_delay_variance(const ChannelPacketRateMonitor& p);

string print_packet_period(const SlidingChannelPacketRateMonitor& p);
string print_packet_delay_variance(const SlidingChannelPacketRateMonitor& p);
};

#endif /* MONITOR_COMPONENTS_H */

