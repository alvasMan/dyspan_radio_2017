/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2015  Andre Puschmann, Francisco Paisana, Justin Tallon
 *
 * \section LICENSE
 *
 * This file is part of dyspanradio.
 *
 * dyspanradio is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * dyspanradio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "ChannelPowerEstimator.h"
#include "general_utils.hpp"
#include "sensing_components.h"
#include "dyspanradio.h"
#include <sstream>
//#include "matplotlibcpp.h"

using namespace std;
using namespace nlohmann;
//using namespace dyspan;

BinMask::BinMask(const vector<int>& bmask)
{
    bin_mask = bmask;
    int ch_idx = 0;
    for(int i = 0; i < bin_mask.size(); ++i)
    {
        auto it = section_props.find(bin_mask[i]);
        if(it==section_props.end())
        {
            if(bin_mask[i]>=0)
                section_props[bin_mask[i]] = {ch_idx++,1,BinMask::valid};
            else
                section_props[bin_mask[i]] = {bin_mask[i],1,BinMask::guard};
        }
        else
            it->second.count++;
    }
    auto mask_cpy = bin_mask;
    Nch = std::distance(mask_cpy.begin(),unique(mask_cpy.begin(),mask_cpy.end()));
}

BinMask::BinMask(const vector<int>& bmask, const vector<int>& channel_map, const vector<bool>& ref_map)
{
    bin_mask = bmask;
    Nch = 0;
    for(int i = 0; i < bin_mask.size(); ++i)
    {
        auto it = section_props.find(bin_mask[i]);
        if(it==section_props.end())
        {
            if(bin_mask[i]>=0)
            {
                BinMask::BinType t = (ref_map[bin_mask[i]]) ? BinMask::reference : BinMask::valid;
                section_props[bin_mask[i]] = {channel_map[bin_mask[i]],1,t};
                if(t==BinMask::valid)
                    Nch++;
            }
            else
                section_props[bin_mask[i]] = {bin_mask[i],1,BinMask::guard};
        }
        else
            it->second.count++;
    }
}

namespace sensing_utils
{
BinMask generate_bin_mask(int Nch, int nBins)
{
    vector<int> bin_mask(nBins);
    float inv_bins_per_channel = (float)Nch / nBins; // 1/bins_per_channel
    for(int i = 0; i < nBins; i++)
    {
        bin_mask[i] = floor(((i + nBins/2) % nBins) * inv_bins_per_channel);
    }
    
    // Cancel DC Offset
    bin_mask[nBins-1] = -1;
    bin_mask[nBins-2] = -1;
    bin_mask[0] = -1;
    bin_mask[1] = -1;
    bin_mask[2] = -1;
    
    return BinMask(bin_mask);
}

BinMask generate_bin_mask(int Nch, int nBins, float non_guard_percentage)
{
    int Nbins_per_channel = nBins / Nch;
    int non_guard_bins = round(non_guard_percentage*Nbins_per_channel);
    int half_guard_bins = (Nbins_per_channel - non_guard_bins)/2;
    
    vector<int> bin_mask(nBins,-1);
    for(int i = 0; i < nBins; i++)
    {
        int ch_idx = ((i + nBins/2)%nBins) / Nbins_per_channel;
        int j = ((i + nBins/2)%nBins) % Nbins_per_channel;
        if(j >= half_guard_bins && j < (Nbins_per_channel-half_guard_bins))
            bin_mask[i] = ch_idx;
    }
    
    // Cancel DC Offset
    bin_mask[nBins-1] = -1;
    bin_mask[nBins-2] = -1;
    bin_mask[0] = -1;
    bin_mask[1] = -1;
    bin_mask[2] = -1;
    
    return BinMask(bin_mask);
}

// bin mask is equal to -1 for non assigned bins
// non_reference_mask is false for the values of the bin mask which correspond to reference bins
BinMask generate_bin_mask_and_reference(int Nch, int nBins, float non_guard_percentage, float reference_percentage)
{
    int Nbins_per_channel = nBins / Nch;
    int non_guard_bins = round(non_guard_percentage*Nbins_per_channel);
    int half_guard_bins = (Nbins_per_channel - non_guard_bins)/2;
    int half_reference_bins = round(reference_percentage*Nbins_per_channel/2);
 
    assert(half_reference_bins>0);   
    assert(half_reference_bins<=half_guard_bins);
    
    vector<int> bin_mask(nBins,-1);
    vector<int> ch_map(3*Nch);
    vector<bool> ref_map(3*Nch);
    for(int i = 0; i < nBins; i++)
    {
        int ch_idx = ((i + nBins/2)%nBins) / Nbins_per_channel;
        int j = ((i + nBins/2)%nBins) % Nbins_per_channel;
        if(j < half_reference_bins)
            bin_mask[i] = ch_idx*3;
        else if(j>= half_guard_bins && j < half_guard_bins + non_guard_bins)
            bin_mask[i] = ch_idx*3+1;
        else if(j >= Nbins_per_channel - half_reference_bins)
            bin_mask[i] = ch_idx*3+2;
    }
    for(int m = 0; m < Nch; ++m)
    {
        ch_map[m*3] = m;
        ch_map[m*3+1] = m;
        ch_map[m*3+2] = m;
        ref_map[m*3] = true;
        ref_map[m*3+1] = false;
        ref_map[m*3+2] = true;
    }
    
    // Cancel DC Offset
    bin_mask[nBins-1] = -1;
    bin_mask[nBins-2] = -1;
    bin_mask[0] = -1;
    bin_mask[1] = -1;
    bin_mask[2] = -1;
    
    return BinMask(bin_mask, ch_map, ref_map);
}

vector<float> relative_channel_powers(const BinMask& bmask, const vector<float> &ch_powers)
{
    vector<float> rel_ch_powers(bmask.Nch,0);
    vector<float> tmp_ref_powers(bmask.Nch,0);
    vector<int> rel_ch_counts(bmask.Nch,0), tmp_ch_counts(bmask.Nch,0);
    
    //assert(ch_powers.size()==bmask.n_sections());
    for(int i = 0; i < ch_powers.size();++i)
    {
        const auto& el = bmask.section_props.find(i)->second;
        if(el.type==BinMask::valid)
        {
            rel_ch_powers[el.channel_idx] += ch_powers[i]*el.count;
            rel_ch_counts[el.channel_idx] += el.count;
        }
        else if(el.type==BinMask::reference)
        {
            tmp_ref_powers[el.channel_idx] += ch_powers[i] * el.count;
            tmp_ch_counts[el.channel_idx] += el.count;
        }
    }
    
    for(int i = 0; i < bmask.Nch; ++i)
        rel_ch_powers[i] = (rel_ch_powers[i]/rel_ch_counts[i]) / (tmp_ref_powers[i]/tmp_ch_counts[i]);
    
    return rel_ch_powers;
}

};

ChannelPowerEstimator::ChannelPowerEstimator() : fftBins(NULL), fft(NULL)
{
}

void ChannelPowerEstimator::destroy() 
{
    if(fftBins != NULL)
        fftwf_free(fftBins);
    if(fft != NULL)
        fftwf_destroy_plan(fft);
}

void ChannelPowerEstimator::set_parameters(uint16_t _avg_win_size, 
                                    uint16_t fftsize, 
                                    uint16_t num_channels) 
{
    mavg_size = _avg_win_size;
    nBins = fftsize;
    Nch = num_channels;
    
    bin_mask = BinMask(Nch,nBins,1.0);
    
    cout << "Bin mask:";
    cout << print_range(bin_mask) << endl;
    
    setup();
}

void ChannelPowerEstimator::set_parameters(uint16_t _avg_win_size, 
                                    uint16_t num_channels, 
                                    const BinMask &_bin_mask) 
{
    mavg_size = _avg_win_size;
    Nch = num_channels;
    bin_mask = _bin_mask;       // Values in bin_mask must be within [0, number of channels] and size of bin_mask must be equal to nBins. No protection yet.
    nBins = bin_mask.size();

    cout << "Bin mask:";
    cout << print_range(bin_mask) << endl;
    
    setup();
}

void ChannelPowerEstimator::setup() 
{
    // Create new FFT
    destroy();
    fftBins = reinterpret_cast<Cplx*>(fftwf_malloc(sizeof(fftwf_complex) * nBins));
    fft = fftwf_plan_dft_1d(nBins, (fftwf_complex*)fftBins, (fftwf_complex*)fftBins, FFTW_FORWARD, FFTW_MEASURE);
    bin_idx = 0;
    
//    // Create SpectrogramGenerator
//    spectrogram_module.reset(new SpectrogramGenerator(Nch, mavg_size));
    
    output_ch_pwrs.resize(bin_mask.n_sections(),-1);
    ch_avg_coeff.resize(bin_mask.n_sections(), 1);
    //mavg_count = mavg_step_size-mavg_size; // let the mavg fill completely the first time
    
    // Count number of bins belonging to each channel
    vector<int> ch_count(bin_mask.n_sections(),0);
    for(unsigned int i = 0; i < nBins; i++) 
    {
        if(bin_mask[i] >= 0)
            ch_count[bin_mask[i]]++;
    }
    for(int j = 0; j < bin_mask.n_sections(); j++)
        ch_avg_coeff[j] = 1/((float)ch_count[j] * nBins);
    
    // create noise_filter
    //noise_filter.reset(new NoiseFilter3(Nch, 0.001, 7));
}

void ChannelPowerEstimator::push_samples(const vector<Cplx> &vec) 
{
    for(unsigned int i = 0; i < vec.size(); i++) 
    {
        fftBins[bin_idx] = vec[i];
        if(++bin_idx == nBins) 
        {
            process();
            bin_idx = 0;
        }
    }
}

void ChannelPowerEstimator::push_sample(Cplx val) 
{
    fftBins[bin_idx] = val;
    if (++bin_idx == nBins) 
    {
        process();
        bin_idx = 0;
    }
}

void ChannelPowerEstimator::process(double tstamp) 
{
//    cout << "\n FFT input: ";
//    cout << print_container_dB(&fftBins[0], &fftBins[nBins]) << endl;
    
    fftwf_execute(fft);
    output_ch_pwrs.assign(Nch, 0);
    
//    cout << "\n FFT input: " 
//         << print_container_dB(&fftBins[0], &fftBins[nBins]) << endl;
    
    // Averages across bins belonging to the same channel
    for(unsigned int i = 0; i < nBins; i++)
    {
        if(bin_mask[i] >= 0)
            output_ch_pwrs[bin_mask[i]] += fftBins[i].real()*fftBins[i].real() + fftBins[i].imag()*fftBins[i].imag();//norm(fftBins[i]);
    }
    
    for(unsigned int i = 0; i < Nch; ++i)
        output_ch_pwrs[i] *= ch_avg_coeff[i];
    
    current_tstamp = tstamp;
}

void PacketDetector::work(double tstamp, const vector<float>& vals)
{
    for(int i = 0; i < vals.size(); ++i)
    {
        // compute average pwr
//        avg_pwr[i].first++;
//        avg_pwr[i].second = avg_pwr[i].second + (vals[i]-avg_pwr[i].second)/avg_pwr[i].first;
        
        float old_sample = mov_avg[i].push(vals[i]);
        
        if(params[i].counter_block>0 || vals[i] < pow(10,-90/10))
        {        
            params[i].counter_block--;
            continue;
        }
        
        float val_smoothed = mov_avg[i].get_avg();
        bool test = val_smoothed > thres * params[i].noise_floor;
        if(params[i].pu_detected==false && (test==false || params[i].n_noise_samples < 100))
        {    
            params[i].n_noise_samples++;
            if(params[i].n_noise_samples < 100)
            {
                params[i].noise_floor = params[i].noise_floor + (vals[i]-params[i].noise_floor)/params[i].n_noise_samples;
            }
            else
                params[i].noise_floor = 0.96 * params[i].noise_floor + 0.04*old_sample;
//            cout << "DEBUG: Noise floor " << noise_floor[i] << endl;
        }
        else
        {
            if(params[i].pu_detected==false)
            {
                params[i].pu_detected = true;
                params[i].counter_stop = 0;
                params[i].n_packet = 0;
                mov_max[i] = make_pair(-1,-1);
            }
            
            if(test==false)
                params[i].counter_stop++;
            else
                params[i].counter_stop = 0;
            
            if(val_smoothed > mov_max[i].second)
            {
                mov_max[i].first = tstamp;
                mov_max[i].second = val_smoothed;
                params[i].n_packet = 0;
            }
            
            if(params[i].counter_stop > counter_max || params[i].n_packet >= mov_avg[i].size()/2)
            {
                //std:: cout << "DEBUG: Finished search for packet. Val smoothed " << val_smoothed << endl;
                // select winner
                if(mov_max[i].second > thres*params[i].noise_floor)
                {
                    detected_pulses.push_back(make_tuple(mov_max[i].first, i, mov_max[i].second));
                    params[i].counter_block = mov_avg[i].size() - params[i].n_packet;
                    //cout << "DEBUG: Packet detected in channel " << i << " with power " << 10*log10(mov_max[i].second) << endl;
                    //cout << "DEBUG: Noise floor " << 10*log10(params[i].noise_floor) << endl;
//                    auto d = mov_avg[i].data();
//                    cout << "DEBUG: Average Power " << 10*log10(avg_pwr[i].second) << endl;
//                    transform(d.begin(), d.end(), d.begin(), [](float &f){return 10*log10(f);});
//                    matplotlibcpp::plot(d);
//                    matplotlibcpp::show();
                }
                
                params[i].pu_detected = false;
            }
            else
                params[i].n_packet++;
        }
    }
    sort(detected_pulses.begin(), detected_pulses.end(), [](DetectedPacket& a, DetectedPacket &b){return get<0>(a) < get<0>(b);});
}

void SpectrogramGenerator::work(double tstamp, const vector<float>& ch_pwrs)
{
    assert(ch_pwrs.size()==mov_avg.size());
    assert(mov_avg[0].size()>0);
    
    // Adds the channel average power to the moving average
    for(uint16_t j = 0; j < mov_avg.size(); j++) 
        mov_avg[j].push(ch_pwrs[j]);
    
    // If a big enough step was done in the moving average, send the average values to the results buffer
    if(++mavg_count == step_size)
    {
        buffer_utils::wdataset<ChPowers> d = results.get_wdataset();
        d().first = tstamp;
        d().second.resize(mov_avg.size());
        
        for(uint16_t j = 0; j < mov_avg.size(); j++) 
        {
            d().second[j] = mov_avg[j].get_avg();//ch_pwr_ma_last_outputs[j].max(); 
        }
        
        assert(d().second.size()>0);
        
        mavg_count = 0;
    }
}

void ForgetfulChannelMonitor::work(const std::vector<float>& ch_pwrs)
{
    // make kind of a fosphor thing
    for(int i = 0; i < channel_energy.size(); ++i)
//        if(channel_energy[i] < ch_pwrs[i])
//            channel_energy[i] = ch_pwrs[i];
//        else
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

void ChannelPacketRateMonitor::work(const std::vector<DetectedPacket>& packets)
{
    
    for(const auto& e : packets)
    {
        int ch_idx = get<1>(e);
        if(prev_packet_tstamp[ch_idx] > 0)
        {
            time_format tdelay = get<0>(e) - prev_packet_tstamp[ch_idx];
            tdelay_sum[ch_idx].first++;
            tdelay_sum[ch_idx].second += tdelay;
        }
        prev_packet_tstamp[ch_idx] = get<0>(e);
    }
}


json ChannelPacketRateMonitor::to_json()
{
    vector<time_format> tsum(Nch,0), tsum_free(Nch,0);
    vector<long> tcount(Nch,0), tcount_free(Nch,0);
    for(int i = 0; i < Nch; ++i)
    {
        tsum[i] = tdelay_sum[i].second;
        tcount[i] = tdelay_sum[i].first;
        tsum_free[i] = tdelay_free_sum[i].second;
        tcount_free[i] = tdelay_free_sum[i].first;
    }
    
    json j = {
        {"Nch", Nch},
        {"channel_stats", {{"sum",tsum},{"count",tcount}}},
        {"channel_free", {{"sum",tsum_free},{"count",tcount_free}}}
    };
    
    return j;
}


void ChannelPacketRateMonitor::from_json(nlohmann::json& j, vector<int> ch_occupancy)
{
    if(j.empty())
        return;
    Nch = j["Nch"].get<decltype(Nch)>();
    
    
    tdelay_free_sum.resize(Nch, make_pair(0,0));
    tdelay_sum.resize(Nch, make_pair(0,0));
    for(int i = 0; i < Nch; ++i)
    {
        tdelay_free_sum[i].first = j["channel_free"]["count"][i].get<long>();
        tdelay_free_sum[i].second = j["channel_free"]["sum"][i].get<time_format>();
        tdelay_sum[i].first = j["channel_stats"]["count"][i].get<long>();
        tdelay_sum[i].second = j["channel_stats"]["sum"][i].get<time_format>();
    }

    if(ch_occupancy.size()!=0) // swap positions according to occupancy
        for(int i = 0; i < Nch; ++i)
            if(ch_occupancy[i]==0)
                swap(tdelay_free_sum[i],tdelay_sum[i]);
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
            tdelay_sum[i].first += m2.tdelay_sum[i].first;
            tdelay_sum[i].second += m2.tdelay_sum[i].second;
            tdelay_free_sum[i].first += m2.tdelay_free_sum[i].first;
            tdelay_free_sum[i].second += m2.tdelay_free_sum[i].second;
        }
    }
}

#define UNOCCUPIED_DELAY 0.1

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
            auto d = m->is_occupied(n) ? m->packet_arrival_period(n) : UNOCCUPIED_DELAY; 
            err_acum += pow(abs(d - true_delay),2);
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
            os << p.packet_arrival_rate(i) << ",";
        else
            os << "free,";
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
            os << p.packet_arrival_period(i) << ",";
        else
            os << "free,";
    if(p.is_occupied(p.Nch-1))
        os << p.packet_arrival_period(p.Nch-1) << "]";
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