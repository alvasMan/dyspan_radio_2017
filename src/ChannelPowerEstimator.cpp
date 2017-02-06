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
    
    // Define the bin mask    
    bin_mask.resize(nBins);
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
    
    cout << "Bin mask:";
    cout << print_range(bin_mask) << endl;
    
    setup();
}

void ChannelPowerEstimator::set_parameters(uint16_t _avg_win_size, 
                                    uint16_t num_channels, 
                                    const std::vector<int> &_bin_mask) 
{
    mavg_size = _avg_win_size;
    Nch = num_channels;
    bin_mask = _bin_mask;       // Values in bin_mask must be within [0, number of channels] and size of bin_mask must be equal to nBins. No protection yet.
    nBins = bin_mask.size();

    setup();
}

void ChannelPowerEstimator::setup() 
{
    // Create new FFT
    destroy();
    fftBins = reinterpret_cast<Cplx*>(fftwf_malloc(sizeof(fftwf_complex) * nBins));
    fft = fftwf_plan_dft_1d(nBins, (fftwf_complex*)fftBins, (fftwf_complex*)fftBins, FFTW_FORWARD, FFTW_MEASURE);
    bin_idx = 0;
    
    // Create SpectrogramGenerator
    spectrogram_module.reset(new SpectrogramGenerator(Nch, mavg_size));
    
    output_ch_pwrs.resize(Nch,-1);
    ch_avg_coeff.resize(Nch, 1);
    //mavg_count = mavg_step_size-mavg_size; // let the mavg fill completely the first time
    
    // Count number of bins belonging to each channel
    vector<int> ch_count(Nch,0);
    for(unsigned int i = 0; i < nBins; i++) 
    {
        if(bin_mask[i] >= 0)
            ch_count[bin_mask[i]]++;
    }
    for(int j = 0; j < Nch; j++)
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
    
    if(spectrogram_module)
        spectrogram_module->work(tstamp, output_ch_pwrs);
    
    current_tstamp = tstamp;
}

bool ChannelPowerEstimator::try_pop_result(buffer_utils::rdataset<ChPowers> &d)
{
    return spectrogram_module->results.try_get_rdataset(d);
//    if(result_exists())
//    {
//        buffer_utils::rdataset<ChPowers> d;
//        results.get_rdataset(d);
//        vec = d().second; // NOTE: should I copy?
//        tstamp = d().first;
//        return true;
//    }
//    return false;
}

buffer_utils::rdataset<ChPowers> ChannelPowerEstimator::pop_result()
{
    if(spectrogram_module)
        return std::move(spectrogram_module->results.get_rdataset());
    else
        return std::move(buffer_utils::rdataset<ChPowers>());
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
                params[i].noise_floor = params[i].noise_floor + (old_sample-params[i].noise_floor)/params[i].n_noise_samples;
            }
            else
                params[i].noise_floor = 0.96 * params[i].noise_floor + 0.04*vals[i];
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
                    cout << "DEBUG: Packet detected in channel " << i << " with power " << 10*log10(mov_max[i].second) << endl;
                    cout << "DEBUG: Noise floor " << 10*log10(params[i].noise_floor) << endl;
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
        if(channel_energy[i] < ch_pwrs[i])
            channel_energy[i] = ch_pwrs[i];
        else
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



namespace monitor_utils
{
std::string print_packet_rate(const ChannelPacketRateMonitor& p)
{
    stringstream os;
    os << "[";
    for(int i = 0; i < p.Nch-1; ++i)
        if(p.is_occupied(i))
            os << 1.0/p.packet_arrival_period(i) << ",";
        else
            os << "free,";
    if(p.is_occupied(p.Nch-1))
        os << 1.0/p.packet_arrival_period(p.Nch-1) << "]";
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