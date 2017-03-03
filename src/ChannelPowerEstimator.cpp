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
    auto ch_idxs = bmask;
    sort(ch_idxs.begin(), ch_idxs.end());
    ch_idxs.erase(std::unique(ch_idxs.begin(), ch_idxs.end()), ch_idxs.end());
    Nch = count_if(ch_idxs.begin(), ch_idxs.end(), [](float f){return f>=0;});
    
    section_props.assign(Nch,SectionProperties(-1,0,BinMask::valid));
    ignored_section_props = SectionProperties(-1,0,BinMask::guard);
    
    for(int i = 0; i < section_props.size(); ++i)
        section_props[i].channel_idx = i;
    
    for(int i = 0; i < bin_mask.size(); ++i)
    {
        if(bin_mask[i]>=0)
            section_props[bin_mask[i]].count++;
        else
            ignored_section_props.count++;
    }
    
    ch_coeffs.resize(n_sections(), 1);
    for(int j = 0; j < n_sections(); j++)
        ch_coeffs[j] = 1.0/(section_props[j].count); // NOTE: when we do FFT we may also need to divide by nBins
}

BinMask::BinMask(const vector<int>& bmask, const vector<int>& channel_map, const vector<bool>& ref_map)
{
    bin_mask = bmask;
    
    section_props.reserve(ref_map.size());
    ignored_section_props = SectionProperties(-1,0,BinMask::guard);
    
    Nch = 0;
    for(int i = 0; i < ref_map.size(); ++i)
    {
        BinMask::BinType t = (ref_map[i]) ? BinMask::reference : BinMask::valid;
        section_props.emplace_back(channel_map[i],0,t);
        if(t==BinMask::valid)
            Nch++;
    }
    for(int i = 0; i < bin_mask.size(); ++i)
    {
        if(bin_mask[i]>=0)
            section_props[bin_mask[i]].count++;
        else
            ignored_section_props.count++;
    }
    
    ch_coeffs.resize(n_sections(), 1);
    for(int j = 0; j < n_sections(); j++)
        ch_coeffs[j] = 1.0/(section_props[j].count); // NOTE: when we do FFT we may also need to divide by nBins
}

void cancel_dc_offset(vector<int> &bin_mask)
{
    int nBins = bin_mask.size();
    int margin = 7;//4;
    for(int i = -margin; i < margin; ++i)
        if(i < 0)
            bin_mask[nBins+i] = -1;
        else
            bin_mask[i] = -1;
}

namespace sensing_utils
{
BinMask generate_bin_mask_no_guard(int Nch, int nBins, bool cancel_DC_offset, bool shift)
{
    int shift_idxs = (shift==true)? nBins/2 : 0;
    vector<int> bin_mask(nBins);
    float inv_bins_per_channel = (float)Nch / nBins; // 1/bins_per_channel
    for(int i = 0; i < nBins; i++)
    {
        bin_mask[i] = floor(((i + shift_idxs) % nBins) * inv_bins_per_channel);
    }
    
    if(cancel_DC_offset)
        cancel_dc_offset(bin_mask);
    
    return BinMask(bin_mask);
}

BinMask generate_bin_mask(int Nch, int nBins, float non_guard_percentage, bool cancel_DC_offset)
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
    
    if(cancel_DC_offset)
        cancel_dc_offset(bin_mask);
    
    return BinMask(bin_mask);
}

// bin mask is equal to -1 for non assigned bins
// non_reference_mask is false for the values of the bin mask which correspond to reference bins
BinMask generate_bin_mask_and_reference(int Nch, int nBins, float non_guard_percentage, float reference_percentage, bool cancel_DC_offset, bool shift)
{
    int shift_idxs = (shift==true)? nBins/2 : 0;
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
        int ch_idx = ((i + shift_idxs)%nBins) / Nbins_per_channel;
        int j = ((i + shift_idxs)%nBins) % Nbins_per_channel;
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
    
    if(cancel_DC_offset)
        cancel_dc_offset(bin_mask);
    
    return BinMask(bin_mask, ch_map, ref_map);
}

vector<float> relative_channel_powers(const BinMask& bmask, const vector<float> &section_powers)
{
    vector<float> rel_ch_powers(bmask.Nch,0);
    vector<float> tmp_ref_powers(bmask.Nch,0);
    vector<int> rel_ch_counts(bmask.Nch,0), tmp_ch_counts(bmask.Nch,0);
    
    //assert(ch_powers.size()==bmask.n_sections());
    for(int i = 0; i < section_powers.size();++i)
    {
        const auto& el = bmask.section_props[i];
        if(el.type==BinMask::valid)
        {
            rel_ch_powers[el.channel_idx] += section_powers[i]*el.count;
            rel_ch_counts[el.channel_idx] += el.count;
        }
        else if(el.type==BinMask::reference)
        {
            tmp_ref_powers[el.channel_idx] += section_powers[i] * el.count;
            tmp_ch_counts[el.channel_idx] += el.count;
        }
    }
    
    for(int i = 0; i < bmask.Nch; ++i)
        rel_ch_powers[i] = (rel_ch_powers[i]/rel_ch_counts[i]) / (tmp_ref_powers[i]/tmp_ch_counts[i]);
    
    return rel_ch_powers;
}

void apply_bin_mask(float* output, const Cplx* cplx_ptr, const BinMask& bmask, float extra_coeff)
{
    for(size_t j = 0; j < bmask.n_sections(); ++j)
        output[j] = 0;
    
    for(unsigned int i = 0; i < bmask.size(); i++)
    {
        if(bmask[i] >= 0)
            output[bmask[i]] += cplx_ptr[i].real()*cplx_ptr[i].real() + cplx_ptr[i].imag()*cplx_ptr[i].imag();//norm(fftBins[i]);
    }
    
    for(size_t j = 0; j < bmask.n_sections(); ++j)
        output[j] *= (bmask.ch_coeffs[j] * extra_coeff);
}

void apply_bin_mask(float* output, const float* pwr_ptr, const BinMask& bmask)
{
    for(size_t j = 0; j < bmask.n_sections(); ++j)
        output[j] = 0;
    
    for(unsigned int i = 0; i < bmask.size(); i++)
    {
        if(bmask[i] >= 0)
            output[bmask[i]] += pwr_ptr[i];//norm(fftBins[i]);
    }
    
    for(size_t j = 0; j < bmask.n_sections(); ++j)
        output[j] *= bmask.ch_coeffs[j];
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
    
    bin_mask = sensing_utils::generate_bin_mask_no_guard(Nch,nBins,1.0);
    
    cout << "Bin mask:";
    cout << print_range(bin_mask) << endl;
    
    setup();
}

void ChannelPowerEstimator::set_parameters(uint16_t _avg_win_size,
                                    const BinMask &_bin_mask) 
{
    mavg_size = _avg_win_size;
    Nch = _bin_mask.Nch;
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
    output_ch_pwrs.assign(bin_mask.n_sections(), 0);
    
//    cout << "\n FFT input: " 
//         << print_container_dB(&fftBins[0], &fftBins[nBins]) << endl;
    
    // Averages across bins belonging to the same channel
    sensing_utils::apply_bin_mask(&output_ch_pwrs[0], fftBins, bin_mask, 1.0/nBins);
//    
//    for(unsigned int i = 0; i < nBins; i++)
//    {
//        if(bin_mask[i] >= 0)
//            output_ch_pwrs[bin_mask[i]] += fftBins[i].real()*fftBins[i].real() + fftBins[i].imag()*fftBins[i].imag();//norm(fftBins[i]);
//    }
//    
//    for(unsigned int i = 0; i < bin_mask.n_sections(); ++i)
//        output_ch_pwrs[i] *= ch_avg_coeff[i];
    
    current_tstamp = tstamp;
}

void PacketDetector::work(double tstamp, const vector<float>& vals)
{
    constexpr float THRES2 = 1.8;//1.6
    constexpr double ALPHA = 1.0e-3;
    constexpr int FIRST_N = 1e3;
    
    for(int i = 0; i < vals.size(); ++i)
    {
        if(vals[i]<0)   // it is the SU-Tx channel
        {
            params[i].counter_block = 0;
            params[i].pu_detected = false; // just keep sure it is not caught in between detecting a packet
            continue;
        }

        // compute average pwr
//        avg_pwr[i].first++;
//        avg_pwr[i].second = avg_pwr[i].second + (vals[i]-avg_pwr[i].second)/avg_pwr[i].first;
        
        float old_sample = mov_avg[i].push(vals[i]);
        
        if(params[i].counter_block>0) // || vals[i] < pow(10,-90/10)
        {        
            params[i].counter_block--;
            continue;
        }
        
        float val_smoothed = mov_avg[i].get_avg();
        bool test = val_smoothed > thres * params[i].noise_floor;
        if(params[i].pu_detected==false && (test==false || params[i].n_noise_samples < 100))
        {
            if(params[i].n_noise_samples < 5 || val_smoothed < THRES2 * params[i].noise_floor || params[i].n_samples_out >= 5*max_plen)
            {
                params[i].n_noise_samples++;
                if(params[i].n_noise_samples < FIRST_N)
                {
                    params[i].noise_floor = params[i].noise_floor + (vals[i]-params[i].noise_floor)/params[i].n_noise_samples;
                }
                else if(val_smoothed < THRES2 * params[i].noise_floor)
                    params[i].noise_floor = (1-ALPHA) * params[i].noise_floor + ALPHA*old_sample;
                params[i].n_samples_out = 0;
                params[i].min_val = std::numeric_limits<float>::max();
            }
            else
            {
                // don't let the noise estimator get locked in a very low value
                params[i].n_samples_out++;
                if(params[i].min_val > val_smoothed)
                    params[i].min_val = val_smoothed;
            }
            // in between thres and THRES2 is the gray area
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

void push_detected_packets(const std::vector<DetectedPacket>& detected_packets, SituationalAwarenessApi* api)
{
    auto range_idxs = ranges::make_range(api->environment_data->num_channels);
    
    for(auto rit = detected_packets.rbegin(); rit != detected_packets.rend() && !range_idxs.empty(); ++rit)
    {
        auto it = find(range_idxs.begin(), range_idxs.end(), get<1>(*rit));
        if(it != range_idxs.end())
        {
            api->stats.add_packet_tstamp(get<0>(*rit),get<1>(*rit));
            range_idxs.erase(it);
        }
    }
}

void SpectrogramGenerator::push_line(double tstamp, const vector<float>& ch_pwrs)
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
            d().second[j] = mov_avg[j].get_avg();
        }
        
        assert(d().second.size()>0);
        
        mavg_count = 0;
    }
}

bool VectorMovingAverage::work(const vector<float>& data)
{
    assert(data.size()==mov_avg.size());
    assert(mov_avg[0].size()>0);
    
    // Adds the channel average power to the moving average
    for(uint16_t j = 0; j < mov_avg.size(); j++)
        mov_avg[j].push(data[j]);
    
    if(++step_count == step_size)
    {
        step_count = 0;
        return true;
    }
    return false;
}

void VectorMovingAverage::read_line(vector<float>& out)
{
    out.resize(mov_avg.size());
    
    for(uint16_t j = 0; j < mov_avg.size(); j++) 
    {
        out[j] = mov_avg[j].get_avg();
    }
}

buffer_utils::rdataset<ChPowers> SpectrogramGenerator::pop_line()
{
    buffer_utils::rdataset<ChPowers> ch_powers = results.get_rdataset();
    
    return std::move(ch_powers);
}
