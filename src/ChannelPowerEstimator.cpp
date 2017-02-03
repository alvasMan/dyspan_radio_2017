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

using namespace std;
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
    bin_mask[0] = -1;
    bin_mask[1] = -1;
    
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
    
    tmp_ch_power.resize(Nch);
    ch_avg_coeff.resize(Nch, 1);
    mavg_step_size = mavg_size;
    mavg_count = 0;
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
    tmp_ch_power.assign(Nch, 0);
    
//    cout << "\n FFT input: " 
//         << print_container_dB(&fftBins[0], &fftBins[nBins]) << endl;
    
    // Averages across bins belonging to the same channel
    for(unsigned int i = 0; i < nBins; i++)
    {
        if(bin_mask[i] >= 0)
            tmp_ch_power[bin_mask[i]] += fftBins[i].real()*fftBins[i].real() + fftBins[i].imag()*fftBins[i].imag();//norm(fftBins[i]);
    }
    
    for(unsigned int i = 0; i < Nch; ++i)
        tmp_ch_power[i] *= ch_avg_coeff[i];
    
    if(spectrogram_module())
        spectrogram_module->work(tstamp, tmp_ch_power);
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
    return std::move(buffer_utils::rdataset<ChPowers>(spectrogram_module->results.get_rdataset()));
}

class PacketDetector
{
public:
    PacketDetector(int n_channels, int mov_avg_len, int max_packet_length) : 
            max_plen(max_packet_length), Nch(n_channels) 
            {
                setup();
            }
    
    void setup();
    void work(double tstamp, const vector<float>& vals);
    
private:
    std::vector< MovingAverage<float> > mov_avg;
    std::pair<double, float> mov_max = {0,0};
    int max_plen;
    int Nch = 4;
    float thres;
    float thres2;
    
    bool pu_detected = false;
    int counter_stop = 0;
    int counter_max = 2;
    int n_packet = 0;
    std::vector<float> noise_floor;
    std::deque< std::pair<double, float> > detected_pulses;
};

void PacketDetector::setup()
{
    mov_avg.resize(Nch);
    for(auto& m : mov_avg)
        m.set_size(max_plen);
}

void PacketDetector::work(double tstamp, const vector<float>& vals)
{
    for(int i = 0; i < vals.size(); ++i)
    {
        mov_avg[i].push(vals[i] - noise_floor[i]);
        
        float val_smoothed = mov_avg[i].get_avg();
        bool test = val_smoothed > (thres - 1) * noise_floor[i];
        if(test && pu_detected==false)
        {
            noise_floor[i] = 0.99 * noise_floor[i] + 0.01*vals[i];
        }
        else
        {
            if(pu_detected==false)
            {
                pu_detected = true;
                counter_stop = 0;
                n_packet = 0;
            }
            
            if(test==false)
                counter_stop++;
            else
                counter_stop = 0;
            
            if(val_smoothed > mov_max[i].second)
            {
                mov_max[i].first = tstamp;
                mov_max[i].second = val_smoothed;
            }
            
            if(counter_stop > counter_max || n_packet >= mov_avg[i].size())
            {
                // select winner
                if(mov_max[i].second > thres2)
                {
                    detected_pulses.push_back(make_pair(mov_max[i].first, mov_max[i].second));
                }
                
                pu_detected = false;
            }
            else
                n_packet++;
        }
    }
}

void SpectrogramGenerator::work(double tstamp, vector<float>& ch_pwrs)
{
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