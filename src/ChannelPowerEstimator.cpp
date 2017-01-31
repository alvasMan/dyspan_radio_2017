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

using namespace std;
//using namespace dyspan;


ChannelPowerEstimator::ChannelPowerEstimator() : fftBins(NULL), fft(NULL), results(1000)
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
    
    // Create Moving Averages
    ch_pwr_ma.resize(Nch, MovingAverage<double>(mavg_size));
    
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

void ChannelPowerEstimator::process(double tstamp) {
//    cout << "\n FFT input: ";
//    cout << print_container_dB(&fftBins[0], &fftBins[nBins]) << endl;
    
    fftwf_execute(fft);
    tmp_ch_power.assign(ch_pwr_ma.size(), 0);
    
//    cout << "\n FFT input: " 
//         << print_container_dB(&fftBins[0], &fftBins[nBins]) << endl;

    // Averages across bins belonging to the same channel
    for(unsigned int i = 0; i < nBins; i++) 
    {
        if(bin_mask[i] >= 0)
            tmp_ch_power[bin_mask[i]] += fftBins[i].real()*fftBins[i].real() + fftBins[i].imag()*fftBins[i].imag();//norm(fftBins[i]);
    }
    
    // Adds the channel average power to the moving average
    for(uint16_t j = 0; j < ch_pwr_ma.size(); j++) 
    {
        ch_pwr_ma[j].push(tmp_ch_power[j] * ch_avg_coeff[j]);
        //ch_pwr_ma_last_outputs[j].push(ch_pwr_ma[j].get_avg());
    }
    
    // If a big enough step was done in the moving average, send the average values to the results buffer
    if(++mavg_count == mavg_step_size) 
    {
        buffer_utils::wdataset<ChPowers> d;
        results.get_wdataset(d);
        d().first = tstamp;
        d().second.resize(ch_pwr_ma.size());
        
        for(uint16_t j = 0; j < ch_pwr_ma.size(); j++) 
        {
            d().second[j] = ch_pwr_ma[j].get_avg();//ch_pwr_ma_last_outputs[j].max(); 
        }
                
        mavg_count = 0;
    }
}

bool ChannelPowerEstimator::try_pop_result(double &tstamp, std::vector<float> &vec)
{
    if(result_exists())
    {
        buffer_utils::rdataset<ChPowers> d;
        results.get_rdataset(d);
        vec = d().second; // NOTE: should I copy?
        tstamp = d().first;
        return true;
    }
    return false;
}

void ChannelPowerEstimator::pop_result(buffer_utils::rdataset<ChPowers> &d)
{
    results.get_rdataset(d);
}
