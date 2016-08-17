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

#include "EnergyDetector2.h"

using namespace std;
//using namespace dyspan;

EnergyDetector2::EnergyDetector2() : fftBins(NULL), fft(NULL)
{
}

void EnergyDetector2::set_parameters(uint16_t _ma_step_size, uint16_t fftsize, uint16_t num_channels, float bin_mask_select_perc, float bin_mask_ref_perc) 
{
    mavg_size = 160;
    mavg_step_size = _ma_step_size;
    Nch = num_channels;
    nBins = fftsize;
    
    // Define the bin mask
    bin_mask.resize(nBins,-1);
    bin_mask_ref.resize(nBins,-1);
    
    float half_ch_reject_perc = (1 - bin_mask_select_perc) / 2;
    int a, Nbins_per_ch = nBins/Nch;
    int half_ch_reject_bins = half_ch_reject_perc*Nbins_per_ch, half_ch_ref_bins = bin_mask_ref_perc * Nbins_per_ch / 2;
    
    for(int i = 0; i< nBins; i++)
    {
        a = i/Nbins_per_ch;

        if (i >= half_ch_reject_bins+(Nbins_per_ch*a) && i < (Nbins_per_ch-half_ch_reject_bins+Nbins_per_ch*a))
            bin_mask[i] = (a + Nch/2) % Nch;
        if(i < (Nbins_per_ch*a + half_ch_ref_bins) || i >= (Nbins_per_ch*a + Nbins_per_ch - half_ch_ref_bins))
            bin_mask_ref[i] = (a + Nch/2) % Nch;
    }
    
    // Cancel DC Offset
    bin_mask[nBins-1] = -1;
    bin_mask[0] = -1;
    bin_mask[1] = -1;
    
    std::cout << "Bin mask: [" ;
    for(int i = 0; i < bin_mask.size() - 1; ++i)
        cout << bin_mask[i] << ", ";
    cout << bin_mask[bin_mask.size()-1] << "]\n";
    
    std::cout << "Ref bin mask: [" ;
    for(int i = 0; i < bin_mask_ref.size() - 1; ++i)
        cout << bin_mask_ref[i] << ", ";
    cout << bin_mask_ref[bin_mask_ref.size()-1] << "]\n";
    
    setup();
}

void EnergyDetector2::setup() 
{
    // Create new FFT
    destroy();
    fftBins = reinterpret_cast<Cplx*>(fftwf_malloc(sizeof(fftwf_complex) * nBins));
    fft = fftwf_plan_dft_1d(nBins, (fftwf_complex*)fftBins, (fftwf_complex*)fftBins, FFTW_FORWARD, FFTW_MEASURE);
    bin_idx = 0;
    
    // Create Moving Averages
    ch_pwr_ma.resize(Nch, MovingAverage<double>(mavg_size));
    ch_pwr_ma_last_outputs.resize(Nch, MovingWindowMax(mavg_size*1.5));   // previous moving average outputs to compensate 50% duty cycle
    
    tmp_ch_power.resize(Nch);
    ch_avg_coeff.resize(Nch, 1);
    tmp_ch_power_ref.resize(Nch);
    ch_avg_coeff_ref.resize(Nch, 1);
    mavg_count = mavg_step_size-mavg_size; // let the mavg fill completely the first time
    
    // Count number of bins belonging to each channel
    vector<int> ch_count(Nch,0), ch_ref_count(Nch,0);
    for(unsigned int i = 0; i < nBins; i++) 
    {
        if(bin_mask[i] >= 0)
            ch_count[bin_mask[i]]++;
        if(bin_mask_ref[i] >= 0)
            ch_ref_count[bin_mask_ref[i]]++;
    }
    for(int j = 0; j < Nch; j++)
    {
        ch_avg_coeff[j] = 1/((float)ch_count[j] * nBins);
        ch_avg_coeff_ref[j] = 1/((float)ch_ref_count[j] * nBins);
    }

    // create noise filter
    noise_filter.reset(new NoiseFilter3(Nch, 0.001, 7));
}

void EnergyDetector2::destroy() 
{
    if(fftBins != NULL)
        fftwf_free(fftBins);
    if(fft != NULL)
        fftwf_destroy_plan(fft);
}

void EnergyDetector2::push_samples(const vector<Cplx> &vec) 
{
    for(uint32_t i = 0; i < vec.size(); i++) 
    {
        fftBins[bin_idx] = vec[i];
        if(++bin_idx == nBins) 
        {
            process();
            bin_idx = 0;
        }
    }
}

void EnergyDetector2::push_sample(Cplx val) 
{
    fftBins[bin_idx] = val;
    if (++bin_idx == nBins) 
    {
        process();
        bin_idx = 0;
    }
}

void EnergyDetector2::process(double tstamp) 
{
    //std::cout << "\nfft input: [";
    //for_each(&fftBins[0], &fftBins[nBins], [](Cplx f){std::cout << 10*log10(std::norm(f)) << ", ";});
    //std::cout << "] " << std::endl;
    
    fftwf_execute(fft);
    tmp_ch_power.assign(Nch, 0);
    tmp_ch_power_ref.assign(Nch, 0);
    
    //std::cout << "\nfft output: [";
    //for_each(&fftBins[0], &fftBins[nBins], [](Cplx f){std::cout << 10*log10(std::norm(f)) << ", ";});
    //std::cout << "] " << std::endl;

    // Averages across bins belonging to the same channel
    for(unsigned int i = 0; i < nBins; i++) 
    {
        if(bin_mask[i] >= 0)
            tmp_ch_power[bin_mask[i]] += fftBins[i].real()*fftBins[i].real() + fftBins[i].imag()*fftBins[i].imag();//norm(fftBins[i]);
        if(bin_mask_ref[i] >= 0)
            tmp_ch_power_ref[bin_mask_ref[i]] += fftBins[i].real()*fftBins[i].real() + fftBins[i].imag()*fftBins[i].imag();
    }
    
    // Adds the channel average power relative to the ref to the moving average
    // Adds a sample of the output of the moving average to the ch_pwr_ma_last_outputs
    for(uint16_t j = 0; j < ch_pwr_ma.size(); j++) 
    {
        ch_pwr_ma[j].push(tmp_ch_power[j] * ch_avg_coeff[j] / (tmp_ch_power_ref[j] * ch_avg_coeff_ref[j]));
        ch_pwr_ma_last_outputs[j].push(ch_pwr_ma[j].get_avg());
    }
    
    // If a big enough step was done in the moving average, send the average values to the results deque
    if(++mavg_count == mavg_step_size) 
    {
        vector<float> ch_pwr(ch_pwr_ma_last_outputs.size());
        for(uint16_t j = 0; j < ch_pwr_ma_last_outputs.size(); j++) 
        {
            ch_pwr[j] = ch_pwr_ma_last_outputs[j].max();
        }
        //noise_filter->filter(ch_pwr);
        results.emplace(tstamp, std::move(ch_pwr));
        mavg_count = 0;
    }
}

void EnergyDetector2::pop_result(double &tstamp, std::vector<float> &vec) 
{
    if(result_exists()) {
        vec = std::move(results.front().second);
        tstamp = results.front().first;
        results.pop();
    }
    else {
        tstamp = -1;
        vec.resize(0);
    }
    return;
}
