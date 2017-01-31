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

#include <complex>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "fftw3.h"
#include <boost/cstdint.hpp>
//#include <boost/math/distributions/poisson.hpp>
#include <sstream>
//#include "NoiseFilter3.h"
#include "stats.h"
#include "buffer_utils.hpp"

using buffer_utils::bounded_buffer;

typedef std::complex<float> Cplx;
typedef std::vector<std::complex<float> > CplxVec;
typedef std::vector<std::complex<float> >::iterator CplxVecIt;
typedef std::vector<std::complex<float> >::const_iterator CplxVecConstIt;

typedef std::pair<double, std::vector<float> > ChPowers;        ///< timestamp + line of powers

#ifndef CHANNELPOWERESTIMATOR_HPP
#define CHANNELPOWERESTIMATOR_HPP

class ChannelPowerEstimator 
{
    fftwf_plan fft;                        ///< Our FFT object pointer.
    uint16_t bin_idx;

    uint16_t Nch;
    uint16_t mavg_size;
    uint16_t mavg_step_size;
    uint16_t mavg_count;

    std::vector<int> bin_mask;
    std::vector<float> ch_avg_coeff;
    std::vector<float> tmp_ch_power;
    
    std::vector< MovingAverage<double> > ch_pwr_ma;
    //std::vector<MovingWindowMax> ch_pwr_ma_last_outputs;

    bounded_buffer<ChPowers> results;
    
    
public:
    //std::unique_ptr<NoiseFilter3> noise_filter;
    Cplx* fftBins;                       ///< Allocated using fftwf_malloc (SIMD aligned)
    uint16_t nBins;
    
    ChannelPowerEstimator();
    ~ChannelPowerEstimator()
    {
        destroy();
    }

    void set_parameters(uint16_t _avg_win_size, uint16_t num_channels, const std::vector<int> &_bin_mask);
    void set_parameters(uint16_t _avg_win_size, uint16_t fftsize, uint16_t num_channels);
    void setup();
    void destroy();
    void push_samples(const std::vector<Cplx> &vec);
    void push_sample(Cplx val);
    void process(double tstamp = 0);
    inline bool result_exists() 
    {
        return results.empty() == false;
    }
    Cplx& operator[](int idx) 
    {
        assert(idx >= 0 && idx < nBins);
        return fftBins[idx];
    }
    inline uint16_t fft_size() const {return nBins;}
    void pop_result(buffer_utils::rdataset<ChPowers> &d);   // WARNING: with no move semantics I have to use shared_ptr to avoid mem leaks
    bool try_pop_result(double &tstamp, std::vector<float> &vec);
};

#endif