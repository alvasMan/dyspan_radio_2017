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

#ifndef NOISEFILTER3_H
#define	NOISEFILTER3_H

#include "KHMO2.h"
#include "stats.h"
#include <boost/format.hpp>
#include <sstream>

class NoiseFilter3 
{
public:
    NoiseFilter3(uint16_t _Nch, float _alpha = 0.001, float _thres = 7) : Nch(_Nch), min_limit(1e-9) // thres is in dB!
    {
        clusterizers.assign(Nch, KHMO2(2, 2, _alpha, _thres));
        noise_hits_stats.resize(Nch,rate_stats());
    }
    void filter(std::vector<float> &ch_pwr)
    { // Warning: may change ch_pwr
        
        size_t clus_idx, noise_idx;
        for (register uint16_t i = 0; i < Nch; i++)
        {
            double dB_val = 10*log10(ch_pwr[i]);
            if(ch_pwr[i] > min_limit)               // cut off the NaN crap to not skew average
            {
                clus_idx = clusterizers[i].push(dB_val);
            }
            else
            {
                ch_pwr[i] = 0;
                continue;
            }
            
            noise_idx = ch_noise_floor_idx(i);
            //if(clus_idx == noise_idx && 7 < (dB_val - ch_noise_floor(i)))
            //    clus_idx = (clus_idx + 1) % 2;
            
            if(clus_idx == noise_idx)    // if the new sample is just noise
            {
                ch_pwr[i] = 0;
                noise_hits_stats[i].miss();
            }
            else
            {
                noise_hits_stats[i].hit();
            }
        }
    }
    
    void reset() 
    {
        for (uint16_t i = 0; i < Nch; i++) 
        {
            clusterizers[i].clusters.clear();
            noise_hits_stats[i].reset();
        }
    }
    void set_thres(float _thres) 
    {
        thres = _thres;
        reset();
    }
    double estimated_noise_floor()
    {
        double sum = 0;
        for(int i = 0; i < clusterizers.size(); ++i)
        {
            sum += ch_noise_floor(i);
        }
        return sum / Nch;
    }
    inline float ch_noise_floor(uint16_t idx) 
    {
        return (clusterizers[idx].clusters.size() > 0) ? std::min_element(clusterizers[idx].clusters.begin(), clusterizers[idx].clusters.end(), 
        [](const KHMO2::KHMOCluster &a, const KHMO2::KHMOCluster &b){
            return a.mk < b.mk;
        })->mk : -90;
    }
    inline float ch_sig_power(uint16_t idx) 
    {
        return (clusterizers[idx].clusters.size() > 0) ? std::max_element(clusterizers[idx].clusters.begin(), clusterizers[idx].clusters.end(), 
        [](const KHMO2::KHMOCluster &a, const KHMO2::KHMOCluster &b){
            return a.mk < b.mk;
        })->mk : -90;
    }
    inline int ch_noise_floor_idx(uint16_t idx)
    {
        return (clusterizers[idx].clusters.size() > 0) ? std::distance(clusterizers[idx].clusters.begin(), std::min_element(clusterizers[idx].clusters.begin(), clusterizers[idx].clusters.end(), 
        [](const KHMO2::KHMOCluster &a, const KHMO2::KHMOCluster &b){
            return a.mk < b.mk;
        })) : -1;
    }
    inline float ch_detec_rate(uint16_t idx) 
    {
        return (noise_hits_stats[idx].val_count > 0) ? noise_hits_stats[idx].get_rate() : 0;
    }
    std::string print_ch_pdetec() 
    {
        std::stringstream ss;
        for (int i = 0; i < Nch; i++) {
            ss << boost::format("%d: %.4f") % i % (ch_detec_rate(i) * 100) << "%\t";
        }
        ss << "\n";
        return ss.str();
    }
    
    std::vector<KHMO2> clusterizers;
    double min_limit;
private:
    uint16_t Nch;
    float thres;
    std::vector<rate_stats> noise_hits_stats;
};

#endif	/* NOISEFILTER3_H */
