/* 
 * File:   NoiseFilter2.h
 * Author: ctvr
 *
 * Created on August 31, 2015, 5:05 PM
 */

#ifndef NOISEFILTER2_H
#define	NOISEFILTER2_H

#include "KHMO.h"
#include "stats.h"
#include <boost/format.hpp>
#include <sstream>

class NoiseFilter2 
{
public:
    NoiseFilter2(uint16_t _Nch, float _alpha = 0.001, float _thres = 8) : Nch(_Nch), min_limit(1e-9) // thres is in dB!
    {
        clusterizers.assign(Nch, KHMO(2, 3, _alpha, pow(10,_thres/10)));
        noise_hits_stats.resize(Nch,rate_stats());
    }
    void filter(std::vector<float> &ch_pwr)
    { // Warning: may change ch_pwr
        
        size_t clus_idx, noise_idx;
        for (register uint16_t i = 0; i < Nch; i++)
        {
            if(ch_pwr[i] > min_limit)               // cut off the NaN crap to not skew average
                clus_idx = clusterizers[i].push(ch_pwr[i]);
            else
            {
                ch_pwr[i] = 0;
                continue;
            }
            
            noise_idx = ch_noise_floor_idx(i);
            
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
        for(const auto&ch : clusterizers)
        {
            sum += std::min(ch.clusters[0].mk, ch.clusters[1].mk);
        }
        return sum / Nch;
    }
    inline float ch_noise_floor(uint16_t idx) 
    {
        return (clusterizers[idx].clusters.size() > 0) ? std::min_element(clusterizers[idx].clusters.begin(), clusterizers[idx].clusters.end(), 
        [](const KHMO::KHMOCluster &a, const KHMO::KHMOCluster &b){
            return a.mk < b.mk;
        })->mk : 0.0;
    }
    inline float ch_sig_power(uint16_t idx) 
    {
        return (clusterizers[idx].clusters.size() > 0) ? std::max_element(clusterizers[idx].clusters.begin(), clusterizers[idx].clusters.end(), 
        [](const KHMO::KHMOCluster &a, const KHMO::KHMOCluster &b){
            return a.mk < b.mk;
        })->mk : 0.0;
    }
    inline int ch_noise_floor_idx(uint16_t idx)
    {
        return (clusterizers[idx].clusters.size() > 0) ? std::distance(clusterizers[idx].clusters.begin(), std::min_element(clusterizers[idx].clusters.begin(), clusterizers[idx].clusters.end(), 
        [](const KHMO::KHMOCluster &a, const KHMO::KHMOCluster &b){
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
    
    std::vector<KHMO> clusterizers;
    double min_limit;
private:
    uint16_t Nch;
    float thres;
    std::vector<rate_stats> noise_hits_stats;
};

#endif	/* NOISEFILTER2_H */
