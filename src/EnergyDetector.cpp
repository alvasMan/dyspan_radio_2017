#include "EnergyDetector.h"

using namespace std;
//using namespace dyspan;

EnergyDetector::EnergyDetector() : fftBins(NULL), fft(NULL), noise_filter(NULL) {
}

void EnergyDetector::set_parameters(uint16_t _avg_win_size, uint16_t num_channels, uint16_t fftsize) {
    mavg_size = _avg_win_size;
    Nch = num_channels;
    nBins = fftsize;
    
    // Define the bin mask
    bin_mask.resize(fftsize);
    float bins_per_channel = nBins / (float)Nch;
    for(int i = 0; i < nBins; i++) {
        bin_mask[i] = floor(i / bins_per_channel);
    }
    
    setup();
}

void EnergyDetector::set_parameters(uint16_t _avg_win_size, uint16_t num_channels, const std::vector<int> &_bin_mask) {
    mavg_size = _avg_win_size;
    Nch = num_channels;
    bin_mask = _bin_mask;       // Values in bin_mask must be within [0, number of channels] and size of bin_mask must be equal to nBins. No protection yet.
    nBins = bin_mask.size();

    setup();
}

void EnergyDetector::setup() {
    // Create new FFT
    destroy();
    fftBins = reinterpret_cast<Cplx*>(fftwf_malloc(sizeof(fftwf_complex) * nBins));
    fft = fftwf_plan_dft_1d(nBins, (fftwf_complex*)fftBins, (fftwf_complex*)fftBins, FFTW_FORWARD, FFTW_MEASURE);
    bin_idx = 0;
    
    // Create Moving Averages
    ch_pwr_ma.resize(Nch, MovingAverage<float>(mavg_size));
    tmp_ch_power.resize(Nch);
    ch_avg_coeff.resize(Nch, 1);
    mavg_step_size = mavg_size;
    mavg_count = 0;
    
    // Count number of bins belonging to each channel
    vector<int> ch_count(Nch,0);
    for(unsigned int i = 0; i < nBins; i++) {
        ch_count[bin_mask[i]]++;
    }
    for(int j = 0; j < Nch; j++)
        ch_avg_coeff[j] = 1/((float)ch_count[j] * nBins);
    
    // create noise_filter
    if(noise_filter != NULL)
        delete noise_filter;
    // threshold is equal to lamb / (number of samples used in each channel energy value)
    int Me = ch_count[0] * mavg_size, count = 0;
    float val, thr = 1, dist;
    do {
    	boost::math::poisson pois(thr * Me); // poisson distribution with lambda
    	val = boost::math::quantile(pois, 0.01) / Me; // Pfa = 0.01
    	dist = 1 - val;
    	thr += dist;
    	++count;
    	if(count > 100) {
    		throw "Couldn't converge to a threshold value!";
    		break;
    	}
    } while(dist > 0.0001);


    // check: http://www.boost.org/doc/libs/1_35_0/libs/math/doc/sf_and_dist/html/math_toolkit/dist/dist_ref/nmp.html#math.dist.quantile
    noise_filter = new NoiseFilter(Nch, thr);
}

void EnergyDetector::destroy() {
    if(fftBins != NULL)
        fftwf_free(fftBins);
    if(fft != NULL)
        fftwf_destroy_plan(fft);
    if(noise_filter != NULL)
        delete noise_filter;
}

void EnergyDetector::push_samples(const vector<Cplx> &vec) {
    for(unsigned int i = 0; i < vec.size(); i++) {
        fftBins[bin_idx] = vec[i];
        if(++bin_idx == nBins) {
            process();
            bin_idx = 0;
        }
    }
}

void EnergyDetector::push_sample(Cplx val) {
    fftBins[bin_idx] = val;
    if (++bin_idx == nBins) {
        process();
        bin_idx = 0;
    }
}

void EnergyDetector::process(double tstamp) {
    
    
    fftwf_execute(fft);
    tmp_ch_power.assign(ch_pwr_ma.size(), 0);

    // Averages across bins belonging to the same channel
    for(unsigned int i = 0; i < nBins; i++) {
        tmp_ch_power[bin_mask[i]] += fftBins[i].real()*fftBins[i].real() + fftBins[i].imag()*fftBins[i].imag();//norm(fftBins[i]);
    }
    
    // Adds the channel average power to the moving average
    for(uint16_t j = 0; j < ch_pwr_ma.size(); j++) {
        ch_pwr_ma[j].push(tmp_ch_power[j] * ch_avg_coeff[j]);
    }
    
    // If a big enough step was done in the moving average, send the average values to the results deque
    if(++mavg_count == mavg_step_size) {
        vector<float> ch_pwr(ch_pwr_ma.size());
        for(uint16_t j = 0; j < ch_pwr_ma.size(); j++) {
            ch_pwr[j] = ch_pwr_ma[j].get_avg();
        }
        noise_filter->filter(ch_pwr);
        results.push_back(std::pair<double, vector<float> >(tstamp, ch_pwr));
        mavg_count = 0;
    }
}

void EnergyDetector::pop_result(double &tstamp, std::vector<float> &vec) {
    if(result_exists()) {
        vec = results.front().second;
        tstamp = results.front().first;
        results.pop_front(); // this does not call vec destructor
    }
    else {
        tstamp = -1;
        vec.resize(0);
    }
    return;
}

NoiseFilter::NoiseFilter(uint16_t _Nch, float _thres) : Nch(_Nch), thres(_thres) {
    tmp_sort_idx.resize(Nch);
    for(uint16_t i = 0; i < Nch; ++i) {
    	tmp_sort_idx[i] = i;
    }
#ifdef NOISE_STATS
    noise_ch_pwr_stats.resize(Nch,val_stats());
    noise_hits_stats.resize(Nch,rate_stats());
#endif
    min_noise_count = 1;
}

void NoiseFilter::filter(std::vector<float> &ch_pwr) { // Warning: may change ch_pwr
    get_idx_sort_op.assign(ch_pwr);
    sort(tmp_sort_idx.begin(), tmp_sort_idx.end(), get_idx_sort_op);
    
    register uint16_t i = 0;
    if (noise_pwr_stats.val_count < min_noise_count) {    // until it stabilizes
        filter_as_noise(ch_pwr, tmp_sort_idx[i]);
        i++;
    }

    for (; i < Nch; i++) {
        if (ch_pwr[tmp_sort_idx[i]] < thres * noise_pwr_stats.get_avg()) {
            filter_as_noise(ch_pwr, tmp_sort_idx[i]);
        }
        else {
#ifdef NOISE_STATS
            for(register uint16_t j = i; j < Nch; j++)
                noise_hits_stats[tmp_sort_idx[j]].hit();
#endif
            break;
        }
    }
}

float time_thres;
inline bool time_thres_test(double example_time, double new_time) {
    return abs(new_time - example_time) / example_time < time_thres;
}

inline TOAFitStruct fit_dwell_test(double TOA1, double TOA2, double real_dwell) {
    double diff = TOA2 - TOA1;
    int k = round(diff / real_dwell); // last_TOA
    if (k == 0)
        return TOAFitStruct(KBELOW, k, diff);
    diff = diff / k;
    if(time_thres_test(real_dwell, diff))
        if(abs(k) <= 4)
            return TOAFitStruct(TOA_VALID, k, diff);
        else
            return TOAFitStruct(KOVER, k, diff);
    
    return TOAFitStruct(TOA_INVALID, k, diff);
}

inline bool toa_seq_compare(const TOASequence &t1, const TOASequence &t2) {
    if(time_thres_test(t1.dwelltime, t2.dwelltime) == false)
        return false;
    TOAFitStruct tfs = fit_dwell_test(t1.last_TOA, t2.last_TOA, t1.dwelltime);
    if(tfs.test_ret == TOA_VALID || tfs.test_ret == KOVER)
        return true;
    else if(tfs.test_ret == KBELOW && time_thres_test(t1.last_TOA, t2.last_TOA))
        return true;
    return false;
}

TOAFitStruct TOASequence::fit_test(double TOA) {
    return fit_dwell_test(last_TOA, TOA, dwelltime);
}

TOA_FIT TOASequence::push_if_fits(double TOA) {
    TOAFitStruct ret = fit_test(TOA);
    if(ret.test_ret == TOA_VALID) {
        last_TOA = TOA;
        push_dwelltime(ret.dwell);
        k_skip_sum += (ret.k-1);
        ++k_skip_count;
    } else if(ret.test_ret == KOVER) {
        
        if(penalty() > 2.0) // it is removed
            ret.test_ret = SEQ_REMOVAL;
        else {// It is not considered in the dwell time and a penalty is added
            last_TOA = TOA;
            k_skip_sum += 5; // penalty
            ++k_skip_count;
        }
    }
    return ret.test_ret;
}

DwellTimeEstimator::DwellTimeEstimator(uint16_t _Nch, double _max_dwelltime, float _time_percent_thres)
: Nch(_Nch), max_dwelltime(_max_dwelltime), cur_ch(-1), transition_tstamp_cache(3) {
    time_thres = _time_percent_thres;
}

void DwellTimeEstimator::set_cur_ch(double tstamp, uint16_t idx) {
    if(cur_ch < 0) { // first time
        cur_ch = idx;
        transition_tstamp_cache.push(make_pair(tstamp, idx));
        return;
    }
    
    if(cur_ch != idx) { // if the channel changed
        cur_ch = idx;
        bool test = false;
        TOA_FIT fit_test;
        DwellListIt prev_it;
        for(DwellListIt it = dwell_list.begin(); it != dwell_list.end(); ) {
            fit_test = it->push_if_fits(tstamp);
            if(fit_test == TOA_VALID && it->penalty() < 1.0)
                test = true;
            else if(fit_test == SEQ_REMOVAL) { // this is a crappy TOAsequence and needs to be removed
                prev_it = it++;
                dwell_list.erase(prev_it);
                continue;
            }
            ++it;
        }
        if(test == false) { // the insertion in the already existent TOAsequences was not successful
            double dwell;
            for(uint32_t i = 0; i < transition_tstamp_cache.size(); ++i) {
                const std::pair<double, uint16_t>&transition = transition_tstamp_cache.get_val(i);
                if(transition.second == idx) // we know for sure this is not a appropriate dwell time estimation
                    continue;
                dwell = tstamp - transition.first;
                if(dwell < max_dwelltime) {
                    TOASequence tseq(tstamp, dwell);
                    DwellListIt it;
                    for(it = dwell_list.begin(); it != dwell_list.end(); ++it) {
                        if(toa_seq_compare(*it, tseq) == true)
                            break;
                    }
                    if(it == dwell_list.end())
                        dwell_list.push_back(TOASequence(tstamp, dwell));
                }
            }
        }
        transition_tstamp_cache.push(make_pair(tstamp, idx));
    }
}

void DwellTimeEstimator::process(double tstamp, std::vector<float> &ch_pwr) {
    std::vector<float>::iterator max_el_it = std::max_element(ch_pwr.begin(), ch_pwr.end());
    
    if(*max_el_it > 0) { // if it was not yet initialized and the first channel is not noise
        set_cur_ch(tstamp, max_el_it - ch_pwr.begin());
    }
    
    /*while(!transition_tstamp_cache.empty()) { // remove old tstamps in the cache
        if(transition_tstamp_cache.begin()->first + max_dwelltime > tstamp)
            break;
        transition_tstamp_cache.erase(transition_tstamp_cache.begin());
    }*/
}

DwellListIt DwellTimeEstimator::get_dwelltime_it() {
    DwellListIt it = dwell_list.begin(), pref_it = it;
    if(pref_it == dwell_list.end())
        return pref_it;
    
    while(++it != dwell_list.end()) {
        float penalty_diff = pref_it->num_hits()/((1+pref_it->penalty())*(1+pref_it->penalty())) - it->num_hits()/((1+it->penalty())*(1+it->penalty()));
        if(penalty_diff < 0 || pref_it->num_hits() < 3) { // gives priority to low penalty and low dwell time
            // erase those dwell times that are just harmonics of the preferred one
            TOAFitStruct ret = fit_dwell_test(it->last_TOA, pref_it->last_TOA, it->dwelltime);
            if(it->num_hits() > 10 && (ret.test_ret == TOA_VALID || ret.test_ret == KOVER)) {
                dwell_list.erase(pref_it);
            }
            pref_it = it;
        }
    }
    
    return pref_it;
}

//////////////////////////////////////////////////////////////////////////////

double PulseDetector::push_sample(double tstamp, float val) {
    double ret = -1;
    if(val > 0) {
        if(state == false) {
            state = true;
            pulse_tstamp = tstamp;
        }
        count_skip = 0;
        pulse_buffer.push_back(val);
    }
    else {
        if(state == true) {
            ++count_skip;
            if(count_skip > max_count_skip) {
                ret = tstamp - pulse_tstamp;
                state = false;
                pulse_buffer.clear();
            }
        }
    }
    return ret;
}

double DwellTimeEstimator2::dwelltime() {
    double max_dwell = 0;
    int count_hits = 0;

    for(std::list<val_stats>::iterator it = dwelltimes_list.begin(); it != dwelltimes_list.end(); ++it) {
        if(count_hits < it->val_count) {
            max_dwell = it->get_avg();
            count_hits = it->val_count;
        }
    }
    return max_dwell;
}

DwellTimeEstimator2::DwellTimeEstimator2(uint16_t _Nch, double _max_dwelltime, float _time_percent_thres)
: Nch(_Nch), max_dwelltime(_max_dwelltime) {
    time_thres = _time_percent_thres;
    pulse_detector_list.resize(Nch);
}

void DwellTimeEstimator2::process(double tstamp, std::vector<float> &ch_pwr) {
    double dur;
    for(int i = 0; i < Nch; ++i) {
        dur = pulse_detector_list[i].push_sample(tstamp, ch_pwr[i]);
        if(dur <= 0) continue;
        std::list<val_stats>::iterator it;
        for(it = dwelltimes_list.begin(); it != dwelltimes_list.end(); ++it) {
            if(time_thres_test(it->get_avg(), dur)== true) {
                it->push(dur);
                break;
            }
        }
        if(it == dwelltimes_list.end()) {
            val_stats val;
            val.push(dur);
            dwelltimes_list.push_back(val);
        }
    }
}
