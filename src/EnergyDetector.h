#include <complex>
#include <vector>
#include <list>
#include <map>
#include <deque>
#include <boost/shared_ptr.hpp>
#include "fftw3.h"
#include <boost/cstdint.hpp>
#include <boost/math/distributions/poisson.hpp>
#include <sstream>
#include "NoiseFilter2.h"


typedef std::complex<float> Cplx;
typedef std::vector<std::complex<float> > CplxVec;
typedef std::vector<std::complex<float> >::iterator CplxVecIt;
typedef std::vector<std::complex<float> >::const_iterator CplxVecConstIt;

class NoiseFilter;

class EnergyDetector {
    fftwf_plan fft;                        ///< Our FFT object pointer.
    uint16_t bin_idx;

    uint16_t Nch;
    uint16_t mavg_size;
    uint16_t mavg_step_size;
    uint16_t mavg_count;

    std::vector<float> tmp_ch_power;
    std::vector<float> ch_avg_coeff;
    std::vector<int> bin_mask;
    std::vector<MovingAverage<double> > ch_pwr_ma;

    std::deque<std::pair<double, std::vector<float> > > results;
    
    
public:
    NoiseFilter2 *noise_filter;
    Cplx* fftBins;                       ///< Allocated using fftwf_malloc (SIMD aligned)
    uint16_t nBins;
    
    EnergyDetector();
    ~EnergyDetector()
    {
        destroy();
    }

    void set_parameters(uint16_t _avg_win_size, uint16_t num_channels, const std::vector<int> &_bin_mask);
    void set_parameters(uint16_t _avg_win_size, uint16_t num_channels, uint16_t fftsize);
    void set_parameters(uint16_t _avg_win_size, uint16_t num_channels, uint16_t fftsize, double bin_mask_per);
    void setup();
    void destroy();
    void push_samples(const std::vector<Cplx> &vec);
    void push_sample(Cplx val);
    void process(double tstamp = 0);
    inline bool result_exists() {
        return results.empty() == false;
    }
    void pop_result(double &tstamp, std::vector<float> &vec);   // WARNING: with no move semantics I have to use shared_ptr to avoid mem leaks
};

#define DETECT_MAXPWR
#define NOISE_STATS

struct sort_idx_op {
    float const *vec_ptr;
    inline void assign(const std::vector<float> &vec) {
        vec_ptr = &vec[0];
    }
    bool operator()(const uint16_t &left, const uint16_t &right) {
        return vec_ptr[left] < vec_ptr[right];
    }
};

/**
 * Class to decide if a certain channel is occupied or not. If vacant, the energy of the channel is set to zero (in filter function)
 * Note: if NOISE_STATS defined, the class also estimates each channel noise floor and detection rate
 */
class NoiseFilter {
    uint16_t Nch;
    std::vector<exp_stats> noise_ch_pwr_stats;
    unsigned int min_noise_count;
    
    float thres;
    bool noise_estim_mode;
    
#ifdef NOISE_STATS
    std::vector<rate_stats> noise_hits_stats;
#endif
    
    inline void filter_as_noise(std::vector<float> &ch_pwr, uint16_t idx) {
#ifdef NOISE_STATS
        noise_hits_stats[idx].miss();
#endif
        if(noise_estim_mode == true && ch_pwr[idx] > 1e-10) // cut off some NaN crap you may get
            noise_ch_pwr_stats[idx].push(ch_pwr[idx]);
        ch_pwr[idx] = 0;
    }
    
public:
    NoiseFilter(uint16_t _Nch, float _thres = 1.3);
    void reset() {
#ifdef NOISE_STATS
        for (uint16_t i = 0; i < Nch; i++) {
            noise_ch_pwr_stats[i].reset();
            noise_hits_stats[i].reset();
        }
#endif
    }
    void set_thres(float _thres) {
        thres = _thres;
        reset();
    }
    void filter(std::vector<float> &ch_pwr);
    float estimated_noise_floor();
    inline float ch_noise_floor(uint16_t idx) {
        return (noise_ch_pwr_stats[idx].val_count > 0) ? noise_ch_pwr_stats[idx].get_avg() : 0;
    }
    void set_static_noise_floor(const std::vector<float> &noise_floor);
#ifdef NOISE_STATS
    inline float ch_detec_rate(uint16_t idx) {
        return (noise_hits_stats[idx].val_count > 0) ? noise_hits_stats[idx].get_rate() : 0;
    }
    std::string print_ch_pdetec() {
        std::stringstream ss;
        for (int i = 0; i < Nch; i++) {
            ss << boost::format("%d: %.4f") % i % (ch_detec_rate(i) * 100) << "%\t";
        }
        ss << "\n";
        return ss.str();
    }
#endif
};

template<typename T>
std::string print_vector_dB(const std::vector<T> &vec) {
    std::stringstream ss;
    for (int i = 0; i < vec.size(); i++) {
        ss << boost::format("%d: %1.11f dB\t") % i % (10 * log10(vec[i]));
    }
    ss << "\n";
    return ss.str();
}

/////////////////////////////////////////////////////////
//////////////// DWELL TIME ESTIMATION //////////////////
/////////////////////////////////////////////////////////

enum TOA_FIT {KBELOW, KOVER, TOA_VALID, TOA_INVALID, SEQ_REMOVAL};

struct TOAFitStruct {
    TOA_FIT test_ret;
    int k;
    double dwell;
    TOAFitStruct(TOA_FIT _ret, int _k, double _dwell) : test_ret(_ret), k(_k), dwell(_dwell) {}
};

class TOASequence {
    val_stats dwell_estim;
    uint32_t k_skip_sum;
    uint32_t k_skip_count;
public:
    double dwelltime;
    double last_TOA;
    
    TOASequence(double _last_TOA, double _dwelltime) : last_TOA(_last_TOA), k_skip_count(0), k_skip_sum(0) {
        push_dwelltime(_dwelltime);
    }
    inline float penalty() {
        return (k_skip_count == 0) ? 0 : ((float)k_skip_sum) / k_skip_count;
    }
    inline void push_dwelltime(double _dwelltime) {
        dwell_estim.push(_dwelltime);
        dwelltime = dwell_estim.get_avg();
    }
    inline uint32_t num_hits() const {
        return dwell_estim.val_count;
    }
    TOAFitStruct fit_test(double TOA);
    TOA_FIT push_if_fits(double TOA);
    inline bool operator<(const TOASequence &t) const {return num_hits() >= t.num_hits();}
};

typedef std::list<TOASequence>::iterator DwellListIt;

class DwellTimeEstimator {
    //param
    uint16_t Nch;
    double max_dwelltime;
    
    std::list<TOASequence> dwell_list;
    CircularBuffer<std::pair<double, uint16_t> > transition_tstamp_cache;
    
    DwellListIt get_dwelltime_it();
public:
    int16_t cur_ch;
    
    DwellTimeEstimator(uint16_t _Nch, double _max_dwelltime, float _time_percent_thres);
    void set_cur_ch(double tstamp, uint16_t idx);
    void process(double tstamp, std::vector<float> &ch_pwr);
    inline double dwelltime() {
        DwellListIt pref_it = get_dwelltime_it();
        if(pref_it != dwell_list.end())
            return pref_it->dwelltime;
        else
            return 0;
    }
    inline double time_of_transition() {
        DwellListIt pref_it = get_dwelltime_it();
        if(pref_it != dwell_list.end())
            return pref_it->last_TOA + pref_it->dwelltime;
        else
            return 0;
    }
};

class PulseDetector {
    bool state;
    std::vector<float> pulse_buffer;
    double pulse_tstamp;
    uint16_t count_skip;
    uint16_t max_count_skip;

public:
    double push_sample(double tstamp, float val);
    PulseDetector() : state(false), pulse_tstamp(0), max_count_skip(1), count_skip(0) {
    }
};

class DwellTimeEstimator2 {
    //param
    uint16_t Nch;
    double max_dwelltime;

    std::vector<PulseDetector> pulse_detector_list;
    std::list<val_stats> dwelltimes_list;
public:
    DwellTimeEstimator2(uint16_t _Nch, double _max_dwelltime, float _time_percent_thres);
    double dwelltime();
    double time_of_transition() {return 0;}
    void process(double tstamp, std::vector<float> &ch_pwr);
};


