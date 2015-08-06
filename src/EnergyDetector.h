#include <complex>
#include <vector>
#include <list>
#include <map>
#include <deque>
#include <boost/shared_ptr.hpp>
#include "fftw3.h"
#include <boost/cstdint.hpp>
#include <boost/math/distributions/poisson.hpp>


typedef std::complex<float> Cplx;
typedef std::vector<std::complex<float> > CplxVec;
typedef std::vector<std::complex<float> >::iterator CplxVecIt;
typedef std::vector<std::complex<float> >::const_iterator CplxVecConstIt;

struct val_stats {
    double val_sum;
    uint32_t val_count;
    val_stats() {
        reset();
    }
    inline void push(float val) {
        val_sum += val;
        ++val_count;
    }
    inline float get_avg() {
        return val_sum / val_count;
    }
    inline void reset() {
        val_sum = 0;
        val_count = 0;
    }
};

struct rate_stats {
    uint32_t val_sum;
    uint32_t val_count;
    rate_stats() {
        reset();
    }
    inline void reset() {
        val_sum = 0;
        val_count = 0;
    }
    inline void hit() {
        ++val_sum;
        ++val_count;
    }
    inline void miss() {
        ++val_count;
    }
    inline float get_rate() {
        return ((float)val_sum) / val_count;
    }
};

template <typename T>
class CircularBuffer {
    std::vector<T> buf;
    uint32_t idx;
    uint32_t cur_size;

public:
    CircularBuffer() : idx(0) {}
    CircularBuffer(uint32_t size) {set_size(size);}
    inline void set_size(uint32_t size) {buf.resize(size); cur_size = 0; idx = 0;}
    inline uint32_t size() {
        return cur_size;
    }
    inline void push(const T &val) {
        buf[idx++] = val;
        if(idx >= buf.size()) idx = 0;
        if(cur_size < buf.size()) ++cur_size;
    }
    inline const T& get_val(uint32_t pos) {
        pos = (pos+idx) % size();
        return buf[pos];
    }
};

template <typename T>
class MovingAverage {
    std::vector<T> buf;
    T pwr_sum;
    uint32_t idx;

public:
    MovingAverage() {pwr_sum = 0; idx = 0;}
    MovingAverage(uint32_t size) {set_size(size);}
    inline void set_size(uint32_t size) {buf.resize(size, 0); pwr_sum = 0; idx = 0;}
    inline uint32_t size() {return buf.size();}
    inline void push(const T &val) {
        pwr_sum += val - buf[idx];
        buf[idx++] = val;
        if(idx >= size()) idx = 0;
    }
    inline T get_avg() { return pwr_sum / (float)size(); }
};

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
    std::vector<MovingAverage<float> > ch_pwr_ma;

    std::deque<std::pair<double, std::vector<float> > > results;
    
    
public:
    NoiseFilter *noise_filter;
    Cplx* fftBins;                       ///< Allocated using fftwf_malloc (SIMD aligned)
    uint16_t nBins;
    
    EnergyDetector();
    ~EnergyDetector()
    {
        destroy();
    }

    void set_parameters(uint16_t _avg_win_size, uint16_t num_channels, const std::vector<int> &_bin_mask);
    void set_parameters(uint16_t _avg_win_size, uint16_t num_channels, uint16_t fftsize);
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
    sort_idx_op get_idx_sort_op;
    std::vector<uint16_t> tmp_sort_idx;
    val_stats noise_pwr_stats;
    unsigned int min_noise_count;
    
    float thres;
    
#ifdef NOISE_STATS
    std::vector<val_stats> noise_ch_pwr_stats;
    std::vector<rate_stats> noise_hits_stats;
#endif
    
    inline void filter_as_noise(std::vector<float> &ch_pwr, uint16_t idx) {
#ifdef NOISE_STATS
        noise_ch_pwr_stats[idx].push(ch_pwr[idx]);
        noise_hits_stats[idx].miss();
#endif
        noise_pwr_stats.push(ch_pwr[idx]);
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
        noise_pwr_stats.reset();
    }
    void set_thres(float _thres) {
        thres = _thres;
        reset();
    }
    void filter(std::vector<float> &ch_pwr);
    inline float estimated_noise_floor() {
        return (noise_pwr_stats.val_count > 0) ? noise_pwr_stats.get_avg() : 0;
    }
#ifdef NOISE_STATS
    inline float ch_noise_floor(uint16_t idx) {
        return (noise_ch_pwr_stats[idx].val_count > 0) ? noise_ch_pwr_stats[idx].get_avg() : 0;
    }
    inline float ch_detec_rate(uint16_t idx) {
        return (noise_hits_stats[idx].val_count > 0) ? noise_hits_stats[idx].get_rate() : 0;
    }
#endif
};

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


