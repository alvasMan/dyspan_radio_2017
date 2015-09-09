#include <complex>
#include <vector>
#include <list>
#include <map>
#include <queue>
#include "fftw3.h"
#include <boost/cstdint.hpp>
#include <sstream>
#include "NoiseFilter3.h"
#include <memory>
#include <utility>


typedef std::complex<float> Cplx;
typedef std::vector<std::complex<float> > CplxVec;
typedef std::vector<std::complex<float> >::iterator CplxVecIt;
typedef std::vector<std::complex<float> >::const_iterator CplxVecConstIt;

class EnergyDetector2
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
    std::vector<int> bin_mask_ref;
    std::vector<float> ch_avg_coeff_ref;
    std::vector<float> tmp_ch_power_ref;
    
    std::vector<MovingAverage<double> > ch_pwr_ma;
    std::vector<MovingWindowMax> ch_pwr_ma_last_outputs;

    std::queue<std::pair<double, std::vector<float> > > results;
    
    
public:
    std::unique_ptr<NoiseFilter3> noise_filter;
    Cplx* fftBins;                       ///< Allocated using fftwf_malloc (SIMD aligned)
    uint16_t nBins;
    
    EnergyDetector2();
    ~EnergyDetector2()
    {
        destroy();
    }

    void set_parameters(uint16_t _avg_win_size = 16, uint16_t fftsize = 512, uint16_t num_channels = 4, float bin_mask_select_perc = 0.5, float bin_mask_ref_perc = 0.5);
    void setup();
    void destroy();
    void push_samples(const std::vector<Cplx> &vec);
    void push_sample(Cplx val);
    void process(double tstamp = 0);
    inline bool result_exists() {
        return results.empty() == false;
    }
    void pop_result(double &tstamp, std::vector<float> &vec);
};