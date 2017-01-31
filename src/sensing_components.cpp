/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "sensing_components.h"
#include <fstream>

void SpectrogramGenerator::setup_rx_chain(uhd::usrp::multi_usrp::sptr utx)
{
    usrp_tx = utx;
    //create a receive streamer
    std::string wire_format("sc16");
    std::string cpu_format("fc32");
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    uhd::rx_streamer::sptr rx_stream = usrp_tx->get_rx_stream(stream_args);
}

void SpectrogramGenerator::start()
{
    //setup streaming
    std::cout << std::endl;
    std::cout << boost::format("Begin streaming now ..") << std::endl;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = 0; // continuous
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    usrp_tx->issue_stream_cmd(stream_cmd);
}

bool SpectrogramGenerator::recv_fft_pwrs()
{
    size_t num_rx_samps = rx_stream->recv(&(*pwr_estim)[0], pwr_estim->fft_size(), metadata, 5.0);

    //handle the error code
    if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
    {
        std::cout << boost::format("Timeout while streaming") << std::endl;
        return false;
    }
    else if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
    {
        if (overflow_message)
        {
            overflow_message = false;
            std::cerr << boost::format("Got an overflow indication, please reduce sample rate.");
        }
        return true;
    }
    else if (metadata.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
    {
        throw std::runtime_error(str(boost::format(
                                                   "Unexpected error code 0x%x"
                                                   ) % metadata.error_code));
    }

    tspec = metadata.time_spec;
    current_timestamp = tspec.get_real_secs();

    // Always call process because we write on fftBins nBins samples from uhd (see above)
    pwr_estim->process(current_timestamp);
    
    return true;
}

void launch_spectrogram_generator(uhd::usrp::multi_usrp::sptr& usrp_tx, ChannelPowerEstimator* pwr_estim)
{
    SpectrogramGenerator s;
    
    // setup the generator
    s.set_estimator(pwr_estim);
    s.setup_rx_chain(usrp_tx);
    
    // start streaming
    s.start();
    
    // loop
    try 
    {
        while (true)
        {
            boost::this_thread::interruption_point();

            // receive data from USRP
            s.recv_fft_pwrs();
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Receive thread interrupted." << std::endl;
    }
}

//void launch_spectrogram_results_to_file(ChannelPowerEstimator* pwr_estim, std::string filename)
//{
//    std::ofstream of;
//    of.open("temp.bin", std::ios::out | std::ios::binary);
//    
//    if(of.is_open()==false)
//    {
//        std::cout << "Unable to open file." << std::endl;
//        return; // TODO: send a signal
//    }
//    // loop
//    try 
//    {
//        while(true)
//        {
//            double tstamp;
//            std::vector<float> ch_power;
//            //blocks waiting
//            pwr_estim->pop_result(tstamp, ch_power);
//
//            of << ch_power;
//        }
//    }
//    catch(boost::thread_interrupted)
//    {
//        std::cout << "Receive thread interrupted." << std::endl;
//    }
//    
//    of.close();
//    /* Print the top N predictions. */
////    for (size_t i = 0; i < predictions.size(); ++i)
////    {
//    //pwr_estim;
//}