/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "usrp_components.h"

void USRPReader::setup(const uhd::usrp::multi_usrp::sptr& utx, bool overflows_allowed)
{
    usrp_tx = utx;
    //create a receive streamer
    std::string wire_format("sc16");
    std::string cpu_format("fc32");
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    rx_stream = usrp_tx->get_rx_stream(stream_args);
    crash_on_overflow = overflows_allowed;
}

void USRPReader::start()
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

bool USRPReader::recv_block(std::complex<float>* buf_ptr, int Nfft, double& timestamp)
{
    size_t num_rx_samps = rx_stream->recv(buf_ptr, Nfft, metadata, 5.0);

    //handle the error code
    if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
    {
        std::cout << boost::format("Timeout while streaming") << std::endl;
        return false;
    }
    else if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
    {
        if(crash_on_overflow==true)
            throw std::runtime_error("Samples are corrupted now");
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
    timestamp = tspec.get_real_secs();
    
    return true;
}
