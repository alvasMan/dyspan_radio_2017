/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   usrp_components.h
 * Author: connect
 *
 * Created on 17 February 2017, 11:10
 */
#include <uhd/usrp/multi_usrp.hpp>
#include <complex>

#ifndef USRP_COMPONENTS_H
#define USRP_COMPONENTS_H

class USRPReader
{
public:
    void setup(const uhd::usrp::multi_usrp::sptr& utx, bool overflows_allowed = false); ///< Configure the rx_stream
    void start();
    bool recv_block(std::complex<float>* buf_ptr, int Nfft, double &timestamp);
private:
    uhd::usrp::multi_usrp::sptr usrp_tx;
    uhd::rx_streamer::sptr rx_stream;
    bool crash_on_overflow = false;
    bool overflow_message = true;
    uhd::rx_metadata_t metadata;
    uhd::time_spec_t tspec;
};

#endif /* USRP_COMPONENTS_H */

