/* -*- c++ -*- */
/* 
 * Copyright 2015 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
#include <string.h>
#include "pktrecv_impl.h"
#define dout d_debug && std::cout

extern "C"{
#include "packetLib.h"
}

namespace gr {
  namespace dbconnect {

    pktrecv::sptr
    pktrecv::make(const std::string &host, int port, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new pktrecv_impl(host, port, debug));
    }

    /*
     * The private constructor
     */
    pktrecv_impl::pktrecv_impl(const std::string &host, int port, bool debug)
      :block("pktrecv",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
		d_debug(debug)
    {
	    message_port_register_in(pmt::mp("in0"));
	    message_port_register_in(pmt::mp("in1"));
	    message_port_register_in(pmt::mp("in2"));
	    message_port_register_in(pmt::mp("in3"));
	    set_msg_handler(pmt::mp("in0"), boost::bind(&pktrecv_impl::put_data, this, _1));
	    set_msg_handler(pmt::mp("in1"), boost::bind(&pktrecv_impl::put_data, this, _1));
	    set_msg_handler(pmt::mp("in2"), boost::bind(&pktrecv_impl::put_data, this, _1));
	    set_msg_handler(pmt::mp("in3"), boost::bind(&pktrecv_impl::put_data, this, _1));

		rx_connect(host, port);
	}

    /*
     * Our virtual destructor.
     */
    pktrecv_impl::~pktrecv_impl()
    {
    }

	void pktrecv_impl:: rx_connect(const std::string &host, int port)
	{
		char * d_host = new char[host.size() + 1];
		std::copy(host.begin(), host.end(), d_host);
		d_host[host.size()] = '\0';

		demoRx = spectrum_init(0);
		spectrum_eror_t retVal = spectrum_connect(demoRx, d_host, port, 0, 0);
		spectrum_errorToText(demoRx, retVal, d_errorBuf, sizeof(d_errorBuf));
		dout << "RX connect:" << d_errorBuf << std::endl;
		d_myRadio = spectrum_getRadioNumber(demoRx);
		dout << "Radio number:" << d_myRadio << std::endl;
		//spectrum_waitForState(demoRx, 3, -1);
		dout << "Stage 3 started" << std::endl;
		delete[] d_host;
	}

	void pktrecv_impl::put_data (pmt::pmt_t msg) {

		assert(pmt::is_pair(msg));
		pmt::pmt_t blob = pmt::cdr(msg);

		size_t data_len = pmt::blob_length(blob);
		assert(data_len);


		std::memcpy(d_packetBuffer, pmt::blob_data(blob), data_len);
		/*
		int j;
		for(j=0;j<data_len;j++){
			dout << std::hex << int(d_packetBuffer[j]) ;
			dout << ",";
			if(j % 32 == 31){
				dout << std::endl;
			}
		}
		dout << std::endl;
		*/
		spectrum_eror_t retVal = spectrum_putPacket(demoRx, d_packetBuffer, data_len);
		spectrum_errorToText(demoRx, retVal, d_errorBuf, data_len);
		dout << "Put pkt" << d_errorBuf << std::endl;

	}

  } /* namespace dbconnect */
} /* namespace gr */

