/* -*- c++ -*- */
/* 
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
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

#include <gnuradio/block_detail.h>
#include <gnuradio/io_signature.h>
#include <string.h>
#include "cmd_pktgen_impl.h"
#include "pktgen_impl.h"
#define dout false && std::cout

extern "C"{
#include "packetLib.h"
}

namespace gr {
  namespace dbconnect {

    cmd_pktgen::sptr
    cmd_pktgen::make(const std::string &host, int port, int pktsize, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new cmd_pktgen_impl(host, port, pktsize, debug));
    }

    /*
     * The private constructor
     */
    cmd_pktgen_impl::cmd_pktgen_impl(const std::string &host, int port, int pktsize, bool debug)
      : gr::block("cmd_pktgen",
				gr::io_signature::make(0, 0, 0),
				gr::io_signature::make(0, 0, 0)),
		d_pktsize(pktsize),
		d_debug(debug)
    {
		message_port_register_out(pmt::mp("out0"));
	    message_port_register_in(pmt::mp("cmd"));
	    set_msg_handler(pmt::mp("cmd"), boost::bind(&cmd_pktgen_impl::execute_cmd, this, _1));
		tx_connect(host,port);
	}

    /*
     * Our virtual destructor.
     */
    cmd_pktgen_impl::~cmd_pktgen_impl()
    {
    }

	void cmd_pktgen_impl:: tx_connect(const std::string &host, int port)
	{
		char * d_host = new char[host.size() + 1];
		std::copy(host.begin(), host.end(), d_host);
		d_host[host.size()] = '\0';

		dout << "pktsize:" << d_pktsize << std::endl;
		demoTx = spectrum_init(0);
		//spectrum_eror_t retVal = spectrum_connect(demoTx, "127.0.0.1", d_port, (uint16_t) d_pktsize, 1);
		spectrum_eror_t retVal = spectrum_connect(demoTx, d_host, port, (uint16_t) d_pktsize, 1);
		spectrum_errorToText(demoTx, retVal, d_errorBuf, sizeof(d_errorBuf));
		dout << "TX connect:" << d_errorBuf << std::endl;
		d_myRadio = spectrum_getRadioNumber(demoTx);
		dout << "Radio number:" << d_myRadio << std::endl;
		delete[] d_host;

	}
	
	void cmd_pktgen_impl::execute_cmd(pmt::pmt_t msg) {
		
		assert(pmt::is_pair(msg));
        int pktcnt = pmt::to_long(pmt::cdr(msg));
		send_pkt(pktcnt);

	}
					
	
	void cmd_pktgen_impl:: send_pkt(int pktcnt)
	{
		spectrum_eror_t retVal; 
		pmt::pmt_t pdu;
		dout << "pktcnt" << pktcnt <<std::endl;
		for(int i=0; i<pktcnt; i++)
		{
			retVal = spectrum_getPacket(demoTx, d_packetBuffer, sizeof(d_packetBuffer), -1);
			spectrum_errorToText(demoTx, retVal, d_errorBuf, sizeof(d_errorBuf));
			pdu = pmt::make_blob(d_packetBuffer, d_pktsize);
			dout << "Sending pkt 0" << std::endl;
			message_port_pub( pmt::mp("out0"), pmt::cons( pmt::PMT_NIL, pdu ));
		}
	}


  } /* namespace dbconnect */
} /* namespace gr */

