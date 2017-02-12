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
#include "pdu_fillpath_cpp_impl.h"

namespace gr {
  namespace dbconnect {

    pdu_fillpath_cpp::sptr
    pdu_fillpath_cpp::make()
    {
      return gnuradio::get_initial_sptr
        (new pdu_fillpath_cpp_impl());
    }

    /*
     * The private constructor
     */
    pdu_fillpath_cpp_impl::pdu_fillpath_cpp_impl()
      : gr::sync_block("pdu_fillpath_cpp",
              gr::io_signature::make(0, 0,0),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {

		d_curr_len=0;
		d_txdone_cnt=0;
	    message_port_register_in(pmt::mp("in0"));
	    set_msg_handler(pmt::mp("in0"), boost::bind(&pdu_fillpath_cpp_impl::make_frame, this, _1));
	}

    /*
     * Our virtual destructor.
     */
    pdu_fillpath_cpp_impl::~pdu_fillpath_cpp_impl()
    {
    }

	void pdu_fillpath_cpp_impl::make_frame (pmt::pmt_t msg) {
		
		assert(pmt::is_pair(msg));
        d_curr_vect = pmt::cdr(msg);

		d_curr_len = pmt::blob_length(d_curr_vect)/sizeof(gr_complex);
		d_txdone_cnt=0;
		assert(data_len);

	}

    int
    pdu_fillpath_cpp_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        gr_complex *out = (gr_complex *) output_items[0];

		memset(out, 0x00, sizeof(gr_complex) * (noutput_items));
		if(d_curr_len !=0)
		{
		 size_t minval = std::min((int)d_curr_len,noutput_items);
      	 size_t io(0);
      	 const uint8_t* ptr = (uint8_t*) uniform_vector_elements(d_curr_vect, io);
      	 memcpy(out, ptr+(d_txdone_cnt* sizeof(gr_complex)), minval* sizeof(gr_complex));
		 d_txdone_cnt = d_txdone_cnt + minval;
		 //std::cout << "before"<< d_curr_len << "," << d_txdone_cnt << "\n";
		 d_curr_len = std::max(0,(int)(d_curr_len-minval));
		 //std::cout << "after"<< d_curr_len << "\n";
		}

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }

  } /* namespace dbconnect */
} /* namespace gr */

