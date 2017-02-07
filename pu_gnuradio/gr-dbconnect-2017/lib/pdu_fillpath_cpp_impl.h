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

#ifndef INCLUDED_DBCONNECT_PDU_FILLPATH_CPP_IMPL_H
#define INCLUDED_DBCONNECT_PDU_FILLPATH_CPP_IMPL_H

#include <dbconnect/pdu_fillpath_cpp.h>

namespace gr {
  namespace dbconnect {

    class pdu_fillpath_cpp_impl : public pdu_fillpath_cpp
    {
     private:
      // Nothing to declare in this block.
      pmt::pmt_t           d_curr_vect;
      size_t               d_curr_len;
      size_t               d_txdone_cnt;

     public:
      pdu_fillpath_cpp_impl();
      ~pdu_fillpath_cpp_impl();
	
	  void make_frame (pmt::pmt_t msg);

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace dbconnect
} // namespace gr

#endif /* INCLUDED_DBCONNECT_PDU_FILLPATH_CPP_IMPL_H */

