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

#ifndef INCLUDED_DBCONNECT_PKTRECV_IMPL_H
#define INCLUDED_DBCONNECT_PKTRECV_IMPL_H

#include <dbconnect/pktrecv.h>

extern "C"{
#include "packetLib.h"
}

namespace gr {
  namespace dbconnect {

    class pktrecv_impl : public pktrecv
    {
     private:
	  char d_errorBuf[32];
	  int d_myRadio;
	  uint8_t d_packetBuffer[1500];
	  bool d_debug;
	  spectrum* demoRx;

     public:
      pktrecv_impl(const std::string &host, int port, bool debug);
      ~pktrecv_impl();
	  void put_data (pmt::pmt_t msg);
	  void rx_connect (const std::string &host, int port);

    };

  } // namespace dbconnect
} // namespace gr

#endif /* INCLUDED_DBCONNECT_PKTRECV_IMPL_H */

