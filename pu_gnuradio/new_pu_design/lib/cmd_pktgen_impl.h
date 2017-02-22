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

#ifndef INCLUDED_DBCONNECT_CMD_PKTGEN_IMPL_H
#define INCLUDED_DBCONNECT_CMD_PKTGEN_IMPL_H

#include <dbconnect/cmd_pktgen.h>
extern "C"{
#include "packetLib.h"
}

namespace gr {
  namespace dbconnect {

    class cmd_pktgen_impl : public cmd_pktgen
    {
     private:
	  spectrum* demoTx;
	  char d_errorBuf[32];
	  int d_myRadio;
	  uint8_t d_packetBuffer[1500];
	  bool d_debug;
	  int d_pktsize;

     public:
      cmd_pktgen_impl(const std::string &host, int port, int pktsize, bool debug);
      ~cmd_pktgen_impl();

	  void tx_connect (const std::string &host, int port);
	  void execute_cmd(pmt::pmt_t msg);
	  void send_pkt(int pktcnt);
	  
    };

  } // namespace dbconnect
} // namespace 
#endif /* INCLUDED_DBCONNECT_CMD_PKTGEN_IMPL_H */

