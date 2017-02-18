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


#ifndef INCLUDED_DBCONNECT_CMD_PKTGEN_H
#define INCLUDED_DBCONNECT_CMD_PKTGEN_H

#include <dbconnect/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace dbconnect {

    /*!
     * \brief <+description of block+>
     * \ingroup dbconnect
     *
     */
    class DBCONNECT_API cmd_pktgen : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<cmd_pktgen> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of dbconnect::cmd_pktgen.
       *
       * To avoid accidental use of raw pointers, dbconnect::cmd_pktgen's
       * constructor is in a private implementation
       * class. dbconnect::cmd_pktgen::make is the public interface for
       * creating new instances.
       */
      static sptr make(const std::string &host, int port, int pktsize, bool debug);
	  virtual void tx_connect (const std::string &host, int port) = 0;
	  virtual void execute_cmd(pmt::pmt_t msg)=0;
	  virtual void send_pkt(int pktcnt)=0;
    };

  } // namespace dbconnect
} // namespace gr

#endif /* INCLUDED_DBCONNECT_CMD_PKTGEN_H */

