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


#ifndef INCLUDED_DBCONNECT_PKTRECV_H
#define INCLUDED_DBCONNECT_PKTRECV_H

#include <dbconnect/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace dbconnect {

    /*!
     * \brief <+description of block+>
     * \ingroup dbconnect
     *
     */
    class DBCONNECT_API pktrecv : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<pktrecv> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of dbconnect::pktrecv.
       *
       * To avoid accidental use of raw pointers, dbconnect::pktrecv's
       * constructor is in a private implementation
       * class. dbconnect::pktrecv::make is the public interface for
       * creating new instances.
       */
      static sptr make(const std::string &hosti="127.0.0.1", int port=5002,bool debug=false);
	  virtual void put_data (pmt::pmt_t msg)=0;
	  virtual void rx_connect (const std::string &host, int port)=0;
    };

  } // namespace dbconnect
} // namespace gr

#endif /* INCLUDED_DBCONNECT_PKTRECV_H */

