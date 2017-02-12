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


#ifndef INCLUDED_DBCONNECT_PKTGEN_H
#define INCLUDED_DBCONNECT_PKTGEN_H

#include <dbconnect/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace dbconnect {

    /*!
     * \brief <+description of block+>
     * \ingroup dbconnect
     *
     */
    class DBCONNECT_API pktgen : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<pktgen> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of dbconnect::pktgen.
       *
       * To avoid accidental use of raw pointers, dbconnect::pktgen's
       * constructor is in a private implementation
       * class. dbconnect::pktgen::make is the public interface for
       * creating new instances.
       */
	static sptr make(float interval,int pktsize = 1500, bool quit = true, bool debug = false, bool rand_scen = true, const std::string &host="127.0.0.1", int port=5002, 
			         int tmin=5, int tmax=100, int tconst=0, int mean1=20, int mean2=100, int mean3=50, int swtime=60000, float psc=0.05, int seed=6643, int gain_min=10, int gain_max=30, int gain_period=60000, int gain_incr=5, int fixed_scenario=-1, int first_channel = -1, int second_channel = -1);

	virtual void set_nmsg(int nmsg) = 0;
	virtual int get_nmsg() = 0;

	virtual void set_delay(float delay) = 0;
	virtual float get_delay() = 0;

	virtual void start_tx() = 0;
	virtual void stop_tx() = 0;
	virtual bool is_running() = 0;
	virtual void tx_connect (const std::string &host, int port) = 0;
    };

  } // namespace dbconnect
} // namespace gr

#endif /* INCLUDED_DBCONNECT_PKTGEN_H */

