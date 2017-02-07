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

#ifndef INCLUDED_DBCONNECT_PKTGEN_IMPL_H
#define INCLUDED_DBCONNECT_PKTGEN_IMPL_H

#include <dbconnect/pktgen.h>
extern "C"{
#include "packetLib.h"
}

namespace gr {
  namespace dbconnect {

    class pktgen_impl : public pktgen
    {
     private:
		void run(pktgen_impl *instance);

	  	spectrum* demoTx;
		char d_errorBuf[32];
		int d_myRadio;
		uint8_t d_packetBuffer[1500];

		int d_nmsg_total;
		int d_nmsg_left;
		int d_pktsize;
		int d_tmin;
		int d_tmax;
		int d_tconst;
		int d_swtime;
		int d_mean1;
		int d_mean2;
		int d_mean3;
		int d_seed;
		int d_gain_min;
		int d_gain_max;
		int d_gain_period;
		int d_gain_incr;
		int d_fixed_scenario;
		float d_psc;
		bool d_debug;
		bool d_rand_scen;
		bool d_quit;
		bool d_finished;
		float d_interval;
		pmt::pmt_t d_msg;
		boost::thread *d_thread;
		gr::thread::mutex d_mutex;

     public:
			pktgen_impl(float interval, int pktsize,
					bool quit, bool debug, bool rand_scen, const std::string &host, int port, int tmin, int tmax, int tconst, int mean1, int mean2, int mean3, int swtime, float psc, int seed, int gain_min, int gain_max, int gain_period, int gain_incr, int fixed_scenario = -1);
			virtual ~pktgen_impl();

			void set_nmsg(int nmsg);
			int get_nmsg();

			void set_delay(float delay);
			float get_delay();

			void start_tx();
			void stop_tx();
			bool is_running();
	  		void tx_connect (const std::string &host, int port);
    };

  } // namespace dbconnect
} // namespace gr

#endif /* INCLUDED_DBCONNECT_PKTGEN_IMPL_H */

