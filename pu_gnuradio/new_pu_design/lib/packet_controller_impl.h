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

#ifndef INCLUDED_DBCONNECT_PACKET_CONTROLLER_IMPL_H
#define INCLUDED_DBCONNECT_PACKET_CONTROLLER_IMPL_H

#include <dbconnect/packet_controller.h>
#include <gnuradio/messages/msg_queue.h>
#include <random>

namespace gr {
  namespace dbconnect {

    class packet_controller_impl : public packet_controller
    {
     private:
	  bool d_sel_chan[4] = {false, false, false, false};
	  bool d_rand_scen;
	  bool d_pktmode=true;
	  int d_mean1;
	  int d_mean2;
	  int d_mean3;
	  int d_delay_1;
	  int d_delay_2;
	  std::vector<int> d_swtime={15000,};
	  int d_seed;
	  int d_tconst;
	  int d_gain_period;
	  int d_gain_period_samp;
	  float d_samp_rate;
	  int d_pkt_len;
	  int d_scncnt = 0;
	  int d_gcnt = 0;
	  int d_swcnt= 0;
	  int d_sum= 0;
	  
	  
	  int d_samp_cnt=0;
	  int d_gsamp_cnt=0;
	  int d_dist_type=0;
	  int d_swtime_in_samp;
	  int d_delaysamp;
	  std::vector<int> d_scenearr={0,1,2,3,4,5,6,7,8,9};
	  int d_inter_per[4] = {0, 0, 0, 0};
	  int d_hopping_2_chan[2]={0,1};
	  int d_occ_2_chan[2]={0,1};

	  //distributions
	  std::default_random_engine *d_gen1;
	  std::default_random_engine *d_gen2;
	  std::default_random_engine *d_gen3;
	  std::default_random_engine *d_gen4;
	  std::default_random_engine *d_gen5;
	  std::uniform_int_distribution<int> *d_dist1;
	  std::poisson_distribution<int> *d_dist2;
	  std::poisson_distribution<int> *d_dist3;
	  std::poisson_distribution<int> *d_dist4;
	  std::uniform_int_distribution<int> *d_dist5;
	  
	  gr::messages::msg_queue* d_msg_queue;
	  const std::vector<int> d_gain_vals;

     public:
      packet_controller_impl(float samp_rate, const std::vector<int> swtime, int delay_1, int delay_2, int tconst, int mean1, int mean2, int mean3, int seed, int gain_period, const std::vector<int> &gain_vals, const std::vector<int> &scen_list, bool rand_scen);
      ~packet_controller_impl();

	  void get_packet(pmt::pmt_t msg);
	  void request_cmd(int pktcnt); 
	  int  get_delay(int dist_type);
	  void update_channels(int dist_type);
	  int min_delay(void);

	  void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      // Where all the action really happens
      int general_work(int noutput_items,
			  gr_vector_int &ninput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace dbconnect
} // namespace gr

#endif /* INCLUDED_DBCONNECT_PACKET_CONTROLLER_IMPL_H */

