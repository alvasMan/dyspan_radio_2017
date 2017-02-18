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

#include <gnuradio/io_signature.h>
#include "packet_controller_impl.h"
#include <random>
#define dout  false && std::cout

namespace gr {
	namespace dbconnect {

		packet_controller::sptr
			packet_controller::make(float  samp_rate, const std::vector<int> swtime, int delay_1, int delay_2, 
					int tconst, int mean1, int mean2, int mean3, int seed, int gain_period, 
					const std::vector<int> &gain_vals, const std::vector<int> &scen_list, bool rand_scen)
			{
				return gnuradio::get_initial_sptr
					(new packet_controller_impl(samp_rate, swtime, delay_1, delay_2, 
												tconst, mean1, mean2, mean3, seed, gain_period, 
												gain_vals, scen_list, rand_scen));
			}

		/*
		 * The private constructor
		 */
		packet_controller_impl::packet_controller_impl(float samp_rate, const std::vector<int> swtime, int delay_1, int delay_2, 
				int tconst, int mean1, int mean2, int mean3, int seed, int gain_period, 
				const std::vector<int> &gain_vals, const std::vector<int> &scen_list, bool rand_scen)
			: gr::block("packet_controller",
					gr::io_signature::make(0, 0,0),
					gr::io_signature::make(4, 4, sizeof(gr_complex))),
			d_samp_rate(samp_rate),
			d_delay_1(delay_1),
			d_delay_2(delay_2),
			d_tconst(tconst),
			d_mean1(mean1),
			d_mean2(mean2),
			d_mean3(mean3),
			d_seed(seed),
			d_gain_period(gain_period),
			d_gain_vals(gain_vals),
			d_rand_scen(rand_scen)
		{
			message_port_register_in(pmt::mp("in0"));
			message_port_register_out(pmt::mp("cmd"));
			message_port_register_out(pmt::mp("gcmd"));
			set_msg_handler(pmt::mp("in0"), boost::bind(&packet_controller_impl::get_packet, this, _1));

			d_msg_queue = new gr::messages::msg_queue(0);
			d_gen1 = new std::default_random_engine(d_seed);
			d_gen2 = new std::default_random_engine(d_seed); 
			d_gen3 = new std::default_random_engine(d_seed); 
			d_gen4 = new std::default_random_engine(d_seed); 
			d_gen5 = new std::default_random_engine(d_seed); 
			d_dist1 = new std::uniform_int_distribution<int> (0,3);
			d_dist2 = new std::poisson_distribution<int> (d_mean1);
			d_dist3 = new std::poisson_distribution<int> (d_mean2);
			d_dist4 = new std::poisson_distribution<int> (d_mean3);
			d_dist5 = new std::uniform_int_distribution<int> (0,1);

			d_gain_period_samp = d_gain_period * d_samp_rate * 0.001;

			if(scen_list.size()!=0)
			{
				d_scenearr.resize(scen_list.size());
				d_scenearr = scen_list;
			}

			if(swtime.size()!=0)
			{
				d_swtime.resize(scen_list.size());
				d_swtime = swtime;
			}
			d_swtime_in_samp = d_swtime[d_swcnt] * d_samp_rate * 0.001;

			//randomize scenarios
			if (d_rand_scen)
			{
				srand(d_seed);
				//std::random_shuffle(&d_scenearr[0], &d_scenearr[10]);
				std::random_shuffle(d_scenearr.begin(), d_scenearr.end());
			}

			//select hopping channels
			d_hopping_2_chan[0] = (*d_dist1)(*d_gen1); 
			int schan = (*d_dist1)(*d_gen1);
			while( schan == d_hopping_2_chan[0])
			{
				schan = (*d_dist1)(*d_gen1); 
			}
			d_hopping_2_chan[1] = schan;

			//select occupied channels
			d_occ_2_chan[0] = (*d_dist1)(*d_gen1); 
			schan = (*d_dist1)(*d_gen1);
			while( schan == d_occ_2_chan[0])
			{
				schan = (*d_dist1)(*d_gen1); 
			}
			d_occ_2_chan[1] = schan; 

			d_dist_type = d_scenearr[d_scncnt];
			update_channels(d_dist_type);

		}

		/*
		 * Our virtual destructor.
		 */
		packet_controller_impl::~packet_controller_impl()
		{

			delete d_msg_queue;
			delete d_gen1 ;
			delete d_gen2 ; 
			delete d_gen3 ; 
			delete d_gen4 ; 
			delete d_gen5 ; 
			delete d_dist1;
			delete d_dist2;
			delete d_dist3;
			delete d_dist4;
			delete d_dist5;
		}

		void packet_controller_impl::get_packet(pmt::pmt_t msg) {

			assert(pmt::is_pair(msg));
			d_msg_queue->insert_tail(msg);
			pmt::pmt_t curr_vect = pmt::cdr(msg);
			d_pkt_len = pmt::blob_length(curr_vect)/sizeof(gr_complex);

		}

		void packet_controller_impl::request_cmd(int pktcnt) {

			message_port_pub(pmt::mp("cmd"), pmt::cons(pmt::mp("pktcnt"), pmt::mp(pktcnt)));

		}

		/*
		 * Returns delay for all channels based on the scenario
		 */
		int packet_controller_impl:: get_delay(int dist_type)
		{
			int inter_per;
			switch ( dist_type) {
				case 0:
					inter_per = d_delay_1;
					break;
				case 1:
					inter_per = d_delay_2;
					break;
				case 2:
					inter_per = d_delay_1;
					break;
				case 3:
					inter_per = d_delay_2;
					break;
				case 4:
					inter_per = d_delay_1;
					break;
				case 5:
					inter_per = d_delay_1;
					break;
				case 6:
					inter_per = d_tconst;
					break;
				case 7:
					inter_per = (*d_dist2)(*d_gen2);
					break;
				case 8:
					inter_per = (*d_dist3)(*d_gen3);
					break;
				case 9:
					inter_per = (*d_dist4)(*d_gen4);
					break;
				default:
					inter_per = d_tconst;
					break;
			}

			return inter_per;

		}


		/*
		 * Returns enable for all channels based on the scenario
		 */
		void packet_controller_impl:: update_channels(int dist_type)
		{
			int chan, idx;
			switch (dist_type) {
				case 0:
				case 1:
				case 3:
					d_sel_chan[0] = false;  
					d_sel_chan[1] = false;  
					d_sel_chan[2] = false;  
					d_sel_chan[3] = false;  
					chan = (*d_dist1)(*d_gen1);
					switch(chan){
						case 0:
							d_sel_chan[0] = true;  
							break;
						case 1:
							d_sel_chan[1] = true;  
							break;
						case 2:
							d_sel_chan[2] = true;  
							break;
						case 3:
							d_sel_chan[3] = true;  
							break;
					}
					break;
				case 2:
					idx = (*d_dist5)(*d_gen5);
					d_sel_chan[0] = false;  
					d_sel_chan[1] = false;  
					d_sel_chan[2] = false;  
					d_sel_chan[3] = false;  
					switch(d_hopping_2_chan[idx]){
						case 0:
							d_sel_chan[0] = true;  
							break;
						case 1:
							d_sel_chan[1] = true;  
							break;
						case 2:
							d_sel_chan[2] = true;  
							break;
						case 3:
							d_sel_chan[3] = true;  
							break;
					}
					break;
				case 4:
					d_sel_chan[0] = false;  
					d_sel_chan[1] = false;  
					d_sel_chan[2] = false;  
					d_sel_chan[3] = false;  
					for(int j=0; j<2; j++)
					{
						switch(d_occ_2_chan[j]){
							case 0:
								d_sel_chan[0] = true;  
								break;
							case 1:
								d_sel_chan[1] = true;  
								break;
							case 2:
								d_sel_chan[2] = true;  
								break;
							case 3:
								d_sel_chan[3] = true;  
								break;
						}
					}
					break;
				case 5:
				case 6:
				case 7:
				case 8:
				case 9:
					d_sel_chan[0] = true;  
					d_sel_chan[1] = true;  
					d_sel_chan[2] = true;  
					d_sel_chan[3] = true;  
					break;
				default:
					break;
			}

		}

		int packet_controller_impl::min_delay(void)
		{
			auto delay = std::max_element(d_inter_per,d_inter_per+4);
			int mdelay = *delay;
			for(int i=0; i<4; i++)
			{
				if(d_sel_chan[i])
				{
					if(d_inter_per[i] < mdelay)
						mdelay = d_inter_per[i];
				}
			}
			return mdelay;
		}


		void packet_controller_impl::forecast(int noutput_items,
				gr_vector_int &ninput_items_required)
		{
			unsigned ninputs = ninput_items_required.size ();
			for(unsigned i = 0; i < ninputs; i++)
				ninput_items_required[i] = noutput_items;
		}

		int
			packet_controller_impl::general_work(int noutput_items,
					gr_vector_int &ninput_items,
					gr_vector_const_void_star &input_items,
					gr_vector_void_star &output_items)
			{

				//select scenario based on switching time
				if(d_samp_cnt>=d_swtime_in_samp)
				{
					d_scncnt = (d_scncnt+1)%d_scenearr.size();
					d_samp_cnt=0;
					d_dist_type = d_scenearr[d_scncnt];
					dout << "Inside switching: scenario: " << d_scenearr[d_scncnt] << std::endl;
					//update selection based on scenarios
					update_channels(d_dist_type);
					//make all delays=0 while switching scenarios
					for(int i=0; i<4; i++)
					{
						d_inter_per[i] = 0; 
					}
					d_swcnt = (d_swcnt+1)%d_swtime.size();
					d_swtime_in_samp = d_swtime[d_swcnt] * d_samp_rate * 0.001;
				}
			
				if(d_gsamp_cnt>=d_gain_period_samp)
				{
					d_gsamp_cnt=0;
					message_port_pub(pmt::mp("gcmd"), pmt::cons(pmt::mp("gain"), pmt::mp(d_gain_vals[d_gcnt])));
					d_gcnt= (d_gcnt+1)%d_gain_vals.size();
				}


				//update delays on enabled channels if minimum delay in any channel is 0
				int mdelay = min_delay();

				if(mdelay<=0)
				{
					d_sum=0;
					for(int i=0; i<4; i++)
					{
						if(d_sel_chan[i])
						{
							d_sum+=1;
							if(d_msg_queue->count() < 10)
							{
								request_cmd((10-d_msg_queue->count()));
							}
						}
					}
				}


				//wait for required packets and fill output buffers with data
				if(d_msg_queue->count() >= d_sum)
				{
					if(d_pktmode)
					{
						if (noutput_items < d_pkt_len)
						{
							return 0;
						}
						else
						{
							for(int i=0; i<4; i++)
							{
								memset((gr_complex *) output_items[i], 0x00, sizeof(gr_complex) * (d_pkt_len));
								if(d_sel_chan[i] && d_inter_per[i]==0)
								{
									size_t io(0);
									pmt::pmt_t curr_vect = pmt::cdr(d_msg_queue->delete_head());
									const uint8_t* ptr = (uint8_t*) uniform_vector_elements(curr_vect, io);
									memcpy(output_items[i], ptr, d_pkt_len * sizeof(gr_complex));
									//replace zero delays on enabled channels
									d_inter_per[i] = get_delay(d_dist_type);
								}
							}

							int mdelay = min_delay();
							//convert minimum delay to samples; delay is in ms
							d_delaysamp = (mdelay)*0.001*d_samp_rate;
							d_pktmode = false;
							d_samp_cnt += d_pkt_len;
							d_gsamp_cnt += d_pkt_len;
							consume_each(d_pkt_len);
							//update channels if in hopping mode
							if (d_dist_type ==2 || d_dist_type == 3)
							{
								update_channels(d_dist_type);
							}
							return d_pkt_len;
						}
					}
					else
					{
						size_t minval = std::min((int)d_delaysamp,noutput_items);
						for(int i=0; i<4; i++)
						{
							memset((gr_complex *) output_items[i], 0x00, sizeof(gr_complex) * (minval));
						}
						d_delaysamp = std::max(0,(int)(d_delaysamp-minval));
						if(d_delaysamp == 0)
						{
							d_pktmode = true;
							//find the minimum delay and subtract it from all delays
							int mdelay = min_delay();

							for(int i=0; i<4; i++)
							{
								if(d_sel_chan[i])
								{
									d_inter_per[i] -= mdelay;
								}
							}
						}

						d_samp_cnt += minval;
						d_gsamp_cnt += minval;
						consume_each(minval);
						return minval;
					}

				}
				else
				{
					return 0;
				}


			}

	} /* namespace dbconnect */
} /* namespace gr */

