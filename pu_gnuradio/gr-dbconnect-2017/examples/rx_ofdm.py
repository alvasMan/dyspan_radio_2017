#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: OFDM RX Phy
# Generated: Thu Feb  2 17:17:04 2017
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import dbconnect
import sys
import time


class rx_ofdm(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "OFDM RX Phy")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("OFDM RX Phy")
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "rx_ofdm")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        self.usrp_samp_rate = usrp_samp_rate = 10e6
        self.sync_word2 = sync_word2 = [0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 0, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 0, 0, 0, 0, 0] 
        self.sync_word1 = sync_word1 = [0., 0., 0., 0., 0., 0., 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 0., 0., 0., 0., 0.]
        self.samp_rate = samp_rate = 2.5e6
        self.pilot_symbols = pilot_symbols = ((1, 1, 1, -1,),)
        self.pilot_carriers = pilot_carriers = ((-21, -7, 7, 21,),)
        self.packet_len = packet_len = 64
        self.occupied_carriers = occupied_carriers = (range(-26, -21) + range(-20, -7) + range(-6, 0) + range(1, 7) + range(8, 21) + range(22, 27),)
        self.fft_len = fft_len = 64
        self.decim_factor = decim_factor = 5

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_1 = uhd.usrp_source(
        	",".join(("addr=192.168.20.2", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_1.set_samp_rate(usrp_samp_rate)
        self.uhd_usrp_source_1.set_center_freq(1255e6, 0)
        self.uhd_usrp_source_1.set_gain(5, 0)
        self.uhd_usrp_source_1.set_antenna("RX2", 0)
        self.freq_xlating_fft_filter_ccc_0_2 = filter.freq_xlating_fft_filter_ccc(decim_factor, (filter.firdes.low_pass(1,usrp_samp_rate, samp_rate/2.0*0.98, 5000)), -3.75e6, usrp_samp_rate)
        self.freq_xlating_fft_filter_ccc_0_2.set_nthreads(4)
        self.freq_xlating_fft_filter_ccc_0_2.declare_sample_delay(0)
        self.freq_xlating_fft_filter_ccc_0_1 = filter.freq_xlating_fft_filter_ccc(decim_factor, (filter.firdes.low_pass(1,usrp_samp_rate, samp_rate/2.0*0.98, 5000)), 3.75e6, usrp_samp_rate)
        self.freq_xlating_fft_filter_ccc_0_1.set_nthreads(4)
        self.freq_xlating_fft_filter_ccc_0_1.declare_sample_delay(0)
        self.freq_xlating_fft_filter_ccc_0_0 = filter.freq_xlating_fft_filter_ccc(decim_factor, (filter.firdes.low_pass(1,usrp_samp_rate, samp_rate/2.0*0.98, 5000)), -1.25e6, usrp_samp_rate)
        self.freq_xlating_fft_filter_ccc_0_0.set_nthreads(4)
        self.freq_xlating_fft_filter_ccc_0_0.declare_sample_delay(0)
        self.freq_xlating_fft_filter_ccc_0 = filter.freq_xlating_fft_filter_ccc(decim_factor, (filter.firdes.low_pass(1,usrp_samp_rate, samp_rate/2.0*0.98, 5000)), 1.25e6, usrp_samp_rate)
        self.freq_xlating_fft_filter_ccc_0.set_nthreads(4)
        self.freq_xlating_fft_filter_ccc_0.declare_sample_delay(0)
        self.digital_ofdm_rx_0_2 = digital.ofdm_rx(
        	  fft_len=fft_len, cp_len=fft_len/4,
        	  frame_length_tag_key='frame_'+"rx_len",
        	  packet_length_tag_key="rx_len",
        	  occupied_carriers=occupied_carriers,
        	  pilot_carriers=pilot_carriers,
        	  pilot_symbols=pilot_symbols,
        	  sync_word1=sync_word1,
        	  sync_word2=sync_word2,
        	  bps_header=1,
        	  bps_payload=2,
        	  debug_log=False,
        	  scramble_bits=False
        	 )
        self.digital_ofdm_rx_0_1 = digital.ofdm_rx(
        	  fft_len=fft_len, cp_len=fft_len/4,
        	  frame_length_tag_key='frame_'+"rx_len",
        	  packet_length_tag_key="rx_len",
        	  occupied_carriers=occupied_carriers,
        	  pilot_carriers=pilot_carriers,
        	  pilot_symbols=pilot_symbols,
        	  sync_word1=sync_word1,
        	  sync_word2=sync_word2,
        	  bps_header=1,
        	  bps_payload=2,
        	  debug_log=False,
        	  scramble_bits=False
        	 )
        self.digital_ofdm_rx_0_0 = digital.ofdm_rx(
        	  fft_len=fft_len, cp_len=fft_len/4,
        	  frame_length_tag_key='frame_'+"rx_len",
        	  packet_length_tag_key="rx_len",
        	  occupied_carriers=occupied_carriers,
        	  pilot_carriers=pilot_carriers,
        	  pilot_symbols=pilot_symbols,
        	  sync_word1=sync_word1,
        	  sync_word2=sync_word2,
        	  bps_header=1,
        	  bps_payload=2,
        	  debug_log=False,
        	  scramble_bits=False
        	 )
        self.digital_ofdm_rx_0 = digital.ofdm_rx(
        	  fft_len=fft_len, cp_len=fft_len/4,
        	  frame_length_tag_key='frame_'+"rx_len",
        	  packet_length_tag_key="rx_len",
        	  occupied_carriers=occupied_carriers,
        	  pilot_carriers=pilot_carriers,
        	  pilot_symbols=pilot_symbols,
        	  sync_word1=sync_word1,
        	  sync_word2=sync_word2,
        	  bps_header=1,
        	  bps_payload=2,
        	  debug_log=False,
        	  scramble_bits=False
        	 )
        self.dbconnect_pktrecv_0 = dbconnect.pktrecv("127.0.0.1", 5002, False)
        self.blocks_tagged_stream_to_pdu_0_2 = blocks.tagged_stream_to_pdu(blocks.byte_t, "rx_len")
        self.blocks_tagged_stream_to_pdu_0_1 = blocks.tagged_stream_to_pdu(blocks.byte_t, "rx_len")
        self.blocks_tagged_stream_to_pdu_0_0 = blocks.tagged_stream_to_pdu(blocks.byte_t, "rx_len")
        self.blocks_tagged_stream_to_pdu_0 = blocks.tagged_stream_to_pdu(blocks.byte_t, "rx_len")

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0, 'pdus'), (self.dbconnect_pktrecv_0, 'in0'))    
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0_0, 'pdus'), (self.dbconnect_pktrecv_0, 'in1'))    
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0_1, 'pdus'), (self.dbconnect_pktrecv_0, 'in2'))    
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0_2, 'pdus'), (self.dbconnect_pktrecv_0, 'in3'))    
        self.connect((self.digital_ofdm_rx_0, 0), (self.blocks_tagged_stream_to_pdu_0, 0))    
        self.connect((self.digital_ofdm_rx_0_0, 0), (self.blocks_tagged_stream_to_pdu_0_0, 0))    
        self.connect((self.digital_ofdm_rx_0_1, 0), (self.blocks_tagged_stream_to_pdu_0_1, 0))    
        self.connect((self.digital_ofdm_rx_0_2, 0), (self.blocks_tagged_stream_to_pdu_0_2, 0))    
        self.connect((self.freq_xlating_fft_filter_ccc_0, 0), (self.digital_ofdm_rx_0, 0))    
        self.connect((self.freq_xlating_fft_filter_ccc_0_0, 0), (self.digital_ofdm_rx_0_0, 0))    
        self.connect((self.freq_xlating_fft_filter_ccc_0_1, 0), (self.digital_ofdm_rx_0_1, 0))    
        self.connect((self.freq_xlating_fft_filter_ccc_0_2, 0), (self.digital_ofdm_rx_0_2, 0))    
        self.connect((self.uhd_usrp_source_1, 0), (self.freq_xlating_fft_filter_ccc_0, 0))    
        self.connect((self.uhd_usrp_source_1, 0), (self.freq_xlating_fft_filter_ccc_0_0, 0))    
        self.connect((self.uhd_usrp_source_1, 0), (self.freq_xlating_fft_filter_ccc_0_1, 0))    
        self.connect((self.uhd_usrp_source_1, 0), (self.freq_xlating_fft_filter_ccc_0_2, 0))    

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "rx_ofdm")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_usrp_samp_rate(self):
        return self.usrp_samp_rate

    def set_usrp_samp_rate(self, usrp_samp_rate):
        self.usrp_samp_rate = usrp_samp_rate
        self.uhd_usrp_source_1.set_samp_rate(self.usrp_samp_rate)
        self.freq_xlating_fft_filter_ccc_0_2.set_taps((filter.firdes.low_pass(1,self.usrp_samp_rate, self.samp_rate/2.0*0.98, 5000)))
        self.freq_xlating_fft_filter_ccc_0_1.set_taps((filter.firdes.low_pass(1,self.usrp_samp_rate, self.samp_rate/2.0*0.98, 5000)))
        self.freq_xlating_fft_filter_ccc_0_0.set_taps((filter.firdes.low_pass(1,self.usrp_samp_rate, self.samp_rate/2.0*0.98, 5000)))
        self.freq_xlating_fft_filter_ccc_0.set_taps((filter.firdes.low_pass(1,self.usrp_samp_rate, self.samp_rate/2.0*0.98, 5000)))

    def get_sync_word2(self):
        return self.sync_word2

    def set_sync_word2(self, sync_word2):
        self.sync_word2 = sync_word2

    def get_sync_word1(self):
        return self.sync_word1

    def set_sync_word1(self, sync_word1):
        self.sync_word1 = sync_word1

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.freq_xlating_fft_filter_ccc_0_2.set_taps((filter.firdes.low_pass(1,self.usrp_samp_rate, self.samp_rate/2.0*0.98, 5000)))
        self.freq_xlating_fft_filter_ccc_0_1.set_taps((filter.firdes.low_pass(1,self.usrp_samp_rate, self.samp_rate/2.0*0.98, 5000)))
        self.freq_xlating_fft_filter_ccc_0_0.set_taps((filter.firdes.low_pass(1,self.usrp_samp_rate, self.samp_rate/2.0*0.98, 5000)))
        self.freq_xlating_fft_filter_ccc_0.set_taps((filter.firdes.low_pass(1,self.usrp_samp_rate, self.samp_rate/2.0*0.98, 5000)))

    def get_pilot_symbols(self):
        return self.pilot_symbols

    def set_pilot_symbols(self, pilot_symbols):
        self.pilot_symbols = pilot_symbols

    def get_pilot_carriers(self):
        return self.pilot_carriers

    def set_pilot_carriers(self, pilot_carriers):
        self.pilot_carriers = pilot_carriers

    def get_packet_len(self):
        return self.packet_len

    def set_packet_len(self, packet_len):
        self.packet_len = packet_len

    def get_occupied_carriers(self):
        return self.occupied_carriers

    def set_occupied_carriers(self, occupied_carriers):
        self.occupied_carriers = occupied_carriers

    def get_fft_len(self):
        return self.fft_len

    def set_fft_len(self, fft_len):
        self.fft_len = fft_len

    def get_decim_factor(self):
        return self.decim_factor

    def set_decim_factor(self, decim_factor):
        self.decim_factor = decim_factor


def main(top_block_cls=rx_ofdm, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
